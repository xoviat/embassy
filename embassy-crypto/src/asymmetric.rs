//! High-level P-256 API for TLS 1.3 and BLE Secure Connections.
//!
//! Thin layer over the [`embassy_crypto_driver::P256Ec`] driver trait only —
//! no `elliptic-curve`/`p256` software EC stack is pulled in, so a HAL or asm
//! backend registering `P256EcImpl` serves this module with minimal code
//! size. The `driver-p256-ec` feature instead provides the driver in
//! software (`driver_rustcrypto`).
//!
//! Implements the `signature` crate traits (`RandomizedDigestSigner`,
//! `DigestVerifier`, `hazmat::PrehashVerifier`) so the keys interoperate
//! with generic signature code.

use digest::typenum::U32;
use digest::{Digest, Update};
use embassy_crypto_driver::{CryptoError, P256AffinePoint, P256EcImpl, P256Scalar};
use signature::rand_core::{CryptoRng, TryCryptoRng};
use signature::{DigestVerifier, RandomizedDigestSigner};

/// Bridge: adapts an infallible RustCrypto RNG (`CryptoRng`) to the driver's
/// [`embassy_crypto_driver::Rng`].
pub struct DriverRng<'a, R: CryptoRng + ?Sized>(&'a mut R);

impl<R: CryptoRng + ?Sized> embassy_crypto_driver::Rng for DriverRng<'_, R> {
    fn rng_fill(&mut self, buf: &mut [u8]) -> Result<(), CryptoError> {
        self.0.fill_bytes(buf);
        Ok(())
    }
}

/// Bridge: adapts a fallible RustCrypto RNG (`TryCryptoRng`) to the driver's
/// [`embassy_crypto_driver::Rng`].
pub struct TryDriverRng<'a, R: TryCryptoRng + ?Sized>(&'a mut R);

impl<R: TryCryptoRng + ?Sized> embassy_crypto_driver::Rng for TryDriverRng<'_, R> {
    fn rng_fill(&mut self, buf: &mut [u8]) -> Result<(), CryptoError> {
        self.0.try_fill_bytes(buf).map_err(|_| CryptoError::HardwareError)
    }
}

/// P-256 private key: canonical scalar `d` in `[1, n-1]`.
#[derive(Clone)]
pub struct SecretKey(P256Scalar);

/// P-256 public key: uncompressed affine point.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct PublicKey(P256AffinePoint);

/// ECDSA/P-256 signature: canonical low-S `(r, s)`, 64 bytes big-endian.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Signature {
    r: P256Scalar,
    s: P256Scalar,
}

/// ECDH shared secret: the X coordinate of the shared point.
pub struct SharedSecret([u8; 32]);

impl Drop for SharedSecret {
    fn drop(&mut self) {
        use digest::zeroize::Zeroize;
        self.0.zeroize();
    }
}

fn digest32<D: Digest<OutputSize = U32>>(digest: D) -> [u8; 32] {
    let out = digest.finalize();
    let mut d = [0u8; 32];
    d.copy_from_slice(AsRef::<[u8]>::as_ref(&out));
    d
}

impl SecretKey {
    /// Generate a fresh random private key.
    pub fn generate<R: CryptoRng + ?Sized>(rng: &mut R) -> Result<Self, CryptoError> {
        let (d, _) = P256EcImpl::generate_keypair(&mut DriverRng(rng))?;
        Ok(Self(d))
    }

    /// Load a private key from its canonical big-endian encoding.
    ///
    /// `bytes` must encode a scalar in `[1, n-1]` (the driver contract
    /// assumes canonical input; only the all-zero scalar is rejected here).
    pub fn from_bytes(bytes: &[u8; 32]) -> Result<Self, CryptoError> {
        if bytes.iter().all(|&b| b == 0) {
            return Err(CryptoError::InvalidKey);
        }
        Ok(Self(P256Scalar(*bytes)))
    }

    /// doc
    pub fn to_bytes(&self) -> [u8; 32] {
        self.0.0
    }

    /// Compute the public key `d * G`.
    pub fn public_key(&self) -> Result<PublicKey, CryptoError> {
        P256EcImpl::public_key(self.0).map(PublicKey)
    }

    /// ECDH: X coordinate of `d * peer`.
    pub fn ecdh(&self, peer: &PublicKey) -> Result<SharedSecret, CryptoError> {
        P256EcImpl::ecdh_shared_secret(self.0, peer.0).map(SharedSecret)
    }

    /// ECDSA-sign a pre-hashed message (`digest` = SHA-256 output). The nonce
    /// comes from `rng`.
    pub fn sign_prehash<R: CryptoRng + ?Sized>(
        &self,
        digest: &[u8; 32],
        rng: &mut R,
    ) -> Result<Signature, CryptoError> {
        let mut rng = DriverRng(rng);
        P256EcImpl::ecdsa_sign(self.0, digest, &mut rng).map(|s| Signature { r: s.r, s: s.s })
    }
}

impl PublicKey {
    /// Wrap uncompressed affine coordinates (big-endian). Check with
    /// [`is_valid`](Self::is_valid) before use if the point is untrusted.
    pub fn from_xy(x: [u8; 32], y: [u8; 32]) -> Self {
        Self(P256AffinePoint { x, y })
    }

    /// doc
    pub fn to_xy(&self) -> ([u8; 32], [u8; 32]) {
        (self.0.x, self.0.y)
    }

    /// On-curve and not the identity.
    pub fn is_valid(&self) -> bool {
        P256EcImpl::validate_point(&self.0)
    }

    /// ECDSA-verify a pre-hashed message.
    pub fn verify_prehash(&self, digest: &[u8; 32], sig: &Signature) -> Result<(), CryptoError> {
        P256EcImpl::ecdsa_verify(
            self.0,
            digest,
            &embassy_crypto_driver::P256Signature { r: sig.r, s: sig.s },
        )
    }
}

impl SharedSecret {
    /// doc
    pub fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }
}

// ---- `signature` crate traits ----
//
// `DigestSigner` is deliberately NOT implemented: ECDSA signing needs nonce
// entropy, and silently sourcing it from a hidden global RNG is exactly the
// failure mode this API design avoids. Use the inherent `sign_prehash` or
// `RandomizedDigestSigner`, which thread the RNG through.

impl<D> RandomizedDigestSigner<D, Signature> for SecretKey
where
    D: Digest<OutputSize = U32> + Update,
{
    fn try_sign_digest_with_rng<R, F>(&self, rng: &mut R, f: F) -> Result<Signature, signature::Error>
    where
        R: TryCryptoRng + ?Sized,
        F: Fn(&mut D) -> Result<(), signature::Error>,
    {
        let mut digest = D::new();
        f(&mut digest)?;
        let d = digest32(digest);
        let mut rng = TryDriverRng(rng);
        P256EcImpl::ecdsa_sign(self.0, &d, &mut rng)
            .map(|s| Signature { r: s.r, s: s.s })
            .map_err(|_| signature::Error::new())
    }
}

impl<D> DigestVerifier<D, Signature> for PublicKey
where
    D: Digest<OutputSize = U32> + Update,
{
    fn verify_digest<F: Fn(&mut D) -> Result<(), signature::Error>>(
        &self,
        f: F,
        signature: &Signature,
    ) -> Result<(), signature::Error> {
        let mut digest = D::new();
        f(&mut digest)?;
        let d = digest32(digest);
        PublicKey::verify_prehash(self, &d, signature).map_err(|_| signature::Error::new())
    }
}

impl signature::hazmat::PrehashVerifier<Signature> for PublicKey {
    fn verify_prehash(&self, prehash: &[u8], signature: &Signature) -> Result<(), signature::Error> {
        let digest: &[u8; 32] = prehash.try_into().map_err(|_| signature::Error::new())?;
        PublicKey::verify_prehash(self, digest, signature).map_err(|_| signature::Error::new())
    }
}

// ---- Signature encoding ----

impl signature::SignatureEncoding for Signature {
    type Repr = [u8; 64];
}

impl From<Signature> for [u8; 64] {
    fn from(sig: Signature) -> [u8; 64] {
        let mut out = [0u8; 64];
        out[..32].copy_from_slice(&sig.r.0);
        out[32..].copy_from_slice(&sig.s.0);
        out
    }
}

impl TryFrom<&[u8]> for Signature {
    type Error = signature::Error;
    fn try_from(bytes: &[u8]) -> Result<Self, Self::Error> {
        if bytes.len() != 64 {
            return Err(signature::Error::new());
        }
        let mut r = [0u8; 32];
        let mut s = [0u8; 32];
        r.copy_from_slice(&bytes[..32]);
        s.copy_from_slice(&bytes[32..]);
        Ok(Signature {
            r: P256Scalar(r),
            s: P256Scalar(s),
        })
    }
}
