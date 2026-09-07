//! High-level asymmetric operations

use embassy_crypto_driver::CryptoError;
use signature::rand_core::{CryptoRng, TryCryptoRng};

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

/// High-level P-256 API for TLS 1.3 and BLE Secure Connections.
///
/// Thin layer over the [`embassy_crypto_driver::P256Ec`] driver trait only —
/// no `elliptic-curve`/`p256` software EC stack is pulled in, so a HAL or asm
/// backend registering `P256EcImpl` serves this module with minimal code
/// size. The `driver-p256-ec` feature instead runs the operations in software
/// directly (via the `p256` crate), without touching the unitrait.
///
/// Implements the `signature` crate traits (`RandomizedDigestSigner`,
/// `DigestVerifier`, `hazmat::PrehashVerifier`) so the keys interoperate
/// with generic signature code.
pub mod p256 {
    use digest::typenum::U32;
    use digest::{Digest, Update};
    #[cfg(not(feature = "driver-p256-ec"))]
    use embassy_crypto_driver::P256EcImpl;
    use embassy_crypto_driver::{CryptoError, P256AffinePoint, P256Scalar};
    use signature::rand_core::{CryptoRng, TryCryptoRng};
    use signature::{DigestVerifier, RandomizedDigestSigner};

    use crate::asymmetric::{DriverRng, TryDriverRng};

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
            #[cfg(not(feature = "driver-p256-ec"))]
            let (d, _) = P256EcImpl::generate_keypair(&mut DriverRng(rng))?;
            #[cfg(feature = "driver-p256-ec")]
            let (d, _) = sw::generate_keypair(&mut DriverRng(rng))?;
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

        /// Serialize the private key to its canonical big-endian encoding.
        pub fn to_bytes(&self) -> [u8; 32] {
            self.0.0
        }

        /// Compute the public key `d * G`.
        pub fn public_key(&self) -> Result<PublicKey, CryptoError> {
            #[cfg(not(feature = "driver-p256-ec"))]
            let p = P256EcImpl::public_key(self.0)?;
            #[cfg(feature = "driver-p256-ec")]
            let p = sw::public_key(self.0)?;
            Ok(PublicKey(p))
        }

        /// ECDH: X coordinate of `d * peer`.
        pub fn ecdh(&self, peer: &PublicKey) -> Result<SharedSecret, CryptoError> {
            #[cfg(not(feature = "driver-p256-ec"))]
            let s = P256EcImpl::ecdh_shared_secret(self.0, peer.0)?;
            #[cfg(feature = "driver-p256-ec")]
            let s = sw::ecdh_shared_secret(self.0, peer.0)?;
            Ok(SharedSecret(s))
        }

        /// ECDSA-sign a pre-hashed message (`digest` = SHA-256 output). The nonce
        /// comes from `rng`.
        pub fn sign_prehash<R: CryptoRng + ?Sized>(
            &self,
            digest: &[u8; 32],
            rng: &mut R,
        ) -> Result<Signature, CryptoError> {
            let mut rng = DriverRng(rng);
            #[cfg(not(feature = "driver-p256-ec"))]
            let s = P256EcImpl::ecdsa_sign(self.0, digest, &mut rng)?;
            #[cfg(feature = "driver-p256-ec")]
            let s = sw::ecdsa_sign(self.0, digest, &mut rng)?;
            Ok(Signature { r: s.r, s: s.s })
        }
    }

    impl PublicKey {
        /// Wrap uncompressed affine coordinates (big-endian). Check with
        /// [`is_valid`](Self::is_valid) before use if the point is untrusted.
        pub fn from_xy(x: [u8; 32], y: [u8; 32]) -> Self {
            Self(P256AffinePoint { x, y })
        }

        /// Uncompressed affine coordinates (big-endian).
        pub fn to_xy(&self) -> ([u8; 32], [u8; 32]) {
            (self.0.x, self.0.y)
        }

        /// On-curve and not the identity.
        pub fn is_valid(&self) -> bool {
            #[cfg(not(feature = "driver-p256-ec"))]
            return P256EcImpl::validate_point(&self.0);
            #[cfg(feature = "driver-p256-ec")]
            return sw::validate_point(&self.0);
        }

        /// ECDSA-verify a pre-hashed message.
        pub fn verify_prehash(&self, digest: &[u8; 32], sig: &Signature) -> Result<(), CryptoError> {
            let sig = embassy_crypto_driver::P256Signature { r: sig.r, s: sig.s };
            #[cfg(not(feature = "driver-p256-ec"))]
            P256EcImpl::ecdsa_verify(self.0, digest, &sig)?;
            #[cfg(feature = "driver-p256-ec")]
            sw::ecdsa_verify(self.0, digest, &sig)?;
            Ok(())
        }
    }

    impl SharedSecret {
        /// Secret bytes. Handle with care.
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
            #[cfg(not(feature = "driver-p256-ec"))]
            let s = P256EcImpl::ecdsa_sign(self.0, &d, &mut rng).map_err(|_| signature::Error::new())?;
            #[cfg(feature = "driver-p256-ec")]
            let s = sw::ecdsa_sign(self.0, &d, &mut rng).map_err(|_| signature::Error::new())?;
            Ok(Signature { r: s.r, s: s.s })
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

    // ===========================================================================
    // Software fallback (driver-p256-ec): RustCrypto `p256` called directly.
    // ===========================================================================

    #[cfg(feature = "driver-p256-ec")]
    mod sw {
        use embassy_crypto_driver::{CryptoError, P256AffinePoint, P256Scalar};

        fn field_bytes(bytes: &[u8; 32]) -> p256::FieldBytes {
            p256::FieldBytes::try_from(&bytes[..]).expect("slice is 32 bytes long")
        }

        fn sec1(p: &P256AffinePoint) -> [u8; 65] {
            let mut out = [0u8; 65];
            out[0] = 0x04;
            out[1..33].copy_from_slice(&p.x);
            out[33..65].copy_from_slice(&p.y);
            out
        }

        fn nonzero_scalar(bytes: &[u8; 32]) -> Result<p256::NonZeroScalar, CryptoError> {
            use p256::elliptic_curve::PrimeField;
            // from_repr rejects >= n; NonZeroScalar::new rejects zero.
            let s = Option::<p256::Scalar>::from(p256::Scalar::from_repr(field_bytes(bytes)))
                .ok_or(CryptoError::InvalidKey)?;
            Option::<p256::NonZeroScalar>::from(p256::NonZeroScalar::new(s)).ok_or(CryptoError::InvalidKey)
        }

        fn point_xy(sk: &p256::SecretKey) -> ([u8; 32], [u8; 32]) {
            use p256::elliptic_curve::sec1::ToSec1Point;
            let ep = sk.public_key().to_sec1_point(false);
            let mut x = [0u8; 32];
            let mut y = [0u8; 32];
            x.copy_from_slice(ep.x().unwrap());
            y.copy_from_slice(ep.y().unwrap());
            (x, y)
        }

        pub fn generate_keypair(
            rng: &mut dyn embassy_crypto_driver::Rng,
        ) -> Result<(P256Scalar, P256AffinePoint), CryptoError> {
            let mut b = [0u8; 32];
            loop {
                rng.rng_fill(&mut b).map_err(|_| CryptoError::HardwareError)?;
                if let Ok(sk) = p256::SecretKey::from_slice(&b) {
                    let (x, y) = point_xy(&sk);
                    return Ok((P256Scalar(b), P256AffinePoint { x, y }));
                }
            }
        }

        pub fn public_key(k: P256Scalar) -> Result<P256AffinePoint, CryptoError> {
            let sk = p256::SecretKey::from_slice(&k.0).map_err(|_| CryptoError::InvalidKey)?;
            let (x, y) = point_xy(&sk);
            Ok(P256AffinePoint { x, y })
        }

        pub fn validate_point(p: &P256AffinePoint) -> bool {
            p256::PublicKey::from_sec1_bytes(&sec1(p)).is_ok()
        }

        pub fn ecdh_shared_secret(k: P256Scalar, peer: P256AffinePoint) -> Result<[u8; 32], CryptoError> {
            let nz = nonzero_scalar(&k.0)?;
            let pk = p256::PublicKey::from_sec1_bytes(&sec1(&peer)).map_err(|_| CryptoError::InvalidKey)?;
            let secret = p256::elliptic_curve::ecdh::diffie_hellman(&nz, pk.as_affine());
            let mut out = [0u8; 32];
            out.copy_from_slice(secret.raw_secret_bytes());
            Ok(out)
        }

        pub fn ecdsa_sign(
            k: P256Scalar,
            digest: &[u8; 32],
            rng: &mut dyn embassy_crypto_driver::Rng,
        ) -> Result<embassy_crypto_driver::P256Signature, CryptoError> {
            let d = nonzero_scalar(&k.0)?;
            let mut nb = [0u8; 32];
            let nonce = loop {
                rng.rng_fill(&mut nb).map_err(|_| CryptoError::HardwareError)?;
                if let Ok(n) = nonzero_scalar(&nb) {
                    break n;
                }
            };
            let (sig, _rid) = ecdsa::hazmat::sign_prehashed::<p256::NistP256>(&d, &nonce, digest)
                .map_err(|_| CryptoError::InvalidSignature)?;
            let sig = sig.normalize_s();
            let (mut r, mut s) = ([0u8; 32], [0u8; 32]);
            r.copy_from_slice(&sig.r().to_bytes());
            s.copy_from_slice(&sig.s().to_bytes());
            Ok(embassy_crypto_driver::P256Signature {
                r: P256Scalar(r),
                s: P256Scalar(s),
            })
        }

        pub fn ecdsa_verify(
            q: P256AffinePoint,
            digest: &[u8; 32],
            sig: &embassy_crypto_driver::P256Signature,
        ) -> Result<(), CryptoError> {
            use ecdsa::signature::hazmat::PrehashVerifier;
            let vk = p256::ecdsa::VerifyingKey::from_sec1_bytes(&sec1(&q)).map_err(|_| CryptoError::InvalidKey)?;
            let signature = p256::ecdsa::Signature::from_scalars(field_bytes(&sig.r.0), field_bytes(&sig.s.0))
                .map_err(|_| CryptoError::InvalidSignature)?;
            vk.verify_prehash(digest, &signature)
                .map_err(|_| CryptoError::InvalidSignature)
        }
    }
}

// ===========================================================================
// P-384: same API, served by the `P384Ec` driver trait.
// ===========================================================================

/// High-level P-384 API for TLS 1.3.
///
/// Thin layer over the [`embassy_crypto_driver::P384Ec`] driver trait only —
/// no `elliptic-curve`/`p384` software EC stack is pulled in, so a HAL or asm
/// backend registering `P384EcImpl` serves this module with minimal code
/// size. The `driver-p384-ec` feature instead runs the operations in software
/// directly (via the `p384` crate), without touching the unitrait.
pub mod p384 {
    use digest::typenum::U48;
    use digest::{Digest, Update};
    #[cfg(not(feature = "driver-p384-ec"))]
    use embassy_crypto_driver::P384EcImpl;
    use embassy_crypto_driver::{CryptoError, P384AffinePoint, P384Scalar};
    use signature::rand_core::{CryptoRng, TryCryptoRng};
    use signature::{DigestVerifier, RandomizedDigestSigner};

    use super::{DriverRng, TryDriverRng};

    /// P-384 private key: canonical scalar `d` in `[1, n-1]`.
    #[derive(Clone)]
    pub struct SecretKey(P384Scalar);

    /// P-384 public key: uncompressed affine point.
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct PublicKey(P384AffinePoint);

    /// ECDSA/P-384 signature: canonical low-S `(r, s)`, 96 bytes big-endian.
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct Signature {
        r: P384Scalar,
        s: P384Scalar,
    }

    /// ECDH shared secret: the X coordinate of the shared point.
    pub struct SharedSecret([u8; 48]);

    impl Drop for SharedSecret {
        fn drop(&mut self) {
            use digest::zeroize::Zeroize;
            self.0.zeroize();
        }
    }

    fn digest48<D: Digest<OutputSize = U48>>(digest: D) -> [u8; 48] {
        let out = digest.finalize();
        let mut d = [0u8; 48];
        d.copy_from_slice(AsRef::<[u8]>::as_ref(&out));
        d
    }

    impl SecretKey {
        /// Generate a fresh random private key.
        pub fn generate<R: CryptoRng + ?Sized>(rng: &mut R) -> Result<Self, CryptoError> {
            #[cfg(not(feature = "driver-p384-ec"))]
            let (d, _) = P384EcImpl::generate_keypair(&mut DriverRng(rng))?;
            #[cfg(feature = "driver-p384-ec")]
            let (d, _) = sw::generate_keypair(&mut DriverRng(rng))?;
            Ok(Self(d))
        }

        /// Load a private key from its canonical big-endian encoding.
        ///
        /// `bytes` must encode a scalar in `[1, n-1]` (the driver contract
        /// assumes canonical input; only the all-zero scalar is rejected here).
        pub fn from_bytes(bytes: &[u8; 48]) -> Result<Self, CryptoError> {
            if bytes.iter().all(|&b| b == 0) {
                return Err(CryptoError::InvalidKey);
            }
            Ok(Self(P384Scalar(*bytes)))
        }

        /// Serialize the private key to its canonical big-endian encoding.
        pub fn to_bytes(&self) -> [u8; 48] {
            self.0.0
        }

        /// Compute the public key `d * G`.
        pub fn public_key(&self) -> Result<PublicKey, CryptoError> {
            #[cfg(not(feature = "driver-p384-ec"))]
            let p = P384EcImpl::public_key(self.0)?;
            #[cfg(feature = "driver-p384-ec")]
            let p = sw::public_key(self.0)?;
            Ok(PublicKey(p))
        }

        /// ECDH: X coordinate of `d * peer`.
        pub fn ecdh(&self, peer: &PublicKey) -> Result<SharedSecret, CryptoError> {
            #[cfg(not(feature = "driver-p384-ec"))]
            let s = P384EcImpl::ecdh_shared_secret(self.0, peer.0)?;
            #[cfg(feature = "driver-p384-ec")]
            let s = sw::ecdh_shared_secret(self.0, peer.0)?;
            Ok(SharedSecret(s))
        }

        /// ECDSA-sign a pre-hashed message (`digest` = SHA-384 output). The
        /// nonce comes from `rng`.
        pub fn sign_prehash<R: CryptoRng + ?Sized>(
            &self,
            digest: &[u8; 48],
            rng: &mut R,
        ) -> Result<Signature, CryptoError> {
            let mut rng = DriverRng(rng);
            #[cfg(not(feature = "driver-p384-ec"))]
            let s = P384EcImpl::ecdsa_sign(self.0, digest, &mut rng)?;
            #[cfg(feature = "driver-p384-ec")]
            let s = sw::ecdsa_sign(self.0, digest, &mut rng)?;
            Ok(Signature { r: s.r, s: s.s })
        }
    }

    impl PublicKey {
        /// Wrap uncompressed affine coordinates (big-endian). Check with
        /// [`is_valid`](Self::is_valid) before use if the point is untrusted.
        pub fn from_xy(x: [u8; 48], y: [u8; 48]) -> Self {
            Self(P384AffinePoint { x, y })
        }

        /// Uncompressed affine coordinates (big-endian).
        pub fn to_xy(&self) -> ([u8; 48], [u8; 48]) {
            (self.0.x, self.0.y)
        }

        /// On-curve and not the identity.
        pub fn is_valid(&self) -> bool {
            #[cfg(not(feature = "driver-p384-ec"))]
            return P384EcImpl::validate_point(&self.0);
            #[cfg(feature = "driver-p384-ec")]
            return sw::validate_point(&self.0);
        }

        /// ECDSA-verify a pre-hashed message.
        pub fn verify_prehash(&self, digest: &[u8; 48], sig: &Signature) -> Result<(), CryptoError> {
            let sig = embassy_crypto_driver::P384Signature { r: sig.r, s: sig.s };
            #[cfg(not(feature = "driver-p384-ec"))]
            P384EcImpl::ecdsa_verify(self.0, digest, &sig)?;
            #[cfg(feature = "driver-p384-ec")]
            sw::ecdsa_verify(self.0, digest, &sig)?;
            Ok(())
        }
    }

    impl SharedSecret {
        /// Secret bytes. Handle with care.
        pub fn as_bytes(&self) -> &[u8; 48] {
            &self.0
        }
    }

    impl<D> RandomizedDigestSigner<D, Signature> for SecretKey
    where
        D: Digest<OutputSize = U48> + Update,
    {
        fn try_sign_digest_with_rng<R, F>(&self, rng: &mut R, f: F) -> Result<Signature, signature::Error>
        where
            R: TryCryptoRng + ?Sized,
            F: Fn(&mut D) -> Result<(), signature::Error>,
        {
            let mut digest = D::new();
            f(&mut digest)?;
            let d = digest48(digest);
            let mut rng = TryDriverRng(rng);
            #[cfg(not(feature = "driver-p384-ec"))]
            let s = P384EcImpl::ecdsa_sign(self.0, &d, &mut rng).map_err(|_| signature::Error::new())?;
            #[cfg(feature = "driver-p384-ec")]
            let s = sw::ecdsa_sign(self.0, &d, &mut rng).map_err(|_| signature::Error::new())?;
            Ok(Signature { r: s.r, s: s.s })
        }
    }

    impl<D> DigestVerifier<D, Signature> for PublicKey
    where
        D: Digest<OutputSize = U48> + Update,
    {
        fn verify_digest<F: Fn(&mut D) -> Result<(), signature::Error>>(
            &self,
            f: F,
            signature: &Signature,
        ) -> Result<(), signature::Error> {
            let mut digest = D::new();
            f(&mut digest)?;
            let d = digest48(digest);
            PublicKey::verify_prehash(self, &d, signature).map_err(|_| signature::Error::new())
        }
    }

    impl signature::hazmat::PrehashVerifier<Signature> for PublicKey {
        fn verify_prehash(&self, prehash: &[u8], signature: &Signature) -> Result<(), signature::Error> {
            let digest: &[u8; 48] = prehash.try_into().map_err(|_| signature::Error::new())?;
            PublicKey::verify_prehash(self, digest, signature).map_err(|_| signature::Error::new())
        }
    }

    impl signature::SignatureEncoding for Signature {
        type Repr = [u8; 96];
    }

    impl From<Signature> for [u8; 96] {
        fn from(sig: Signature) -> [u8; 96] {
            let mut out = [0u8; 96];
            out[..48].copy_from_slice(&sig.r.0);
            out[48..].copy_from_slice(&sig.s.0);
            out
        }
    }

    impl TryFrom<&[u8]> for Signature {
        type Error = signature::Error;
        fn try_from(bytes: &[u8]) -> Result<Self, Self::Error> {
            if bytes.len() != 96 {
                return Err(signature::Error::new());
            }
            let mut r = [0u8; 48];
            let mut s = [0u8; 48];
            r.copy_from_slice(&bytes[..48]);
            s.copy_from_slice(&bytes[48..]);
            Ok(Signature {
                r: P384Scalar(r),
                s: P384Scalar(s),
            })
        }
    }

    // ---- software fallback (driver-p384-ec): RustCrypto `p384` directly ----

    #[cfg(feature = "driver-p384-ec")]
    mod sw {
        use embassy_crypto_driver::{CryptoError, P384AffinePoint, P384Scalar};

        fn field_bytes(bytes: &[u8; 48]) -> p384::FieldBytes {
            p384::FieldBytes::try_from(&bytes[..]).expect("slice is 48 bytes long")
        }

        fn sec1(p: &P384AffinePoint) -> [u8; 97] {
            let mut out = [0u8; 97];
            out[0] = 0x04;
            out[1..49].copy_from_slice(&p.x);
            out[49..97].copy_from_slice(&p.y);
            out
        }

        fn nonzero_scalar(bytes: &[u8; 48]) -> Result<p384::NonZeroScalar, CryptoError> {
            use p384::elliptic_curve::PrimeField;
            let s = Option::<p384::Scalar>::from(p384::Scalar::from_repr(field_bytes(bytes)))
                .ok_or(CryptoError::InvalidKey)?;
            Option::<p384::NonZeroScalar>::from(p384::NonZeroScalar::new(s)).ok_or(CryptoError::InvalidKey)
        }

        fn point_xy(sk: &p384::SecretKey) -> ([u8; 48], [u8; 48]) {
            use p384::elliptic_curve::sec1::ToSec1Point;
            let ep = sk.public_key().to_sec1_point(false);
            let mut x = [0u8; 48];
            let mut y = [0u8; 48];
            x.copy_from_slice(ep.x().unwrap());
            y.copy_from_slice(ep.y().unwrap());
            (x, y)
        }

        pub fn generate_keypair(
            rng: &mut dyn embassy_crypto_driver::Rng,
        ) -> Result<(P384Scalar, P384AffinePoint), CryptoError> {
            let mut b = [0u8; 48];
            loop {
                rng.rng_fill(&mut b).map_err(|_| CryptoError::HardwareError)?;
                if let Ok(sk) = p384::SecretKey::from_slice(&b) {
                    let (x, y) = point_xy(&sk);
                    return Ok((P384Scalar(b), P384AffinePoint { x, y }));
                }
            }
        }

        pub fn public_key(k: P384Scalar) -> Result<P384AffinePoint, CryptoError> {
            let sk = p384::SecretKey::from_slice(&k.0).map_err(|_| CryptoError::InvalidKey)?;
            let (x, y) = point_xy(&sk);
            Ok(P384AffinePoint { x, y })
        }

        pub fn validate_point(p: &P384AffinePoint) -> bool {
            p384::PublicKey::from_sec1_bytes(&sec1(p)).is_ok()
        }

        pub fn ecdh_shared_secret(k: P384Scalar, peer: P384AffinePoint) -> Result<[u8; 48], CryptoError> {
            let nz = nonzero_scalar(&k.0)?;
            let pk = p384::PublicKey::from_sec1_bytes(&sec1(&peer)).map_err(|_| CryptoError::InvalidKey)?;
            let secret = p384::elliptic_curve::ecdh::diffie_hellman(&nz, pk.as_affine());
            let mut out = [0u8; 48];
            out.copy_from_slice(secret.raw_secret_bytes());
            Ok(out)
        }

        pub fn ecdsa_sign(
            k: P384Scalar,
            digest: &[u8; 48],
            rng: &mut dyn embassy_crypto_driver::Rng,
        ) -> Result<embassy_crypto_driver::P384Signature, CryptoError> {
            let d = nonzero_scalar(&k.0)?;
            let mut nb = [0u8; 48];
            let nonce = loop {
                rng.rng_fill(&mut nb).map_err(|_| CryptoError::HardwareError)?;
                if let Ok(n) = nonzero_scalar(&nb) {
                    break n;
                }
            };
            let (sig, _rid) = ecdsa::hazmat::sign_prehashed::<p384::NistP384>(&d, &nonce, digest)
                .map_err(|_| CryptoError::InvalidSignature)?;
            let sig = sig.normalize_s();
            let (mut r, mut s) = ([0u8; 48], [0u8; 48]);
            r.copy_from_slice(&sig.r().to_bytes());
            s.copy_from_slice(&sig.s().to_bytes());
            Ok(embassy_crypto_driver::P384Signature {
                r: P384Scalar(r),
                s: P384Scalar(s),
            })
        }

        pub fn ecdsa_verify(
            q: P384AffinePoint,
            digest: &[u8; 48],
            sig: &embassy_crypto_driver::P384Signature,
        ) -> Result<(), CryptoError> {
            use ecdsa::signature::hazmat::PrehashVerifier;
            let vk = p384::ecdsa::VerifyingKey::from_sec1_bytes(&sec1(&q)).map_err(|_| CryptoError::InvalidKey)?;
            let signature = p384::ecdsa::Signature::from_scalars(field_bytes(&sig.r.0), field_bytes(&sig.s.0))
                .map_err(|_| CryptoError::InvalidSignature)?;
            vk.verify_prehash(digest, &signature)
                .map_err(|_| CryptoError::InvalidSignature)
        }
    }
}

// ===========================================================================
// X25519 (Curve25519) ECDH, served by the `X25519` driver trait.
// ===========================================================================

/// High-level X25519 (Curve25519) ECDH API for TLS 1.3.
///
/// Thin layer over the [`embassy_crypto_driver::X25519`] driver trait only —
/// no `x25519-dalek` software stack is pulled in, so a HAL or asm backend
/// registering `X25519Impl` serves this module with minimal code size. The
/// `driver-x25519` feature instead runs the operations in software directly
/// (via the `x25519-dalek` crate), without touching the unitrait.
pub mod x25519 {
    #[cfg(not(feature = "driver-x25519"))]
    use embassy_crypto_driver::X25519Impl;
    use embassy_crypto_driver::{CryptoError, X25519PrivateKey, X25519PublicKey};
    use signature::rand_core::CryptoRng;

    use super::DriverRng;

    /// X25519 private key: 32-byte scalar, little-endian (unclamped; the
    /// implementation clamps as part of the X25519 function).
    #[derive(Clone)]
    pub struct SecretKey(X25519PrivateKey);

    /// X25519 public key: 32-byte u-coordinate, little-endian (RFC 7748).
    #[derive(Clone, Copy, PartialEq, Eq)]
    pub struct PublicKey(X25519PublicKey);

    /// ECDH shared secret: `X25519(k, peer)`, 32 bytes.
    pub struct SharedSecret([u8; 32]);

    impl Drop for SharedSecret {
        fn drop(&mut self) {
            use digest::zeroize::Zeroize;
            self.0.zeroize();
        }
    }

    impl SecretKey {
        /// Generate a fresh random private key.
        pub fn generate<R: CryptoRng + ?Sized>(rng: &mut R) -> Result<Self, CryptoError> {
            #[cfg(not(feature = "driver-x25519"))]
            let (d, _) = X25519Impl::generate_keypair(&mut DriverRng(rng))?;
            #[cfg(feature = "driver-x25519")]
            let (d, _) = sw::generate_keypair(&mut DriverRng(rng))?;
            Ok(Self(d))
        }

        /// Load a private key from its canonical 32-byte little-endian
        /// encoding. Only the all-zero scalar is rejected.
        pub fn from_bytes(bytes: &[u8; 32]) -> Result<Self, CryptoError> {
            if bytes.iter().all(|&b| b == 0) {
                return Err(CryptoError::InvalidKey);
            }
            Ok(Self(X25519PrivateKey(*bytes)))
        }

        /// Serialize the private key to its canonical encoding.
        pub fn to_bytes(&self) -> [u8; 32] {
            self.0.0
        }

        /// Compute the public key `X25519(k, 9)`.
        pub fn public_key(&self) -> Result<PublicKey, CryptoError> {
            #[cfg(not(feature = "driver-x25519"))]
            let p = X25519Impl::public_key(self.0)?;
            #[cfg(feature = "driver-x25519")]
            let p = sw::public_key(self.0)?;
            Ok(PublicKey(p))
        }

        /// ECDH: `X25519(k, peer)`. X25519 accepts all 32-byte
        /// u-coordinates, so no peer validation is required (RFC 7748).
        pub fn ecdh(&self, peer: &PublicKey) -> Result<SharedSecret, CryptoError> {
            #[cfg(not(feature = "driver-x25519"))]
            let s = X25519Impl::ecdh_shared_secret(self.0, peer.0)?;
            #[cfg(feature = "driver-x25519")]
            let s = sw::ecdh_shared_secret(self.0, peer.0)?;
            Ok(SharedSecret(s))
        }
    }

    impl PublicKey {
        /// Wrap a canonical 32-byte little-endian u-coordinate.
        pub fn from_bytes(bytes: &[u8; 32]) -> Self {
            Self(X25519PublicKey(*bytes))
        }

        /// Serialize the public key to its canonical encoding.
        pub fn to_bytes(&self) -> [u8; 32] {
            self.0.0
        }
    }

    impl SharedSecret {
        /// Secret bytes. Handle with care.
        pub fn as_bytes(&self) -> &[u8; 32] {
            &self.0
        }
    }

    // ---- software fallback (driver-x25519): `x25519-dalek` called directly ----

    #[cfg(feature = "driver-x25519")]
    mod sw {
        use embassy_crypto_driver::{CryptoError, X25519PrivateKey, X25519PublicKey};
        use x25519_dalek::{PublicKey, StaticSecret};

        pub fn generate_keypair(
            rng: &mut dyn embassy_crypto_driver::Rng,
        ) -> Result<(X25519PrivateKey, X25519PublicKey), CryptoError> {
            let mut b = [0u8; 32];
            rng.rng_fill(&mut b).map_err(|_| CryptoError::HardwareError)?;
            let secret = StaticSecret::from(b);
            let public = PublicKey::from(&secret);
            Ok((X25519PrivateKey(b), X25519PublicKey(public.to_bytes())))
        }

        pub fn public_key(k: X25519PrivateKey) -> Result<X25519PublicKey, CryptoError> {
            let secret = StaticSecret::from(k.0);
            Ok(X25519PublicKey(PublicKey::from(&secret).to_bytes()))
        }

        pub fn ecdh_shared_secret(k: X25519PrivateKey, peer: X25519PublicKey) -> Result<[u8; 32], CryptoError> {
            let secret = StaticSecret::from(k.0);
            let shared = secret.diffie_hellman(&PublicKey::from(peer.0));
            Ok(shared.to_bytes())
        }
    }
}
