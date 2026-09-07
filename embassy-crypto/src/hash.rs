//! Hash and HMAC Operations

use digest::Digest;

// ===========================================================================
// Digest macro
// ===========================================================================

#[allow(unused_macros)]
macro_rules! impl_digest {
    (
        $name:ident,
        $drv:path,
        $trait:path,
        $size:ty,
        $block_size:ty,
        $alg_name:expr
    ) => {
        /// RustCrypto `Digest` implementation backed by the embassy-crypto-driver unitrait.
        #[derive(Clone)]
        pub struct $name {
            ctx: <$drv as $trait>::Context,
        }

        impl Default for $name {
            #[inline]
            fn default() -> Self {
                Self { ctx: <$drv>::init() }
            }
        }

        impl ::core::fmt::Debug for $name {
            fn fmt(&self, f: &mut ::core::fmt::Formatter<'_>) -> ::core::fmt::Result {
                f.debug_struct(stringify!($name)).finish_non_exhaustive()
            }
        }

        impl ::digest::OutputSizeUser for $name {
            type OutputSize = $size;
        }

        impl ::digest::Update for $name {
            #[inline]
            fn update(&mut self, data: &[u8]) {
                <$drv>::update(&mut self.ctx, data);
            }
        }

        impl ::digest::FixedOutput for $name {
            #[inline]
            fn finalize_into(self, out: &mut ::digest::Output<Self>) {
                <$drv>::finalize(self.ctx, out.as_mut_slice());
            }
        }

        impl ::digest::Reset for $name {
            #[inline]
            fn reset(&mut self) {
                *self = Self::default();
            }
        }

        impl ::digest::FixedOutputReset for $name {
            #[inline]
            fn finalize_into_reset(&mut self, out: &mut ::digest::Output<Self>) {
                self.clone().finalize_into(out);
                self.reset();
            }
        }

        impl ::digest::HashMarker for $name {}

        impl ::cipher::BlockSizeUser for $name {
            type BlockSize = $block_size;
        }

        impl ::crypto_common::AlgorithmName for $name {
            fn write_alg_name(f: &mut ::core::fmt::Formatter<'_>) -> ::core::fmt::Result {
                f.write_str($alg_name)
            }
        }
    };
}

// ===========================================================================
// HMAC macro
// ===========================================================================

#[allow(unused_macros)]
macro_rules! impl_hmac {
    (
        $name:ident,
        $drv:path,
        $trait:path,
        $key_size:ty,
        $out_size:ty
    ) => {
        /// RustCrypto `Mac` implementation backed by the embassy-crypto-driver HMAC unitrait.
        #[derive(Clone)]
        pub struct $name {
            ctx: <$drv as $trait>::Context,
        }

        impl ::core::fmt::Debug for $name {
            fn fmt(&self, f: &mut ::core::fmt::Formatter<'_>) -> ::core::fmt::Result {
                f.debug_struct(stringify!($name)).finish_non_exhaustive()
            }
        }

        impl ::digest::OutputSizeUser for $name {
            type OutputSize = $out_size;
        }

        impl ::crypto_common::KeySizeUser for $name {
            type KeySize = $key_size;
        }

        impl ::cipher::KeyInit for $name {
            #[inline]
            fn new(key: &::digest::Key<Self>) -> Self {
                Self {
                    ctx: <$drv>::init(key.as_slice()),
                }
            }

            #[inline]
            fn new_from_slice(key: &[u8]) -> Result<Self, ::digest::InvalidLength> {
                Ok(Self {
                    ctx: <$drv>::init(key),
                })
            }
        }

        impl ::digest::Update for $name {
            #[inline]
            fn update(&mut self, data: &[u8]) {
                <$drv>::update(&mut self.ctx, data);
            }
        }

        impl ::digest::FixedOutput for $name {
            #[inline]
            fn finalize_into(self, out: &mut ::digest::Output<Self>) {
                <$drv>::finalize(self.ctx, out.as_mut_slice());
            }
        }

        impl ::digest::MacMarker for $name {}
    };
}

// ===========================================================================
// Digests
// ===========================================================================

#[cfg(not(feature = "driver-md5"))]
impl_digest!(
    Md5,
    embassy_crypto_driver::Md5Impl,
    embassy_crypto_driver::Md5,
    ::generic_array::typenum::U16,
    ::generic_array::typenum::U64,
    "MD5"
);

/// MD5 hasher: the `md-5` crate's software implementation, called directly.
#[cfg(feature = "driver-md5")]
pub type Md5 = ::md5::Md5;

#[cfg(not(feature = "driver-sha1"))]
impl_digest!(
    Sha1,
    embassy_crypto_driver::Sha1Impl,
    embassy_crypto_driver::Sha1,
    ::generic_array::typenum::U20,
    ::generic_array::typenum::U64,
    "SHA-1"
);

/// SHA-1 hasher: the `sha1` crate's software implementation, called directly.
#[cfg(feature = "driver-sha1")]
pub type Sha1 = ::sha1::Sha1;

#[cfg(not(feature = "driver-sha2"))]
impl_digest!(
    Sha224,
    embassy_crypto_driver::Sha224Impl,
    embassy_crypto_driver::Sha224,
    ::generic_array::typenum::U28,
    ::generic_array::typenum::U64,
    "SHA-224"
);

/// SHA-224 hasher: the `sha2` crate's software implementation, called directly.
#[cfg(feature = "driver-sha2")]
pub type Sha224 = ::sha2::Sha224;

#[cfg(not(feature = "driver-sha2"))]
impl_digest!(
    Sha256,
    embassy_crypto_driver::Sha256Impl,
    embassy_crypto_driver::Sha256,
    ::generic_array::typenum::U32,
    ::generic_array::typenum::U64,
    "SHA-256"
);

/// SHA-256 hasher: the `sha2` crate's software implementation, called directly.
#[cfg(feature = "driver-sha2")]
pub type Sha256 = ::sha2::Sha256;

#[cfg(not(feature = "driver-sha2"))]
impl_digest!(
    Sha384,
    embassy_crypto_driver::Sha384Impl,
    embassy_crypto_driver::Sha384,
    ::generic_array::typenum::U48,
    ::generic_array::typenum::U128,
    "SHA-384"
);

/// SHA-384 hasher: the `sha2` crate's software implementation, called directly.
#[cfg(feature = "driver-sha2")]
pub type Sha384 = ::sha2::Sha384;

#[cfg(not(feature = "driver-sha2"))]
impl_digest!(
    Sha512_224,
    embassy_crypto_driver::Sha512_224Impl,
    embassy_crypto_driver::Sha512_224,
    ::generic_array::typenum::U28,
    ::generic_array::typenum::U128,
    "SHA-512/224"
);

/// SHA-512/224 hasher: the `sha2` crate's software implementation, called directly.
#[cfg(feature = "driver-sha2")]
pub type Sha512_224 = ::sha2::Sha512_224;

#[cfg(not(feature = "driver-sha2"))]
impl_digest!(
    Sha512_256,
    embassy_crypto_driver::Sha512_256Impl,
    embassy_crypto_driver::Sha512_256,
    ::generic_array::typenum::U32,
    ::generic_array::typenum::U128,
    "SHA-512/256"
);

/// SHA-512/256 hasher: the `sha2` crate's software implementation, called directly.
#[cfg(feature = "driver-sha2")]
pub type Sha512_256 = ::sha2::Sha512_256;

#[cfg(not(feature = "driver-sha2"))]
impl_digest!(
    Sha512,
    embassy_crypto_driver::Sha512Impl,
    embassy_crypto_driver::Sha512,
    ::generic_array::typenum::U64,
    ::generic_array::typenum::U128,
    "SHA-512"
);

/// SHA-512 hasher: the `sha2` crate's software implementation, called directly.
#[cfg(feature = "driver-sha2")]
pub type Sha512 = ::sha2::Sha512;

// ===========================================================================
// HMACs
// ===========================================================================

#[cfg(all(feature = "driver-hmac-sha1", feature = "driver-sha1"))]
pub type HmacSha1 = ::hmac::Hmac<::sha1::Sha1>;

#[cfg(all(feature = "driver-hmac-sha1", not(feature = "driver-sha1")))]
pub type HmacSha1 = ::hmac::SimpleHmac<Sha1>;

#[cfg(not(feature = "driver-hmac-sha1"))]
impl_hmac!(
    HmacSha1,
    embassy_crypto_driver::HmacSha1Impl,
    embassy_crypto_driver::HmacSha1,
    ::generic_array::typenum::U64,
    ::generic_array::typenum::U20
);

#[cfg(all(feature = "driver-hmac-sha2", feature = "driver-sha2"))]
/// doc
pub type HmacSha224 = ::hmac::Hmac<::sha2::Sha224>;

#[cfg(all(feature = "driver-hmac-sha2", not(feature = "driver-sha2")))]
/// doc
pub type HmacSha224 = ::hmac::SimpleHmac<Sha224>;

#[cfg(not(feature = "driver-hmac-sha2"))]
impl_hmac!(
    HmacSha224,
    embassy_crypto_driver::HmacSha224Impl,
    embassy_crypto_driver::HmacSha224,
    ::generic_array::typenum::U64,
    ::generic_array::typenum::U28
);

#[cfg(all(feature = "driver-hmac-sha2", feature = "driver-sha2"))]
/// doc
pub type HmacSha256 = ::hmac::Hmac<::sha2::Sha256>;

#[cfg(all(feature = "driver-hmac-sha2", not(feature = "driver-sha2")))]
/// doc
pub type HmacSha256 = ::hmac::SimpleHmac<Sha256>;

#[cfg(not(feature = "driver-hmac-sha2"))]
impl_hmac!(
    HmacSha256,
    embassy_crypto_driver::HmacSha256Impl,
    embassy_crypto_driver::HmacSha256,
    ::generic_array::typenum::U64,
    ::generic_array::typenum::U32
);

#[cfg(all(feature = "driver-hmac-sha2", feature = "driver-sha2"))]
/// doc
pub type HmacSha384 = ::hmac::Hmac<::sha2::Sha384>;

#[cfg(all(feature = "driver-hmac-sha2", not(feature = "driver-sha2")))]
/// doc
pub type HmacSha384 = ::hmac::SimpleHmac<Sha384>;

#[cfg(not(feature = "driver-hmac-sha2"))]
impl_hmac!(
    HmacSha384,
    embassy_crypto_driver::HmacSha384Impl,
    embassy_crypto_driver::HmacSha384,
    ::generic_array::typenum::U128,
    ::generic_array::typenum::U48
);

#[cfg(all(feature = "driver-hmac-sha2", feature = "driver-sha2"))]
/// doc
pub type HmacSha512_224 = ::hmac::Hmac<::sha2::Sha512_224>;

#[cfg(all(feature = "driver-hmac-sha2", not(feature = "driver-sha2")))]
/// doc
pub type HmacSha512_224 = ::hmac::SimpleHmac<Sha512_224>;

#[cfg(not(feature = "driver-hmac-sha2"))]
impl_hmac!(
    HmacSha512_224,
    embassy_crypto_driver::HmacSha512_224Impl,
    embassy_crypto_driver::HmacSha512_224,
    ::generic_array::typenum::U128,
    ::generic_array::typenum::U28
);

#[cfg(all(feature = "driver-hmac-sha2", feature = "driver-sha2"))]
/// doc
pub type HmacSha512_256 = ::hmac::Hmac<::sha2::Sha512_256>;

#[cfg(all(feature = "driver-hmac-sha2", not(feature = "driver-sha2")))]
/// doc
pub type HmacSha512_256 = ::hmac::SimpleHmac<Sha512_256>;

#[cfg(not(feature = "driver-hmac-sha2"))]
impl_hmac!(
    HmacSha512_256,
    embassy_crypto_driver::HmacSha512_256Impl,
    embassy_crypto_driver::HmacSha512_256,
    ::generic_array::typenum::U128,
    ::generic_array::typenum::U32
);

#[cfg(all(feature = "driver-hmac-sha2", feature = "driver-sha2"))]
/// doc
pub type HmacSha512 = ::hmac::Hmac<::sha2::Sha512>;

#[cfg(all(feature = "driver-hmac-sha2", not(feature = "driver-sha2")))]
/// doc
pub type HmacSha512 = ::hmac::SimpleHmac<Sha512>;

#[cfg(not(feature = "driver-hmac-sha2"))]
impl_hmac!(
    HmacSha512,
    embassy_crypto_driver::HmacSha512Impl,
    embassy_crypto_driver::HmacSha512,
    ::generic_array::typenum::U128,
    ::generic_array::typenum::U64
);
