#![no_std]
#![no_main]

use core::hint::black_box;
use defmt_rtt as _;
use embassy_executor::Spawner;
use panic_probe as _;

use defmt::info;
use embassy_time::Instant;
use p256_cortex_m4::{PublicKey, SecretKey};
use rand_core::{CryptoRng, RngCore};

/// Total wall-clock budget for the benchmark (microseconds).
const TARGET_TOTAL_US: u64 = 2_500_000;
/// Number of benchmark phases; each gets an equal share of the budget.
const NUM_PHASES: u64 = 3;

// ---------------------------------------------------------------------------
// Fixed key material for the benchmark ONLY. Never use fixed keys in
// production. These are the known-answer vectors from the crate's own test
// suite (interoperability-checked against RustCrypto's `p256`).
// ---------------------------------------------------------------------------

/// Alice's private key (big-endian scalar bytes).
const PRIV_ALICE: [u8; 32] = [
    0x51, 0x9b, 0x42, 0x3d, 0x71, 0x5f, 0x8b, 0x58, 0x1f, 0x4f, 0xa8, 0xee, 0x59, 0xf4, 0x77, 0x1a, 0x5b, 0x44, 0xc8,
    0x13, 0x0b, 0x4e, 0x3e, 0xac, 0xca, 0x54, 0xa5, 0x6d, 0xda, 0x72, 0xb4, 0x64,
];

/// Bob's public key, X || Y, big-endian (uncompressed SEC1, without tag).
const PUB_BOB: [u8; 64] = [
    0xd7, 0x17, 0xe9, 0x8c, 0xbb, 0x77, 0x38, 0x25, 0x63, 0xfa, 0xc7, 0x53, 0x0c, 0x4c, 0x10, 0xd6, 0xd6, 0x08, 0xaf,
    0x29, 0x83, 0x7c, 0x05, 0x1e, 0x3c, 0x19, 0x12, 0x43, 0xb1, 0xc2, 0x90, 0xdf, 0x03, 0x6d, 0x56, 0x2d, 0xed, 0x21,
    0xbb, 0x37, 0x53, 0xad, 0x13, 0x46, 0x60, 0xd9, 0xeb, 0x13, 0xa6, 0x6c, 0x17, 0x5e, 0x13, 0xf4, 0x55, 0x56, 0x59,
    0x91, 0x6e, 0x78, 0x31, 0x6d, 0xe4, 0x30,
];

/// Arbitrary 32-byte message hash used for the ECDSA benchmark.
const HASH: [u8; 32] = [0x42; 32];

/// Deterministic xorshift64* RNG.
///
/// BENCHMARK ONLY: only used to draw the one-time setup signature nonce.
/// `CryptoRng` is implemented solely to satisfy the API contract for this
/// non-security-sensitive measurement; never use this RNG in production.
struct BenchRng(u64);

impl RngCore for BenchRng {
    fn next_u32(&mut self) -> u32 {
        (self.next_u64() >> 32) as u32
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        let mut chunks = dest.chunks_exact_mut(8);
        for chunk in &mut chunks {
            chunk.copy_from_slice(&self.next_u64().to_le_bytes());
        }
        let rem = chunks.into_remainder();
        if !rem.is_empty() {
            rem.copy_from_slice(&self.next_u64().to_le_bytes()[..rem.len()]);
        }
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
        self.fill_bytes(dest);
        Ok(())
    }
}

impl CryptoRng for BenchRng {}

struct Phase {
    avg_us: u64,
    total_us: u64,
    iters: u32,
}

/// Calibrate `op` with one timed iteration, then run it for `budget_us`
/// worth of iterations, returning the measured average.
fn bench_op(budget_us: u64, mut op: impl FnMut()) -> Phase {
    // Warm-up, then calibrate the per-iteration cost.
    op();
    let t0 = Instant::now();
    op();
    let single_us = t0.elapsed().as_micros().max(1);

    let iters = (budget_us / single_us).clamp(1, u32::MAX as u64) as u32;

    let t1 = Instant::now();
    for _ in 0..iters {
        op();
    }
    let total_us = t1.elapsed().as_micros();
    Phase {
        avg_us: total_us / iters as u64,
        total_us,
        iters,
    }
}

/// Run the full benchmark. Call from an Embassy task once the time driver
/// is running; blocks for ~2.5 s and prints one `defmt` INFO line.
pub async fn run_p256_benchmark() {
    let phase_budget = TARGET_TOTAL_US / NUM_PHASES;

    // ---- one-time key/signature setup -------------------------------------

    let alice = SecretKey::from_bytes(PRIV_ALICE).unwrap();
    let alice_pub = alice.public_key();
    let bob_pub = PublicKey::from_untagged_bytes(&PUB_BOB).unwrap();

    // Sign the benchmark hash once so the verify loop has a valid signature.
    let signature = alice.sign_prehashed(&HASH, &mut BenchRng(0x9E37_79B9_7F4A_7C15));
    defmt::assert!(alice_pub.verify_prehashed(&HASH, &signature));

    // ---- phase 1: TLS 1.3 ECDHE -------------------------------------------
    // One side of an ephemeral ECDHE exchange: ephemeral keygen
    // (base-point scalar mult) + ECDH shared-secret calculation
    // (generic scalar mult + point validation).
    let ecdhe = bench_op(phase_budget, || {
        let eph_pub = black_box(alice.public_key());
        let shared = black_box(alice.agree(black_box(&bob_pub)));
        black_box((eph_pub, shared));
    });

    // ---- phase 2: TLS 1.3 ECDSA verify ------------------------------------
    let verify = bench_op(phase_budget, || {
        black_box(alice_pub.verify_prehashed(black_box(&HASH), black_box(&signature)));
    });

    // ---- phase 3: BLE LESC DH ---------------------------------------------
    // A single ECDH shared-secret calculation, as in LE Secure Connections
    // pairing (dhkey = private_key x peer_public_key).
    let lesc = bench_op(phase_budget, || {
        let dhkey = alice.agree(black_box(&bob_pub));
        black_box(dhkey.as_bytes());
    });

    let total_us = ecdhe.total_us + verify.total_us + lesc.total_us;
    info!(
        "[default bench] TLS1.3 ECDHE ~{} us | TLS1.3 ECDSA verify ~{} us | BLE LESC DH ~{} us | total ~{} us ({} + {} + {} iters)",
        ecdhe.avg_us, verify.avg_us, lesc.avg_us, total_us, ecdhe.iters, verify.iters, lesc.iters,
    );
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let _p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    run_p256_benchmark().await;

    cortex_m::asm::bkpt();
}
