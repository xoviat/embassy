// required-features: eth, tls
#![no_std]
#![no_main]

#[path = "../common.rs"]
mod common;
use common::*;
use defmt_rtt as _;
use embassy_crypto as _;
use embassy_executor::Spawner;
use embassy_net::StackStorage;
use embassy_stm32::eth::{Ethernet, GenericPhy, PacketQueue, Sma};
use embassy_stm32::peripherals::{ETH, ETH_SMA};
#[cfg(feature = "stop")]
use embassy_stm32::rcc::{StopMode, WakeGuard};
use embassy_stm32::rng::Rng;
use embassy_stm32::{bind_interrupts, eth, peripherals, rng};
use mcu_crypto_asm as _;
use panic_probe as _;
use rand_chacha::ChaCha8Rng;
use rand_core::SeedableRng;
use static_cell::StaticCell;

teleprobe_meta::timeout!(15);

bind_interrupts!(struct Irqs {
    ETH => eth::InterruptHandler<ETH>;
    RNG => rng::InterruptHandler<peripherals::RNG>;
});

type Device = Ethernet<'static, ETH, GenericPhy<Sma<'static, ETH_SMA>>>;

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static>) -> ! {
    runner.run().await
}

#[cfg_attr(
    feature = "stop",
    embassy_executor::main(executor = "embassy_stm32::executor::Executor", entry = "cortex_m_rt::entry")
)]
#[cfg_attr(not(feature = "stop"), embassy_executor::main)]
async fn main(spawner: Spawner) {
    let p = init();
    info!("Hello World!");

    // Random material: 8 bytes seed the network stack, 32 bytes seed a
    // deterministic CSRNG (ChaCha8) for the TLS provider, which needs a
    // rand_core 0.10 CryptoRng.
    let mut rng = Rng::new(p.RNG, Irqs);
    let mut rand_buf = [0; 40];
    rng.fill_bytes(&mut rand_buf);
    let seed = u64::from_le_bytes(rand_buf[..8].try_into().unwrap());
    let tls_rng = ChaCha8Rng::from_seed(rand_buf[8..].try_into().unwrap());

    // Unique MAC id (eth.rs uses 1..=6) so eth and tls tests can run
    // concurrently on different boards on the same LAN.
    let mac_addr = [0x00, 7, 0xDE, 0xAD, 0xBE, 0xEF];

    const PACKET_QUEUE_SIZE: usize = 4;
    static PACKETS: StaticCell<PacketQueue<PACKET_QUEUE_SIZE, PACKET_QUEUE_SIZE>> = StaticCell::new();

    let device = Ethernet::new(
        PACKETS.init(PacketQueue::<PACKET_QUEUE_SIZE, PACKET_QUEUE_SIZE>::new()),
        p.ETH,
        Irqs,
        p.PA1,
        p.PA7,
        p.PC4,
        p.PC5,
        p.PG13,
        #[cfg(not(feature = "stm32h563zi"))]
        p.PB13,
        #[cfg(feature = "stm32h563zi")]
        p.PB15,
        p.PG11,
        mac_addr,
        p.ETH_SMA,
        p.PA2,
        p.PC1,
    );

    static STACK: StaticCell<StackStorage> = StaticCell::new();
    let (stack, runner) = embassy_net::Stack::new(STACK.init(StackStorage::new()), seed);

    static ETH: StaticCell<Device> = StaticCell::new();
    let eth = unwrap!(stack.add_iface(ETH.init(device)));

    eth.set_dhcpv4(Some(Default::default()));

    #[cfg(feature = "stop")]
    let _guard = WakeGuard::new(StopMode::Stop1);

    spawner.spawn(unwrap!(net_task(runner)));

    perf_client::run_tls(
        eth,
        tls_rng,
        perf_client::Expected {
            down_kbps: 100,
            up_kbps: 100,
            updown_kbps: 100,
        },
    )
    .await;

    info!("Test OK");
    cortex_m::asm::bkpt();
}
