#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::pac;
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    let mut led = Output::new(p.PB0, Level::High, Speed::Low);

    pac::RCC.misclpenr().modify(|w| {
        w.set_perlpen(true);
        w.set_xspiphycomplpen(true);
        w.set_dbglpen(true);
    });

    pac::RCC.memlpenr().modify(|w| {
        w.set_bootromlpen(true);
        w.set_vencramlpen(true);
        w.set_flexramlpen(true);
        w.set_axisram2lpen(true);
        w.set_axisram1lpen(true);
        w.set_bkpsramlpen(true);
        w.set_ahbsram1lpen(true);
        w.set_ahbsram2lpen(true);
        w.set_axisram6lpen(true);
        w.set_axisram5lpen(true);
        w.set_axisram4lpen(true);
        w.set_axisram3lpen(true);
        w.set_axisram2lpen(true);
        w.set_axisram1lpen(true);
    });

    pac::RCC.apb2lpenr().modify(|w| {
        w.set_tim9lpen(true);
    });

    let mut delay = cortex_m::delay::Delay::new(unsafe { core::mem::transmute(()) }, 64_000_000);

    loop {
        info!("high");
        led.set_high();
        delay.delay_ms(500);

        // Timer::after_millis(500).await;

        info!("low");
        led.set_low();
        delay.delay_ms(500);
        // Timer::after_millis(500).await;
    }
}
