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

    loop {
        info!("high");
        led.set_high();
        Timer::after_millis(500).await;

        info!("low");
        led.set_low();
        Timer::after_millis(500).await;
    }
}
