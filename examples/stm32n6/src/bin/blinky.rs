#![no_std]
#![no_main]

use cortex_m::peripheral::NVIC;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::{interrupt, pac, Peri};
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::task]
async fn button_task(mut p: ExtiInput<'static>) {
    loop {
        p.wait_for_any_edge().await;
        info!("button pressed!");
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    let mut led = Output::new(p.PG10, Level::High, Speed::Low);
    let button = ExtiInput::new(p.PC13, p.EXTI13, Pull::Up);

    spawner.spawn(button_task(button).unwrap());

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

    let irq = interrupt::TIM9;

    pac::RCC.apb2lpenr().modify(|w| {
        w.set_tim9lpen(true);
    });

    //    pac::TIM9.dier().modify(|w| w.set_ccie(0, true));
    pac::TIM9.dier().modify(|w| w.set_ccie(1, true));
    //
    //    pac::TIM9.cr1().modify(|w| w.set_cen(false));
    //    pac::TIM9.cr1().modify(|w| w.set_cen(true));

    unsafe {
        core::arch::asm!("cpsie i");
    }

    let mut delay = cortex_m::delay::Delay::new(unsafe { core::mem::transmute(()) }, 64_000_000);
    let mut nvic = unsafe { cortex_m::Peripherals::steal() }.NVIC;

    let show_status = || {
        if pac::TIM9.sr().read().uif() {
            info!("uif set");
        } else {
            info!("uif not set");
        }

        if pac::TIM9.sr().read().ccif(0) {
            info!("ccif 0 set");
        } else {
            info!("ccif 0 not set");
        }

        if pac::TIM9.sr().read().ccif(1) {
            info!("ccif 1 set");
        } else {
            info!("ccif 1 not set");
        }

        if NVIC::is_active(irq) {
            info!("irq active");
        } else {
            info!("irq not active");
        }

        if NVIC::is_enabled(irq) {
            info!("irq enabled");
        } else {
            info!("irq not enabled");
        }

        if NVIC::is_pending(irq) {
            info!("irq pending");
        } else {
            info!("irq not pending");
        }
    };

    loop {
        show_status();

        info!("high");
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;

        show_status();

        //        if pac::TIM9.sr().read().ccif(0) {
        //            info!("ccif 0 set");
        //        } else {
        //            info!("ccif 0 not set");
        //        }

        // Timer::after_millis(500).await;

        info!("low");
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
        // Timer::after_millis(500).await;
    }
}
