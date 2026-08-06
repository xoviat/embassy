#![no_std]
#![no_main]
#[path = "../common.rs"]
mod common;

use common::*;
use embassy_executor::Spawner;
use embassy_stm32::dma::{Channel, ChannelInstance, Priority, ReadableRingBuffer, TransferOptions, WritableRingBuffer};
use embassy_stm32::interrupt::typelevel::Binding;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::{RoundTo, Timer};
use embassy_stm32::timer::{GeneralInstance4Channel, UpDma};
use embassy_stm32::{Peri, dma};
use embassy_time::{Duration, Instant, Timer as AsyncTimer};

const RB_SIZE: usize = 64;
const TIMER_FREQ: u32 = 100_000;
const TEST_BUDGET: Duration = Duration::from_millis(7500);
type Word = u16;

#[cfg_attr(
    feature = "stop",
    embassy_executor::main(executor = "embassy_stm32::executor::Executor", entry = "cortex_m_rt::entry")
)]
#[cfg_attr(not(feature = "stop"), embassy_executor::main)]
async fn main(_spawner: Spawner) {
    let p = init();

    info!("========================================");
    info!("DMA RingBuffer HIL Stress Test");
    info!("========================================");

    let deadline = Instant::now() + TEST_BUDGET;

    macro_rules! check_budget {
        () => {
            crate::assert!(Instant::now() < deadline, "Test exceeded 8 s budget");
        };
    }

    let irqs = irqs!(DMA);

    let mut tim_w = peri!(p, TIM_W);
    let mut dma_w = peri!(p, DMA_W);
    let mut tim_r = peri!(p, TIM_R);
    let mut dma_r = peri!(p, DMA_R);

    info!("[1/8] WritableRingBuffer basic...");
    test_writable_basic(tim_w.reborrow(), dma_w.reborrow(), irqs).await;
    check_budget!();

    info!("[2/8] ReadableRingBuffer basic...");
    test_readable_basic(tim_r.reborrow(), dma_r.reborrow(), irqs).await;
    check_budget!();

    info!("[3/8] Wraparound stress...");
    test_wraparound(
        tim_w.reborrow(),
        dma_w.reborrow(),
        tim_r.reborrow(),
        dma_r.reborrow(),
        irqs,
    )
    .await;
    check_budget!();

    info!("[4/8] Overrun detection...");
    test_overrun_detection(tim_r.reborrow(), dma_r.reborrow(), irqs).await;
    check_budget!();

    info!("[5/8] Pause / resume...");
    test_pause_resume(tim_w.reborrow(), dma_w.reborrow(), irqs).await;
    check_budget!();

    info!("[6/8] Drop and recreation...");
    test_drop_recreation(
        tim_w.reborrow(),
        dma_w.reborrow(),
        tim_r.reborrow(),
        dma_r.reborrow(),
        irqs,
    )
    .await;
    check_budget!();

    info!("[7/8] Race condition stress...");
    test_race_conditions(
        tim_w.reborrow(),
        dma_w.reborrow(),
        tim_r.reborrow(),
        dma_r.reborrow(),
        irqs,
    )
    .await;
    check_budget!();

    info!("[8/8] Exact semantics...");
    test_exact_semantics(
        tim_w.reborrow(),
        dma_w.reborrow(),
        tim_r.reborrow(),
        dma_r.reborrow(),
        irqs,
    )
    .await;
    check_budget!();

    info!("========================================");
    info!("ALL TESTS PASSED");
    info!("========================================");
}

// =============================================================================
// HELPERS
// =============================================================================

fn wopts() -> TransferOptions {
    let mut opts = TransferOptions::default();
    opts.priority = Priority::VeryHigh;
    opts
}

fn setup_timer<T: GeneralInstance4Channel>(tim: &mut Timer<'_, T>) {
    tim.set_frequency(Hertz(TIMER_FREQ), RoundTo::Faster);
    tim.set_counting_mode(embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp);
    tim.enable_outputs();
    tim.start();
}

async fn wait_for<F: FnMut() -> bool>(mut f: F, timeout_us: u64) {
    let end = Instant::now() + Duration::from_micros(timeout_us);
    while !f() {
        if Instant::now() > end {
            core::panic!("wait_for timeout");
        }
        AsyncTimer::after(Duration::from_micros(10)).await;
    }
}

// =============================================================================
// PHASE 1: WritableRingBuffer
// =============================================================================

async fn test_writable_basic(
    tim: Peri<'_, peris::TIM_W>,
    dma: Peri<'_, peris::DMA_W>,
    irq: impl Binding<<peris::DMA_W as ChannelInstance>::Interrupt, dma::InterruptHandler<peris::DMA_W>>,
) {
    let mut tim = Timer::new(tim);
    let ccr_addr = tim.regs_gp16().ccr(0).as_ptr() as *mut Word;
    let mut dma_buf = [0 as Word; RB_SIZE];

    let req = <peris::DMA_W as UpDma<peris::TIM_W>>::request(&*dma);
    let mut ch = Channel::new(dma, irq);

    setup_timer(&mut tim);
    tim.enable_update_dma(true);

    let mut rb = unsafe { WritableRingBuffer::new(ch.reborrow(), req, ccr_addr, &mut dma_buf, wopts()) };

    let prefill: [Word; RB_SIZE / 2] = core::array::from_fn(|i| (i + 1) as Word);
    let (written, remaining) = rb.write(&prefill).unwrap();
    crate::assert_eq!(written, RB_SIZE / 2);
    crate::assert_eq!(remaining, RB_SIZE / 2);
    crate::assert_eq!(rb.len().unwrap(), RB_SIZE / 2);
    crate::assert_eq!(rb.capacity(), RB_SIZE);

    rb.start();
    crate::assert!(rb.is_running());

    AsyncTimer::after(Duration::from_millis(5)).await;

    let mut seq = 0 as Word;
    for _ in 0..50 {
        let chunk = [seq.wrapping_add(1); 8];
        let (w, _) = rb.write(&chunk).unwrap();
        crate::assert_eq!(w, 8);
        seq = seq.wrapping_add(1);
        AsyncTimer::after(Duration::from_micros(50)).await;
    }

    rb.request_pause();
    wait_for(|| !rb.is_running(), 10_000).await;

    info!("WritableRingBuffer basic test passed");
}

// =============================================================================
// PHASE 2: ReadableRingBuffer
// =============================================================================

async fn test_readable_basic(
    tim: Peri<'_, peris::TIM_R>,
    dma: Peri<'_, peris::DMA_R>,
    irq: impl Binding<<peris::DMA_R as ChannelInstance>::Interrupt, dma::InterruptHandler<peris::DMA_R>>,
) {
    let mut tim = Timer::new(tim);
    let cnt_addr = tim.regs_core().cnt().as_ptr() as *mut Word;
    let mut dma_buf = [0 as Word; RB_SIZE];

    let req = <peris::DMA_R as UpDma<peris::TIM_R>>::request(&*dma);
    let mut ch = Channel::new(dma, irq);

    setup_timer(&mut tim);
    tim.enable_update_dma(true);

    let mut rb = unsafe { ReadableRingBuffer::new(ch.reborrow(), req, cnt_addr, &mut dma_buf, wopts()) };
    rb.start();
    crate::assert!(rb.is_running());

    AsyncTimer::after(Duration::from_millis(5)).await;

    let mut read_buf = [0 as Word; RB_SIZE / 2];
    let (len, _) = rb.read(&mut read_buf).unwrap();
    info!("ReadableRingBuffer read {} items", len);
    crate::assert!(len > 0);
    crate::assert!(len <= RB_SIZE / 2);

    let arr = tim.get_max_compare_value() as Word;
    for i in 0..len {
        crate::assert!(read_buf[i] <= arr, "counter value out of range");
    }

    let len_after = rb.len().unwrap();
    crate::assert!(len_after < RB_SIZE);

    rb.clear();
    crate::assert_eq!(rb.len().unwrap(), 0);

    rb.request_reset();
    wait_for(|| !rb.is_running(), 10_000).await;

    info!("ReadableRingBuffer basic test passed");
}

// =============================================================================
// PHASE 3: Wraparound stress
// =============================================================================

async fn test_wraparound(
    tim_w: Peri<'_, peris::TIM_W>,
    dma_w: Peri<'_, peris::DMA_W>,
    tim_r: Peri<'_, peris::TIM_R>,
    dma_r: Peri<'_, peris::DMA_R>,
    irq: impl Binding<<peris::DMA_R as ChannelInstance>::Interrupt, dma::InterruptHandler<peris::DMA_R>>
    + Binding<<peris::DMA_W as ChannelInstance>::Interrupt, dma::InterruptHandler<peris::DMA_W>>,
) {
    const SMALL: usize = 16;

    let mut tim_w = Timer::new(tim_w);
    let ccr_addr = tim_w.regs_gp16().ccr(0).as_ptr() as *mut Word;
    let mut buf_w = [0 as Word; SMALL];
    let req_w = <peris::DMA_W as UpDma<peris::TIM_W>>::request(&*dma_w);
    let mut ch_w = Channel::new(dma_w, irq);
    setup_timer(&mut tim_w);
    tim_w.enable_update_dma(true);
    let mut rb_w = unsafe { WritableRingBuffer::new(ch_w.reborrow(), req_w, ccr_addr, &mut buf_w, wopts()) };
    rb_w.start();

    let mut tim_r = Timer::new(tim_r);
    let cnt_addr = tim_r.regs_core().cnt().as_ptr() as *mut Word;
    let mut buf_r = [0 as Word; SMALL];
    let req_r = <peris::DMA_R as UpDma<peris::TIM_R>>::request(&*dma_r);
    let mut ch_r = Channel::new(dma_r, irq);
    setup_timer(&mut tim_r);
    tim_r.enable_update_dma(true);
    let mut rb_r = unsafe { ReadableRingBuffer::new(ch_r.reborrow(), req_r, cnt_addr, &mut buf_r, wopts()) };
    rb_r.start();

    let mut write_seq = 0 as Word;
    let mut read_buf = [0 as Word; 8];
    for lap in 0..300 {
        let _ = rb_w.write(&[write_seq; 4]);
        write_seq = write_seq.wrapping_add(1);
        let _ = rb_r.read(&mut read_buf);

        if lap % 50 == 0 {
            AsyncTimer::after(Duration::from_micros(50)).await;
        }
    }

    rb_w.request_pause();
    rb_r.request_reset();
    wait_for(|| !rb_w.is_running(), 10_000).await;
    wait_for(|| !rb_r.is_running(), 10_000).await;

    info!("Wraparound stress test passed");
}

// =============================================================================
// PHASE 4: Overrun detection
// =============================================================================

async fn test_overrun_detection(
    tim: Peri<'_, peris::TIM_R>,
    dma: Peri<'_, peris::DMA_R>,
    irq: impl Binding<<peris::DMA_R as ChannelInstance>::Interrupt, dma::InterruptHandler<peris::DMA_R>>,
) {
    const SMALL: usize = 16;
    let mut tim = Timer::new(tim);
    let cnt_addr = tim.regs_core().cnt().as_ptr() as *mut Word;
    let mut buf = [0 as Word; SMALL];
    let req = <peris::DMA_R as UpDma<peris::TIM_R>>::request(&*dma);
    let mut ch = Channel::new(dma, irq);

    setup_timer(&mut tim);
    tim.set_frequency(Hertz(500_000), RoundTo::Faster);
    tim.enable_update_dma(true);

    let mut rb = unsafe { ReadableRingBuffer::new(ch.reborrow(), req, cnt_addr, &mut buf, wopts()) };
    rb.start();

    AsyncTimer::after(Duration::from_millis(30)).await;

    let mut read_buf = [0 as Word; 4];
    let res = rb.read(&mut read_buf);
    match res {
        Ok((len, _)) => info!("Read succeeded, len={}", len),
        Err(e) => info!("Overrun detected (expected): {:?}", e),
    }

    let len = rb.read_latest(&mut read_buf);
    crate::assert!(len <= read_buf.len());

    rb.clear();
    crate::assert_eq!(rb.len().unwrap(), 0);

    rb.request_reset();
    wait_for(|| !rb.is_running(), 10_000).await;

    info!("Overrun detection test passed");
}

// =============================================================================
// PHASE 5: Pause / resume
// =============================================================================

async fn test_pause_resume(
    tim: Peri<'_, peris::TIM_W>,
    dma: Peri<'_, peris::DMA_W>,
    irq: impl Binding<<peris::DMA_W as ChannelInstance>::Interrupt, dma::InterruptHandler<peris::DMA_W>>,
) {
    let mut tim = Timer::new(tim);
    let ccr_addr = tim.regs_gp16().ccr(0).as_ptr() as *mut Word;
    let mut buf = [0 as Word; RB_SIZE];
    let req = <peris::DMA_W as UpDma<peris::TIM_W>>::request(&*dma);
    let mut ch = Channel::new(dma, irq);

    setup_timer(&mut tim);
    tim.enable_update_dma(true);

    let mut rb = unsafe { WritableRingBuffer::new(ch.reborrow(), req, ccr_addr, &mut buf, wopts()) };

    rb.start();
    crate::assert!(rb.is_running());

    rb.request_pause();
    wait_for(|| !rb.is_running(), 10_000).await;

    let (w, _) = rb.write(&[0xABCD as Word; 10]).unwrap();
    crate::assert_eq!(w, 10);
    crate::assert_eq!(rb.len().unwrap(), 10);

    rb.start();
    crate::assert!(rb.is_running());
    AsyncTimer::after(Duration::from_millis(2)).await;

    rb.request_pause();
    wait_for(|| !rb.is_running(), 10_000).await;

    info!("Pause/resume test passed");
}

// =============================================================================
// PHASE 6: Drop and recreation
// =============================================================================

async fn test_drop_recreation(
    mut tim_w: Peri<'_, peris::TIM_W>,
    mut dma_w: Peri<'_, peris::DMA_W>,
    mut tim_r: Peri<'_, peris::TIM_R>,
    mut dma_r: Peri<'_, peris::DMA_R>,
    irq: impl Binding<<peris::DMA_R as ChannelInstance>::Interrupt, dma::InterruptHandler<peris::DMA_R>>
    + Binding<<peris::DMA_W as ChannelInstance>::Interrupt, dma::InterruptHandler<peris::DMA_W>>,
) {
    // WritableRingBuffer — first life
    {
        let mut tim = Timer::new(tim_w.reborrow());
        let ccr_addr = tim.regs_gp16().ccr(0).as_ptr() as *mut Word;
        let mut buf = [0 as Word; RB_SIZE];
        let req = <peris::DMA_W as UpDma<peris::TIM_W>>::request(&*dma_w);
        let mut ch = Channel::new(dma_w.reborrow(), irq);
        setup_timer(&mut tim);
        tim.enable_update_dma(true);
        let mut rb = unsafe { WritableRingBuffer::new(ch.reborrow(), req, ccr_addr, &mut buf, wopts()) };
        rb.start();
        AsyncTimer::after(Duration::from_millis(2)).await;
        rb.request_pause();
        wait_for(|| !rb.is_running(), 10_000).await;
    }

    // Second life on same hardware
    {
        let mut tim = Timer::new(tim_w.reborrow());
        let ccr_addr = tim.regs_gp16().ccr(0).as_ptr() as *mut Word;
        let mut buf = [0 as Word; RB_SIZE];
        let req = <peris::DMA_W as UpDma<peris::TIM_W>>::request(&*dma_w);
        let mut ch = Channel::new(dma_w.reborrow(), irq);
        setup_timer(&mut tim);
        tim.enable_update_dma(true);
        let mut rb = unsafe { WritableRingBuffer::new(ch.reborrow(), req, ccr_addr, &mut buf, wopts()) };
        rb.start();
        let (w, _) = rb.write(&[0x1234 as Word; 20]).unwrap();
        crate::assert_eq!(w, 20);
        AsyncTimer::after(Duration::from_millis(2)).await;
        rb.request_pause();
        wait_for(|| !rb.is_running(), 10_000).await;
    }

    // ReadableRingBuffer — first life
    {
        let mut tim = Timer::new(tim_r.reborrow());
        let cnt_addr = tim.regs_core().cnt().as_ptr() as *mut Word;
        let mut buf = [0 as Word; RB_SIZE];
        let req = <peris::DMA_R as UpDma<peris::TIM_R>>::request(&*dma_r);
        let mut ch = Channel::new(dma_r.reborrow(), irq);
        setup_timer(&mut tim);
        tim.enable_update_dma(true);
        let mut rb = unsafe { ReadableRingBuffer::new(ch.reborrow(), req, cnt_addr, &mut buf, wopts()) };
        rb.start();
        AsyncTimer::after(Duration::from_millis(2)).await;
        rb.request_reset();
        wait_for(|| !rb.is_running(), 10_000).await;
    }

    // Second life
    {
        let mut tim = Timer::new(tim_r.reborrow());
        let cnt_addr = tim.regs_core().cnt().as_ptr() as *mut Word;
        let mut buf = [0 as Word; RB_SIZE];
        let req = <peris::DMA_R as UpDma<peris::TIM_R>>::request(&*dma_r);
        let mut ch = Channel::new(dma_r.reborrow(), irq);
        setup_timer(&mut tim);
        tim.enable_update_dma(true);
        let mut rb = unsafe { ReadableRingBuffer::new(ch.reborrow(), req, cnt_addr, &mut buf, wopts()) };
        rb.start();
        let mut tmp = [0 as Word; 4];
        let _ = rb.read(&mut tmp);
        rb.request_reset();
        wait_for(|| !rb.is_running(), 10_000).await;
    }

    info!("Drop and recreation test passed");
}

// =============================================================================
// PHASE 7: Race-condition stress
// =============================================================================

async fn test_race_conditions(
    tim_w: Peri<'_, peris::TIM_W>,
    dma_w: Peri<'_, peris::DMA_W>,
    tim_r: Peri<'_, peris::TIM_R>,
    dma_r: Peri<'_, peris::DMA_R>,
    irq: impl Binding<<peris::DMA_R as ChannelInstance>::Interrupt, dma::InterruptHandler<peris::DMA_R>>
    + Binding<<peris::DMA_W as ChannelInstance>::Interrupt, dma::InterruptHandler<peris::DMA_W>>,
) {
    let mut tim_w = Timer::new(tim_w);
    let ccr_addr = tim_w.regs_gp16().ccr(0).as_ptr() as *mut Word;
    let mut buf_w = [0 as Word; 32];
    let req_w = <peris::DMA_W as UpDma<peris::TIM_W>>::request(&*dma_w);
    let mut ch_w = Channel::new(dma_w, irq);
    setup_timer(&mut tim_w);
    tim_w.enable_update_dma(true);
    let mut rb_w = unsafe { WritableRingBuffer::new(ch_w.reborrow(), req_w, ccr_addr, &mut buf_w, wopts()) };
    rb_w.start();

    let mut tim_r = Timer::new(tim_r);
    let cnt_addr = tim_r.regs_core().cnt().as_ptr() as *mut Word;
    let mut buf_r = [0 as Word; 32];
    let req_r = <peris::DMA_R as UpDma<peris::TIM_R>>::request(&*dma_r);
    let mut ch_r = Channel::new(dma_r, irq);
    setup_timer(&mut tim_r);
    tim_r.enable_update_dma(true);
    let mut rb_r = unsafe { ReadableRingBuffer::new(ch_r.reborrow(), req_r, cnt_addr, &mut buf_r, wopts()) };
    rb_r.start();

    let mut write_seq = 0 as Word;
    let mut read_buf = [0 as Word; 6];
    let mut total_read = 0usize;
    let mut total_written = 0usize;

    for _ in 0..400 {
        let chunk = [write_seq; 3];
        if let Ok((w, _)) = rb_w.write(&chunk) {
            total_written += w;
            write_seq = write_seq.wrapping_add(1);
        }

        AsyncTimer::after(Duration::from_ticks(0)).await;

        if let Ok((r, _)) = rb_r.read(&mut read_buf) {
            total_read += r;
        }

        if total_written % 100 == 0 {
            let _ = rb_w.len();
            let _ = rb_r.len();
        }
    }

    info!("Race stress totals: written={}, read={}", total_written, total_read);

    rb_w.request_pause();
    rb_r.request_reset();
    wait_for(|| !rb_w.is_running(), 10_000).await;
    wait_for(|| !rb_r.is_running(), 10_000).await;

    info!("Race condition stress test passed");
}

// =============================================================================
// PHASE 8: Exact read/write semantics
// =============================================================================

async fn test_exact_semantics(
    tim_w: Peri<'_, peris::TIM_W>,
    dma_w: Peri<'_, peris::DMA_W>,
    tim_r: Peri<'_, peris::TIM_R>,
    dma_r: Peri<'_, peris::DMA_R>,
    irq: impl Binding<<peris::DMA_R as ChannelInstance>::Interrupt, dma::InterruptHandler<peris::DMA_R>>
    + Binding<<peris::DMA_W as ChannelInstance>::Interrupt, dma::InterruptHandler<peris::DMA_W>>,
) {
    // WritableRingBuffer::write_exact
    {
        let mut tim = Timer::new(tim_w);
        let ccr_addr = tim.regs_gp16().ccr(0).as_ptr() as *mut Word;
        let mut buf = [0 as Word; RB_SIZE];
        let req = <peris::DMA_W as UpDma<peris::TIM_W>>::request(&*dma_w);
        let mut ch = Channel::new(dma_w, irq);
        setup_timer(&mut tim);
        tim.enable_update_dma(true);
        let mut rb = unsafe { WritableRingBuffer::new(ch.reborrow(), req, ccr_addr, &mut buf, wopts()) };
        rb.start();

        let data = [0xBEEF as Word; 20];
        rb.write_exact(&data).await.unwrap();

        let huge = [0 as Word; RB_SIZE + 1];
        crate::assert!(
            rb.write_exact(&huge).await.is_err(),
            "write_exact should fail when over-capacity"
        );

        rb.request_pause();
        wait_for(|| !rb.is_running(), 10_000).await;
    }

    // ReadableRingBuffer::read_exact
    {
        let mut tim = Timer::new(tim_r);
        let cnt_addr = tim.regs_core().cnt().as_ptr() as *mut Word;
        let mut buf = [0 as Word; RB_SIZE];
        let req = <peris::DMA_R as UpDma<peris::TIM_R>>::request(&*dma_r);
        let mut ch = Channel::new(dma_r, irq);
        setup_timer(&mut tim);
        tim.enable_update_dma(true);
        let mut rb = unsafe { ReadableRingBuffer::new(ch.reborrow(), req, cnt_addr, &mut buf, wopts()) };
        rb.start();

        AsyncTimer::after(Duration::from_millis(5)).await;

        let mut out = [0 as Word; 4];
        let remaining = rb.read_exact(&mut out).await.unwrap();
        info!("read_exact remaining: {}", remaining);

        let mut huge = [0 as Word; RB_SIZE + 1];
        crate::assert!(
            rb.read_exact(&mut huge).await.is_err(),
            "read_exact should fail when not enough data"
        );

        rb.request_reset();
        wait_for(|| !rb.is_running(), 10_000).await;
    }

    info!("Exact semantics test passed");
}
