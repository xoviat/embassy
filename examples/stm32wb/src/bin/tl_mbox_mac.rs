#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::poll_once;
use embassy_stm32::ipcc::{Config, ReceiveInterruptHandler, TransmitInterruptHandler};
use embassy_stm32::{bind_interrupts, pac};
use embassy_stm32_wpan::sub::mm;
use embassy_stm32_wpan::TlMbox;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs{
    IPCC_C1_RX => ReceiveInterruptHandler;
    IPCC_C1_TX => TransmitInterruptHandler;
});

#[embassy_executor::task]
async fn run_mm_queue(memory_manager: mm::MemoryManager) {
    memory_manager.run_queue().await;
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    /*
        How to make this work:

        - Obtain a NUCLEO-STM32WB55 from your preferred supplier.
        - Download and Install STM32CubeProgrammer.
        - Download stm32wb5x_FUS_fw.bin, stm32wb5x_BLE_Stack_full_fw.bin, and Release_Notes.html from
          gh:STMicroelectronics/STM32CubeWB@2234d97/Projects/STM32WB_Copro_Wireless_Binaries/STM32WB5x
        - Open STM32CubeProgrammer
        - On the right-hand pane, click "firmware upgrade" to upgrade the st-link firmware.
        - Once complete, click connect to connect to the device.
        - On the left hand pane, click the RSS signal icon to open "Firmware Upgrade Services".
        - In the Release_Notes.html, find the memory address that corresponds to your device for the stm32wb5x_FUS_fw.bin file
        - Select that file, the memory address, "verify download", and then "Firmware Upgrade".
        - Once complete, in the Release_Notes.html, find the memory address that corresponds to your device for the
          stm32wb5x_BLE_Stack_full_fw.bin file. It should not be the same memory address.
        - Select that file, the memory address, "verify download", and then "Firmware Upgrade".
        - Select "Start Wireless Stack".
        - Disconnect from the device.
        - In the examples folder for stm32wb, modify the memory.x file to match your target device.
        - Run this example.

        Note: extended stack versions are not supported at this time. Do not attempt to install a stack with "extended" in the name.
    */

    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    //    pac::FLASH.sr().modify(|w| w.set_operr(false));
    //
    //    pac::PWR.cr1().modify(|w| w.set_dbp(true));
    //    pac::PWR.cr1().modify(|w| w.set_dbp(true));
    //
    //    pac::RCC.bdcr().modify(|w| w.set_lseon(true));
    //    while !pac::RCC.bdcr().read().lserdy() {}
    //
    //    pac::RCC.csr().modify(|w| w.set_rfwkpsel(0b01));
    //    pac::RCC.cfgr().modify(|w| w.set_stopwuck(true));

    let config = Config::default();
    let mbox = TlMbox::init(p.IPCC, Irqs, config);

    // spawner.spawn(run_mm_queue(mbox.mm_subsystem)).unwrap();

    let sys_event = mbox.sys_subsystem.read().await;
    info!("sys event: {}", sys_event.payload());

    let _ = poll_once(mbox.sys_subsystem.read());

    // core::mem::drop(sys_event);

    use embassy_stm32_wpan::channels::cpu2::{IPCC_MAC_802_15_4_NOTIFICATION_ACK_CHANNEL, IPCC_TRACES_CHANNEL};
    use embassy_stm32_wpan::consts::POOL_SIZE;
    use embassy_stm32_wpan::tables::{
        Mac802_15_4Table, TracesTable, EVT_POOL, MAC_802_15_4_CMD_BUFFER, MAC_802_15_4_NOTIF_RSP_EVT_BUFFER,
        SYS_SPARE_EVT_BUF, TL_MAC_802_15_4_TABLE, TL_TRACES_TABLE, TRACES_EVT_QUEUE,
    };
    use embassy_stm32_wpan::unsafe_linked_list::LinkedListNode;

    unsafe {
        LinkedListNode::init_head(TRACES_EVT_QUEUE.as_mut_ptr() as *mut _);

        TL_TRACES_TABLE.as_mut_ptr().write_volatile(TracesTable {
            traces_queue: TRACES_EVT_QUEUE.as_ptr() as *const _,
        });

        TL_MAC_802_15_4_TABLE.as_mut_ptr().write_volatile(Mac802_15_4Table {
            p_cmdrsp_buffer: MAC_802_15_4_CMD_BUFFER.as_mut_ptr().cast(),
            p_notack_buffer: MAC_802_15_4_NOTIF_RSP_EVT_BUFFER.as_mut_ptr().cast(),
            evt_queue: core::ptr::null_mut(),
        });
    };

    //    pac::IPCC
    //        .cpu(0)
    //        .mr()
    //        .modify(|w| w.set_chom(IPCC_MAC_802_15_4_NOTIFICATION_ACK_CHANNEL as usize, false));
    //
    //    pac::IPCC
    //        .cpu(0)
    //        .mr()
    //        .modify(|w| w.set_chom(IPCC_TRACES_CHANNEL as usize, false));

    //    unsafe {
    //        use core::ptr;
    //

    info!("mac 802 15 4 cmd buffer: {:x}", unsafe {
        MAC_802_15_4_CMD_BUFFER.as_ptr()
    });
    info!("mac 802 15 4 notif rsp evt buffer: {:x}", unsafe {
        MAC_802_15_4_NOTIF_RSP_EVT_BUFFER.as_ptr()
    });
    info!("evt pool: {} {:x}", POOL_SIZE, unsafe { EVT_POOL.as_ptr() });

    //        use embassy_stm32_wpan::unsafe_linked_list::LinkedListNode;
    //
    //        LinkedListNode::init_head(TRACES_EVT_QUEUE.as_mut_ptr());
    //
    //        TL_MAC_802_15_4_TABLE.as_mut_ptr().write_volatile(Mac802_15_4Table {
    //            p_cmdrsp_buffer: MAC_802_15_4_CMD_BUFFER.as_mut_ptr().cast(),
    //            p_notack_buffer: MAC_802_15_4_NOTIF_RSP_EVT_BUFFER.as_mut_ptr().cast(),
    //            evt_queue: ptr::null_mut(),
    //        });
    //    }

    let result = mbox.sys_subsystem.shci_c2_mac_802_15_4_init().await;
    info!("initialized mac: {}", result);

    //
    //    info!("starting ble...");
    //    mbox.ble_subsystem.t_write(0x0c, &[]).await;
    //
    //    info!("waiting for ble...");
    //    let ble_event = mbox.ble_subsystem.tl_read().await;
    //
    //    info!("ble event: {}", ble_event.payload());

    info!("Test OK");
    cortex_m::asm::bkpt();
}
