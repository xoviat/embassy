use cortex_m::asm::wfi;
use cortex_m::peripheral::SCB;
use embassy_stm32::rtc::AnyRtc;
use embassy_time::{Duration, Timer, with_timeout};

use crate::shci::{SchiSysEventReady, ShciFusGetStateErrorCode};
use crate::sub::sys::Sys;

#[derive(Clone, Copy, PartialEq)]
pub enum UpgradeStatus {
    Pending,
    Complete,
}

/// Magic number to request an upgrade
const MAGIC_PENDING: u32 = 0x4f2a9c1b;

pub trait UpgradeState {
    /// Get the fimrware upgrade status
    fn get_upgrade_status(&mut self) -> UpgradeStatus;
    /// Set the fimrware upgrade status
    fn set_upgrade_status(&mut self, upgrade_status: UpgradeStatus);
}

pub struct RtcUpgradeState<T: AnyRtc> {
    rtc: T,
    backup_register: usize,
}

impl<T: AnyRtc> RtcUpgradeState<T> {
    pub const fn new(rtc: T, backup_register: usize) -> Self {
        Self { rtc, backup_register }
    }
}

impl<T: AnyRtc> UpgradeState for RtcUpgradeState<T> {
    fn get_upgrade_status(&mut self) -> UpgradeStatus {
        if self.rtc.read_backup_register(self.backup_register).unwrap_or_default() == MAGIC_PENDING {
            UpgradeStatus::Pending
        } else {
            UpgradeStatus::Complete
        }
    }

    fn set_upgrade_status(&mut self, upgrade_status: UpgradeStatus) {
        self.rtc.write_backup_register(
            self.backup_register,
            match upgrade_status {
                UpgradeStatus::Complete => 0,
                UpgradeStatus::Pending => MAGIC_PENDING,
            },
        )
    }
}

/// Boot the device with no upgrade
pub async fn boot_noupgrade(sys: &mut Sys<'_>) {
    // let _ = sys.shci_c2_fus_startws().await;
    // SCB::sys_reset();

    let sys_evt = with_timeout(Duration::from_millis(5), async {
        sys.read_ready().await.unwrap_or(SchiSysEventReady::FusFwRunning)
    })
    .await
    .unwrap_or(SchiSysEventReady::FusFwRunning);

    if sys_evt == SchiSysEventReady::WirelessFwRunning {
        debug!("wireless fw running");

        return;
    }

    debug!("starting wireless fw...");
    // If wireless fw is not running, then start the wireless stack
    let duration = Duration::from_secs(3);
    let _ = with_timeout(duration, async {
        let _ = sys.shci_c2_fus_startws().await;
        SCB::sys_reset();
    });

    SCB::sys_reset();
}

pub async fn boot<T: UpgradeState>(state: &mut T, sys: &mut Sys<'_>) -> Result<(), ()> {
    //    let firmware_started = ready_event == SchiSysEventReady::WirelessFwRunning
    //        && sys
    //            .wireless_fw_info()
    //            .is_some_and(|info| info.version_major() + info.version_minor() > 0);

    let firmware_started = false;
    let upgrade_status = state.get_upgrade_status();

    // If wireless firmware is started, then abort the upgrade and return
    if firmware_started {
        state.set_upgrade_status(UpgradeStatus::Complete);

        return Ok(());
    }

    // If we cannot get the FUS state, then return with an error
    match sys.shci_c2_fus_getstate().await? {
        ShciFusGetStateErrorCode::FusStateErrorErrUnknown => {
            // This is the first time in the life of the product the FUS is involved.
            // After this command, it will be properly initialized
            // Request the device to reboot to install the wireless firmware

            SCB::sys_reset();
        }
        ShciFusGetStateErrorCode::FusStateErrorNoError if upgrade_status == UpgradeStatus::Complete => {
            // FUS is idle and upgrade is complete, start the wireless stack
            sys.shci_c2_fus_startws().await?;
        }
        ShciFusGetStateErrorCode::FusStateErrorNoError if upgrade_status == UpgradeStatus::Pending => {
            // FUS is idle and upgrade is pending, start the upgrade
            state.set_upgrade_status(UpgradeStatus::Complete);

            sys.shci_c2_fus_fwupgrade(0, 0).await?;
        }
        _ => {}
    }

    // Wait for the FUS to reboot us
    loop {
        wfi();
    }
}

/// Start the upgrade of firmware; must be called after boot
pub async fn start_upgrade<T: UpgradeState>(state: &mut T, sys: &mut Sys<'_>) -> Result<(), ()> {
    state.set_upgrade_status(UpgradeStatus::Pending);

    sys.shci_c2_fus_getstate().await?;
    sys.shci_c2_fus_getstate().await?;

    // wait for FUS to reboot us
    loop {
        wfi();
    }
}
