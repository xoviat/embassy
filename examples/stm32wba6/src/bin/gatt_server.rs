#![no_std]
#![no_main]

use core::cell::RefCell;
use core::time::Duration;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::aes::Aes;
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals::{AES, PKA, RNG};
use embassy_stm32::pka::Pka;
use embassy_stm32::rcc::{
    AHB5Prescaler, AHBPrescaler, APBPrescaler, Hse, HsePrescaler, LsConfig, LseConfig, LseDrive, LseMode, PllDiv,
    PllMul, PllPreDiv, PllSource, RtcClockSource, Sysclk, VoltageScale, mux,
};
use embassy_stm32::rng::{self, Rng};
use embassy_stm32::time::Hertz;
use embassy_stm32::{Config, aes, bind_interrupts, pka};
use embassy_stm32_wpan::{Controller, HighInterruptHandler, LowInterruptHandler, new_controller_state};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use static_cell::StaticCell;
use stm32wb_hci::event::command::{CommandComplete, ReturnParameters};
use stm32wb_hci::host::uart::Packet;
use stm32wb_hci::host::{AdvertisingFilterPolicy, EncryptionKey, HostHci, OwnAddressType};
use stm32wb_hci::types::AdvertisingType;
use stm32wb_hci::vendor::command::gap::{
    AddressType, AuthenticationRequirements, DiscoverableParameters, GapCommands, IoCapability, LocalName, Pin, Role,
    SecureConnectionSupport,
};
use stm32wb_hci::vendor::command::gatt::{
    AddCharacteristicParameters, AddServiceParameters, CharacteristicEvent, CharacteristicPermission,
    CharacteristicProperty, EncryptionKeySize, GattCommands, ServiceType, UpdateCharacteristicValueParameters, Uuid,
    WriteResponseParameters,
};
use stm32wb_hci::vendor::command::hal::{ConfigData, HalCommands, PowerLevel};
use stm32wb_hci::vendor::event::command::VendorReturnParameters;
use stm32wb_hci::vendor::event::{self, AttributeHandle, VendorEvent};
use stm32wb_hci::{BdAddr, Event};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    RNG => rng::InterruptHandler<RNG>;
    RADIO => HighInterruptHandler;
    AES => aes::InterruptHandler<AES>;
    PKA => pka::InterruptHandler<PKA>;
    HASH => LowInterruptHandler;
});

const BLE_GAP_DEVICE_NAME_LENGTH: u8 = 7;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();

    // Enable HSE (32 MHz external crystal) - REQUIRED for BLE radio
    config.rcc.hse = Some(Hse {
        prescaler: HsePrescaler::Div1,
    });

    // Enable LSE (32.768 kHz external crystal) - REQUIRED for BLE radio sleep timer
    config.rcc.ls = LsConfig {
        rtc: RtcClockSource::Lse,
        lsi: false,
        lse: Some(LseConfig {
            frequency: Hertz(32_768),
            mode: LseMode::Oscillator(LseDrive::MediumLow),
            peripherals_clocked: true,
        }),
    };

    // Configure PLL1 (required on WBA)
    config.rcc.pll1 = Some(embassy_stm32::rcc::Pll {
        source: PllSource::Hsi,
        prediv: PllPreDiv::Div1,
        mul: PllMul::Mul30,
        divr: Some(PllDiv::Div5),
        divq: None,
        divp: Some(PllDiv::Div30),
        frac: Some(0),
    });

    config.rcc.ahb_pre = AHBPrescaler::Div1;
    config.rcc.apb1_pre = APBPrescaler::Div1;
    config.rcc.apb2_pre = APBPrescaler::Div1;
    config.rcc.apb7_pre = APBPrescaler::Div1;
    config.rcc.ahb5_pre = AHB5Prescaler::Div4;
    config.rcc.voltage_scale = VoltageScale::Range1;
    config.rcc.sys = Sysclk::Pll1R;
    config.rcc.mux.rngsel = mux::Rngsel::Hsi;

    let p = embassy_stm32::init(config);

    // Apply HSE trimming for accurate radio frequency (matching ST's Config_HSE)
    // and configure radio sleep timer to use LSE
    {
        use embassy_stm32::pac::RCC;
        use embassy_stm32::pac::rcc::vals::Radiostsel;
        RCC.ecscr1().modify(|w| w.set_hsetrim(0x0C));
        RCC.bdcr().modify(|w| w.set_radiostsel(Radiostsel::Lse));
    }

    info!("Embassy STM32WBA6 BLE Central Example");

    // Initialize hardware peripherals required by BLE stack
    static RNG_INST: StaticCell<Mutex<CriticalSectionRawMutex, RefCell<Rng<'static, RNG>>>> = StaticCell::new();
    let rng = RNG_INST.init(Mutex::new(RefCell::new(Rng::new(p.RNG, Irqs))));

    static AES_INST: StaticCell<Mutex<CriticalSectionRawMutex, RefCell<Aes<'static, AES, Blocking>>>> =
        StaticCell::new();
    let aes = AES_INST.init(Mutex::new(RefCell::new(Aes::new_blocking(p.AES, Irqs))));

    static PKA_INST: StaticCell<Mutex<CriticalSectionRawMutex, RefCell<Pka<'static, PKA>>>> = StaticCell::new();
    let pka = PKA_INST.init(Mutex::new(RefCell::new(Pka::new_blocking(p.PKA, Irqs))));

    info!("Hardware peripherals initialized (RNG, AES, PKA)");

    // Initialize BLE stack
    let mut ble = Controller::new(new_controller_state!(8), rng, Some(aes), Some(pka), Irqs)
        .await
        .expect("BLE initialization failed");

    info!("BLE stack initialized");

    info!("resetting BLE...");
    ble.reset().await;
    let response = ble.read_event().await;
    defmt::debug!("{}", response);

    info!("config public address...");
    ble.write_config_data(&ConfigData::public_address(get_bd_addr()).build())
        .await;
    let response = ble.read_event().await;
    defmt::debug!("{}", response);

    info!("config random address...");
    ble.write_config_data(&ConfigData::random_address(get_random_addr()).build())
        .await;
    let response = ble.read_event().await;
    defmt::debug!("{}", response);

    info!("config identity root...");
    ble.write_config_data(&ConfigData::identity_root(&get_irk()).build())
        .await;
    let response = ble.read_event().await;
    defmt::debug!("{}", response);

    info!("config encryption root...");
    ble.write_config_data(&ConfigData::encryption_root(&get_erk()).build())
        .await;
    let response = ble.read_event().await;
    defmt::debug!("{}", response);

    info!("config tx power level...");
    ble.set_tx_power_level(PowerLevel::ZerodBm).await;
    let response = ble.read_event().await;
    defmt::debug!("{}", response);

    info!("GATT init...");
    ble.init_gatt().await;
    let response = ble.read_event().await;
    defmt::debug!("{}", response);

    info!("GAP init...");
    ble.init_gap(Role::PERIPHERAL, false, BLE_GAP_DEVICE_NAME_LENGTH).await;
    let response = ble.read_event().await;
    defmt::debug!("{}", response);

    info!("set IO capabilities...");
    ble.set_io_capability(IoCapability::DisplayConfirm).await;
    let response = ble.read_event().await;
    defmt::debug!("{}", response);

    info!("set authentication requirements...");
    ble.set_authentication_requirement(&AuthenticationRequirements {
        bonding_required: false,
        keypress_notification_support: false,
        mitm_protection_required: false,
        encryption_key_size_range: (8, 16),
        fixed_pin: Pin::Requested,
        identity_address_type: AddressType::Public,
        secure_connection_support: SecureConnectionSupport::Optional,
    })
    .await
    .unwrap();
    let response = ble.read_event().await;
    defmt::debug!("{}", response);

    info!("set scan response data...");
    ble.le_set_scan_response_data(b"TXTX").await.unwrap();
    let response = ble.read_event().await;
    defmt::debug!("{}", response);

    defmt::info!("initializing services and characteristics...");
    let mut ble_context = init_gatt_services(&mut ble).await.unwrap();
    defmt::info!("{}", ble_context);

    let discovery_params = DiscoverableParameters {
        advertising_type: AdvertisingType::ConnectableUndirected,
        advertising_interval: Some((Duration::from_millis(100), Duration::from_millis(100))),
        address_type: OwnAddressType::Public,
        filter_policy: AdvertisingFilterPolicy::AllowConnectionAndScan,
        local_name: Some(LocalName::Complete(b"TXTX")),
        advertising_data: &[],
        conn_interval: (None, None),
    };

    info!("set discoverable...");
    ble.set_discoverable(&discovery_params).await.unwrap();
    let response = ble.read_event().await;
    defmt::debug!("{}", response);

    loop {
        let response = ble.read_event().await;
        defmt::debug!("{}", response);

        if let Ok(Packet::Event(event)) = response {
            match event {
                Event::LeConnectionComplete(_) => {
                    defmt::info!("connected");
                }
                Event::DisconnectionComplete(_) => {
                    defmt::info!("disconnected");
                    ble_context.is_subscribed = false;
                    ble.set_discoverable(&discovery_params).await.unwrap();
                }
                Event::Vendor(vendor_event) => match vendor_event {
                    VendorEvent::AttReadPermitRequest(read_req) => {
                        defmt::info!("read request received {}, allowing", read_req);
                        ble.allow_read(read_req.conn_handle).await
                    }
                    VendorEvent::AttWritePermitRequest(write_req) => {
                        defmt::info!("write request received {}, allowing", write_req);
                        ble.write_response(&WriteResponseParameters {
                            conn_handle: write_req.conn_handle,
                            attribute_handle: write_req.attribute_handle,
                            status: Ok(()),
                            value: write_req.value(),
                        })
                        .await
                        .unwrap()
                    }
                    VendorEvent::GattAttributeModified(attribute) => {
                        defmt::info!("{}", ble_context);
                        if attribute.attr_handle.0 == ble_context.chars.notify.0 + 2 {
                            if attribute.data()[0] == 0x01 {
                                defmt::info!("subscribed");
                                ble_context.is_subscribed = true;
                            } else {
                                defmt::info!("unsubscribed");
                                ble_context.is_subscribed = false;
                            }
                        }
                    }
                    _ => {}
                },
                _ => {}
            }
        }
    }
}

fn get_bd_addr() -> BdAddr {
    let bytes = [0u8; 6];

    // let lhci_info = LhciC1DeviceInformationCcrp::new();
    // bytes[0] = (lhci_info.uid64 & 0xff) as u8;
    // bytes[1] = ((lhci_info.uid64 >> 8) & 0xff) as u8;
    // bytes[2] = ((lhci_info.uid64 >> 16) & 0xff) as u8;
    // bytes[3] = lhci_info.device_type_id;
    // bytes[4] = (lhci_info.st_company_id & 0xff) as u8;
    // bytes[5] = (lhci_info.st_company_id >> 8 & 0xff) as u8;

    BdAddr(bytes)
}

fn get_random_addr() -> BdAddr {
    let bytes = [0u8; 6];

    // let lhci_info = LhciC1DeviceInformationCcrp::new();
    // bytes[0] = (lhci_info.uid64 & 0xff) as u8;
    // bytes[1] = ((lhci_info.uid64 >> 8) & 0xff) as u8;
    // bytes[2] = ((lhci_info.uid64 >> 16) & 0xff) as u8;
    // bytes[3] = 0;
    // bytes[4] = 0x6E;
    // bytes[5] = 0xED;

    BdAddr(bytes)
}

const BLE_CFG_IRK: [u8; 16] = [
    0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0,
];
const BLE_CFG_ERK: [u8; 16] = [
    0xfe, 0xdc, 0xba, 0x09, 0x87, 0x65, 0x43, 0x21, 0xfe, 0xdc, 0xba, 0x09, 0x87, 0x65, 0x43, 0x21,
];

fn get_irk() -> EncryptionKey {
    EncryptionKey(BLE_CFG_IRK)
}

fn get_erk() -> EncryptionKey {
    EncryptionKey(BLE_CFG_ERK)
}

#[derive(defmt::Format)]
pub struct BleContext {
    pub service_handle: AttributeHandle,
    pub chars: CharHandles,
    pub is_subscribed: bool,
}

#[derive(defmt::Format)]
pub struct CharHandles {
    pub read: AttributeHandle,
    pub write: AttributeHandle,
    pub notify: AttributeHandle,
}

pub async fn init_gatt_services<'a>(ble_subsystem: &mut Controller) -> Result<BleContext, ()> {
    let service_handle = gatt_add_service(ble_subsystem, Uuid::Uuid16(0x500)).await?;

    let read = gatt_add_char(
        ble_subsystem,
        service_handle,
        Uuid::Uuid16(0x501),
        CharacteristicProperty::READ,
        Some(b"Hello from embassy!"),
    )
    .await?;

    let write = gatt_add_char(
        ble_subsystem,
        service_handle,
        Uuid::Uuid16(0x502),
        CharacteristicProperty::WRITE_WITHOUT_RESPONSE | CharacteristicProperty::WRITE | CharacteristicProperty::READ,
        None,
    )
    .await?;

    let notify = gatt_add_char(
        ble_subsystem,
        service_handle,
        Uuid::Uuid16(0x503),
        CharacteristicProperty::NOTIFY | CharacteristicProperty::READ,
        None,
    )
    .await?;

    Ok(BleContext {
        service_handle,
        is_subscribed: false,
        chars: CharHandles { read, write, notify },
    })
}

async fn gatt_add_service<'a>(ble_subsystem: &mut Controller, uuid: Uuid) -> Result<AttributeHandle, ()> {
    ble_subsystem
        .add_service(&AddServiceParameters {
            uuid,
            service_type: ServiceType::Primary,
            max_attribute_records: 8,
        })
        .await;
    let response = ble_subsystem.read_event().await;
    defmt::debug!("{}", response);

    if let Ok(Packet::Event(Event::CommandComplete(CommandComplete {
        return_params:
            ReturnParameters::Vendor(VendorReturnParameters::GattAddService(event::command::GattService {
                service_handle,
                ..
            })),
        ..
    }))) = response
    {
        Ok(service_handle)
    } else {
        Err(())
    }
}

async fn gatt_add_char<'a>(
    ble_subsystem: &mut Controller,
    service_handle: AttributeHandle,
    characteristic_uuid: Uuid,
    characteristic_properties: CharacteristicProperty,
    default_value: Option<&[u8]>,
) -> Result<AttributeHandle, ()> {
    ble_subsystem
        .add_characteristic(&AddCharacteristicParameters {
            service_handle,
            characteristic_uuid,
            characteristic_properties,
            characteristic_value_len: 32,
            security_permissions: CharacteristicPermission::empty(),
            gatt_event_mask: CharacteristicEvent::all(),
            encryption_key_size: EncryptionKeySize::with_value(7).unwrap(),
            is_variable: true,
        })
        .await;
    let response = ble_subsystem.read_event().await;
    defmt::debug!("{}", response);

    if let Ok(Packet::Event(Event::CommandComplete(CommandComplete {
        return_params:
            ReturnParameters::Vendor(VendorReturnParameters::GattAddCharacteristic(event::command::GattCharacteristic {
                characteristic_handle,
                ..
            })),
        ..
    }))) = response
    {
        if let Some(value) = default_value {
            ble_subsystem
                .update_characteristic_value(&UpdateCharacteristicValueParameters {
                    service_handle,
                    characteristic_handle,
                    offset: 0,
                    value,
                })
                .await
                .unwrap();

            let response = ble_subsystem.read_event().await;
            defmt::debug!("{}", response);
        }
        Ok(characteristic_handle)
    } else {
        Err(())
    }
}
