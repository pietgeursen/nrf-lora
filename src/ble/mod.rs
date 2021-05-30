use core::mem;
use defmt::{error, info, warn};
pub use nrf_softdevice_s140::*;

const APP_CONN_CFG_TAG: u8 = 1;

// 833EF2C4-0045-4828-87A7-62C558847CC8
pub const LORA_MESSAGING_UUID_BASE: [u8; 16] = [
    0xC8, 0x7C, 0x84, 0x58, 0xC5, 0x62, 0xA7, 0x87, 0x28, 0x48, 0x45, 0x00, 0xC4, 0xF2, 0x3E, 0x83,
];

pub const LORA_MESSAGING_UUID_BASE_UUID128: ble_uuid128_t = ble_uuid128_t {
    uuid128: LORA_MESSAGING_UUID_BASE,
};

fn get_app_ram_base() -> u32 {
    extern "C" {
        static mut __sdata: u32;
    }
    unsafe { &mut __sdata as *mut u32 as u32 }
}

fn cfg_set(id: u32, cfg: &ble_cfg_t) {
    let app_ram_base = get_app_ram_base();

    let ret = unsafe { sd_ble_cfg_set(id, cfg, app_ram_base) };

    if ret != NRF_SUCCESS && ret != NRF_ERROR_NO_MEM {
        error! {"oh noes"}
        panic!("sd_ble_cfg_set {:?} err {:?}", id, ret)
    }
}

#[derive(Default)]
pub struct Config {
    pub clock: Option<nrf_clock_lf_cfg_t>,
    pub conn_gap: Option<ble_gap_conn_cfg_t>,
    pub conn_gattc: Option<ble_gattc_conn_cfg_t>,
    pub conn_gatts: Option<ble_gatts_conn_cfg_t>,
    pub conn_gatt: Option<ble_gatt_conn_cfg_t>,
    #[cfg(feature = "ble-l2cap")]
    pub conn_l2cap: Option<ble_l2cap_conn_cfg_t>,
    pub common_vs_uuid: Option<ble_common_cfg_vs_uuid_t>,
    pub gap_role_count: Option<ble_gap_cfg_role_count_t>,
    pub gap_device_name: Option<ble_gap_cfg_device_name_t>,
    pub gap_ppcp_incl: Option<ble_gap_cfg_ppcp_incl_cfg_t>,
    pub gap_car_incl: Option<ble_gap_cfg_car_incl_cfg_t>,
    pub gatts_service_changed: Option<ble_gatts_cfg_service_changed_t>,
    pub gatts_attr_tab_size: Option<ble_gatts_cfg_attr_tab_size_t>,
}

pub fn configure_ble(config: &Config) {
    configure_sd();

    let app_ram_base = get_app_ram_base();

    // Set at least one GAP config so conn_cfg_tag 1 (APP_CONN_CFG_TAG) is usable.
    // If you set none, it seems the softdevice won't let you use it, requiring a conn_cfg_tag of 0 (BLE_CONN_CFG_TAG_DEFAULT) instead.
    let val = config.conn_gap.unwrap_or(ble_gap_conn_cfg_t {
        conn_count: BLE_GAP_CONN_COUNT_DEFAULT as u8,
        event_length: BLE_GAP_EVENT_LENGTH_DEFAULT as u16,
    });

    cfg_set(
        BLE_CONN_CFGS_BLE_CONN_CFG_GAP,
        &ble_cfg_t {
            conn_cfg: ble_conn_cfg_t {
                conn_cfg_tag: APP_CONN_CFG_TAG,
                params: ble_conn_cfg_t__bindgen_ty_1 { gap_conn_cfg: val },
            },
        },
    );

    if let Some(val) = config.conn_gatt {
        cfg_set(
            BLE_CONN_CFGS_BLE_CONN_CFG_GATT,
            &ble_cfg_t {
                conn_cfg: ble_conn_cfg_t {
                    conn_cfg_tag: APP_CONN_CFG_TAG,
                    params: ble_conn_cfg_t__bindgen_ty_1 { gatt_conn_cfg: val },
                },
            },
        );
    }

    if let Some(val) = config.conn_gattc {
        cfg_set(
            BLE_CONN_CFGS_BLE_CONN_CFG_GATTC,
            &ble_cfg_t {
                conn_cfg: ble_conn_cfg_t {
                    conn_cfg_tag: APP_CONN_CFG_TAG,
                    params: ble_conn_cfg_t__bindgen_ty_1 {
                        gattc_conn_cfg: val,
                    },
                },
            },
        );
    }

    if let Some(val) = config.conn_gatts {
        cfg_set(
            BLE_CONN_CFGS_BLE_CONN_CFG_GATTS,
            &ble_cfg_t {
                conn_cfg: ble_conn_cfg_t {
                    conn_cfg_tag: APP_CONN_CFG_TAG,
                    params: ble_conn_cfg_t__bindgen_ty_1 {
                        gatts_conn_cfg: val,
                    },
                },
            },
        );
    }

    #[cfg(feature = "ble-l2cap")]
    if let Some(val) = config.conn_l2cap {
        cfg_set(
            BLE_CONN_CFGS_BLE_CONN_CFG_L2CAP,
            &ble_cfg_t {
                conn_cfg: ble_conn_cfg_t {
                    conn_cfg_tag: APP_CONN_CFG_TAG,
                    params: ble_conn_cfg_t__bindgen_ty_1 {
                        l2cap_conn_cfg: val,
                    },
                },
            },
        );
    }

    if let Some(val) = config.common_vs_uuid {
        cfg_set(
            BLE_COMMON_CFGS_BLE_COMMON_CFG_VS_UUID,
            &ble_cfg_t {
                common_cfg: ble_common_cfg_t { vs_uuid_cfg: val },
            },
        );
    }

    if let Some(val) = config.gap_role_count {
        cfg_set(
            BLE_GAP_CFGS_BLE_GAP_CFG_ROLE_COUNT,
            &ble_cfg_t {
                gap_cfg: ble_gap_cfg_t {
                    role_count_cfg: val,
                },
            },
        );
    }

    if let Some(val) = config.gap_device_name {
        cfg_set(
            BLE_GAP_CFGS_BLE_GAP_CFG_DEVICE_NAME,
            &ble_cfg_t {
                gap_cfg: ble_gap_cfg_t {
                    device_name_cfg: val,
                },
            },
        );
    }

    if let Some(val) = config.gap_ppcp_incl {
        cfg_set(
            BLE_GAP_CFGS_BLE_GAP_CFG_PPCP_INCL_CONFIG,
            &ble_cfg_t {
                gap_cfg: ble_gap_cfg_t {
                    ppcp_include_cfg: val,
                },
            },
        );
    }

    if let Some(val) = config.gap_car_incl {
        cfg_set(
            BLE_GAP_CFGS_BLE_GAP_CFG_CAR_INCL_CONFIG,
            &ble_cfg_t {
                gap_cfg: ble_gap_cfg_t {
                    car_include_cfg: val,
                },
            },
        );
    }

    if let Some(val) = config.gatts_service_changed {
        cfg_set(
            BLE_GATTS_CFGS_BLE_GATTS_CFG_SERVICE_CHANGED,
            &ble_cfg_t {
                gatts_cfg: ble_gatts_cfg_t {
                    service_changed: val,
                },
            },
        );
    }

    if let Some(val) = config.gatts_attr_tab_size {
        cfg_set(
            BLE_GATTS_CFGS_BLE_GATTS_CFG_ATTR_TAB_SIZE,
            &ble_cfg_t {
                gatts_cfg: ble_gatts_cfg_t { attr_tab_size: val },
            },
        );
    }

    
//    let mut uuid_t: ble_uuid_t = unsafe { mem::zeroed() };
//    let result = unsafe{ sd_ble_uuid_vs_add(&LORA_MESSAGING_UUID_BASE_UUID128, (&mut uuid_t as *mut ble_uuid_t).cast()) };
//
//    if result != NRF_SUCCESS {
//        error!("error adding vs uuid, err: {:?}", result);
//    }

    let mut wanted_app_ram_base = app_ram_base;
    let ret = unsafe { sd_ble_enable(&mut wanted_app_ram_base as _) };
    info!(
        "softdevice RAM: {:?} bytes",
        wanted_app_ram_base - 0x20000000
    );
    match ret {
        NRF_SUCCESS => {}
        NRF_ERROR_NO_MEM => {
            if wanted_app_ram_base <= app_ram_base {
                panic!("selected configuration has too high RAM requirements.")
            } else {
                panic!(
                    "too little RAM for softdevice. Change your app's RAM start address to {:x}",
                    wanted_app_ram_base
                );
            }
        }
        err => panic!("sd_ble_enable err {:?}", err),
    }

    if wanted_app_ram_base < app_ram_base {
        warn!("You're giving more RAM to the softdevice than needed. You can change your app's RAM start address to {:?}", wanted_app_ram_base);
    }

    let mut adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET as u8;

    #[rustfmt::skip]
        let mut adv_data = [
            0x02, 0x01, BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE as u8,
            0x03, 0x03, 0x09, 0x18,
            0x0a, 0x09, b'H', b'e', b'l', b'l', b'o', b'R', b'u', b's', b't',
        ];
    #[rustfmt::skip]
        let mut scan_data = [
            0x03, 0x03, 0x09, 0x18,
        ];

    let gap_adv_data = ble_gap_adv_data_t {
        adv_data: ble_data_t {
            p_data: adv_data.as_mut_ptr(),
            len: adv_data.len() as u16,
        },
        scan_rsp_data: ble_data_t {
            p_data: scan_data.as_mut_ptr(),
            len: scan_data.len() as u16,
        },
    };

    let mut gap_adv_params: ble_gap_adv_params_t = unsafe { mem::zeroed() };

    gap_adv_params.properties.type_ = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED as u8;
    gap_adv_params.primary_phy = BLE_GAP_PHY_AUTO as u8;
    gap_adv_params.secondary_phy = BLE_GAP_PHY_AUTO as u8;
    gap_adv_params.interval = 400;
    //gap_adv_params.duration = 100;

    let result =
        unsafe { sd_ble_gap_adv_set_configure(&mut adv_handle, &gap_adv_data, &gap_adv_params) };
    if result != NRF_SUCCESS {
        defmt::error!("adv set cfg result: {:?}", result);
    }

    let result = unsafe { sd_ble_gap_adv_start(adv_handle, 1) };
    if result != NRF_SUCCESS {
        defmt::error!("adv start result: {:?}", result);
    }
}

pub fn configure_sd() {
    let clock_config = nrf_clock_lf_cfg_t {
        source: NRF_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: 20,
        rc_temp_ctiv: 0,
        accuracy: NRF_CLOCK_LF_ACCURACY_20_PPM as u8,
    };
    defmt::info!("enable sd");
    let err_code = unsafe { sd_softdevice_enable(&clock_config, Some(nrf_fault_handler)) };
    defmt::info!("soft device err code: {:?}", err_code);
    let mut is_enabled = 0;
    let err_code = unsafe { sd_softdevice_is_enabled(&mut is_enabled) };
    defmt::info!("soft device is_enabled: {:?}", is_enabled);
    defmt::info!("soft device err code: {:?}", err_code);
}

extern "C" fn nrf_fault_handler(_id: u32, _pc: u32, _info: u32) {
    defmt::error!("nrf fault handler");
}
