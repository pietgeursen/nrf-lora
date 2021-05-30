use defmt::{info, warn, error};
pub use nrf_softdevice_s140::*;

const APP_CONN_CFG_TAG: u8 = 1;

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
        error!{"oh noes"}
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

    let mut wanted_app_ram_base = app_ram_base;
    let ret = unsafe { sd_ble_enable(&mut wanted_app_ram_base as _) };
    info!(
        "softdevice RAM: {:?} bytes",
        wanted_app_ram_base - 0x20000000
    );
    match ret {
        NRF_SUCCESS  => {}
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
