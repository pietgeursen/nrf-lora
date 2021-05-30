use nrf_softdevice_s140::*;

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
