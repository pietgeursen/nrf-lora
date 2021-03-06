use nrf52840_hal::gpio::{Output, Pin, PushPull};
use nrf52840_hal::pac::{SPIM0, TIMER3};
use nrf52840_hal::timer::OneShot;
use nrf52840_hal::Spim;
use nrf52840_hal::Timer;
use rfm95_rs::{
    lora::{
        dio_mapping::*,
        fei::*,
        frequency_rf::*,
        irq_flags::IrqFlags,
        modem_config1::{ModemConfig1},
        op_mode::Mode,
    },
    size_bytes::SizeBytes,
    LoRaMode, RFM95,
};
use smlang::statemachine;
use crate::TX_FREQUENCY_HZ;

pub struct Context {
    pub rfm95: RFM95<
        Spim<SPIM0>,
        LoRaMode,
        Pin<Output<PushPull>>,
        Pin<Output<PushPull>>,
        Timer<TIMER3, OneShot>,
    >,
    pub ppm_correction: Option<PpmCorrection>,
    pub rf_correction: Option<FrequencyRf>,
}

pub struct TempContext<'a> {
    pub spim: &'a mut Spim<SPIM0>,
}

statemachine! {
    *(&mut TempContext) Idle + StartTransmitting / start_transmission = Transmitting,
    Transmitting + TxCompleted / stop_transmitting = TxCompleted,
    TxCompleted + StartReceiving / start_receiving = Receiving,
    Receiving + RxCompleted [guard_stop_receiving] / stop_receiving = Idle,
}

impl StateMachineContext for Context {
    fn start_transmission(&mut self, ctx: &mut TempContext) -> () {
        defmt::trace!("tx_data");
        // clear the tx interrupt in the rfm95
        let mut data_to_transmit = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10];

        // Switch the dio0 mapping to enable the TxDone interrupt
        self.rfm95
            .read_update_write_packed_struct::<_, _, { DioMapping::SIZE }>(
                &mut ctx.spim,
                |mapping: &mut DioMapping| {
                    mapping.dio_0_mapping = Dio0Mapping::TxDone;
                },
            )
            .unwrap();

        // Set the tx freq back to 915.
        self.rfm95
            .set_frequency(&mut ctx.spim, Frequency::new::<hertz>(TX_FREQUENCY_HZ))
            .unwrap();

        // Sendit
        self.rfm95
            .transmit_data(&mut ctx.spim, &mut data_to_transmit)
            .unwrap();
    }

    fn stop_transmitting(&mut self, ctx: &mut TempContext) -> () {
        defmt::trace!("tx_complete");
        self.rfm95
            .read_update_write_packed_struct::<_, _, { IrqFlags::SIZE }>(
                &mut ctx.spim,
                |masks: &mut IrqFlags| {
                    masks.tx_done = true;
                },
            )
            .unwrap();
    }

    fn start_receiving(&mut self, ctx: &mut TempContext) -> () {
        defmt::trace!("rx_data");
        // clear the tx interrupt in the rfm95
        self.rfm95
            .read_update_write_packed_struct::<_, _, { DioMapping::SIZE }>(
                &mut ctx.spim,
                |mapping: &mut DioMapping| {
                    mapping.dio_0_mapping = Dio0Mapping::RxDone;
                },
            )
            .unwrap();

        // Set the new frequency
        if let Some(rf_correction) = self.rf_correction.as_ref() {
            defmt::trace!("updating rf_correction");
            //let default_freq = Frequency::new::<hertz>(TX_FREQUENCY_HZ);
            self.rfm95
                .write_packed_struct::<FrequencyRf>(&mut ctx.spim, rf_correction)
                .unwrap();
        }

        if let Some(ppm_correction) = self.ppm_correction.as_ref() {
            defmt::trace!("updating ppm_correction");
            self.rfm95
                .write_packed_struct::<_>(&mut ctx.spim, ppm_correction)
                .unwrap();
        }

        self.rfm95.receive_data(&mut ctx.spim).unwrap();
    }

    fn stop_receiving(&mut self, ctx: &mut TempContext) -> () {
        defmt::trace!("rx_complete");
        // clear the rx interrupt in the rfm95

        self.rfm95
            .read_update_write_packed_struct::<_, _, { IrqFlags::SIZE }>(
                &mut ctx.spim,
                |masks: &mut IrqFlags| {
                    masks.rx_done = true;
                    masks.payload_crc_error = true;
                    masks.rx_timeout = true;
                },
            )
            .unwrap();

        let frq_err = self
            .rfm95
            .read_packed_struct::<Fei, { Fei::SIZE }>(&mut ctx.spim)
            .unwrap();

        let bandwidth = self
            .rfm95
            .read_packed_struct::<ModemConfig1, { ModemConfig1::SIZE }>(&mut ctx.spim)
            .unwrap()
            .bandwidth;

        let current_freq = self
            .rfm95
            .read_packed_struct::<FrequencyRf, { FrequencyRf::SIZE }>(&mut ctx.spim)
            .unwrap();

        let rf_correction = frq_err.frf_new(&current_freq, &bandwidth);
        let ppm_correction = frq_err.ppm_correction(&current_freq, &bandwidth);
        self.rf_correction = Some(rf_correction);
        self.ppm_correction = Some(ppm_correction);

        defmt::info!("valid rx complete");

        self.rfm95.set_mode(&mut ctx.spim, Mode::Standby).unwrap();
    }

    fn guard_stop_receiving(&mut self, ctx: &mut TempContext) -> bool {
        let irq_flags = self
            .rfm95
            .read_packed_struct::<IrqFlags, { IrqFlags::SIZE }>(&mut ctx.spim)
            .unwrap();

        irq_flags.rx_done && !irq_flags.payload_crc_error
    }
}


