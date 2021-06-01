use bbqueue::{Consumer, Producer, consts::*};
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
        irq_masks::{IrqMask, IrqMasks},
        modem_config1::ModemConfig1,
        modem_config3::{LowDataRateOptimize, ModemConfig3},
        op_mode::Mode,
        pa_config::{PaConfig, PaSelect},
    },
    size_bytes::SizeBytes,
    LoRaMode, RFM95,
};
use smlang::statemachine;

pub const TX_FREQUENCY_HZ: u32 = 915_000_000;
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
    pub tx_consumer: Consumer<'static, U512>,
    pub rx_producer: Producer<'static, U512>,
}

pub struct TempContext<'a> {
    pub spim: &'a mut Spim<SPIM0>,
}

statemachine! {
    *(&mut TempContext) Uninit + Initialize / initialize = Idle,
    Idle + StartTransmitting [guard_start_transmitting] / start_transmitting = Transmitting,
    Transmitting + TxCompleted / stop_transmitting = Idle,
    Idle + StartReceiving / start_receiving = Receiving,
    Idle + StartCad / start_cad = Cad,
    Cad + CadCompleted / stop_cad = CadComplete,
    CadComplete + StartReceiving / start_receiving = Receiving, 

    Receiving + RxCompleted [guard_stop_receiving] / stop_receiving = Idle,
}

impl StateMachineContext for Context {
    fn initialize(&mut self, ctx: &mut TempContext) -> () {
        defmt::trace!("start init");
        self.rfm95.init(ctx.spim).unwrap();
        defmt::trace!("init done");

        self.rfm95
            .set_frequency(ctx.spim, Frequency::new::<hertz>(TX_FREQUENCY_HZ))
            .unwrap();
        defmt::trace!("set rfm95 freq");
        self.rfm95
            .read_update_write_packed_struct::<_, _, { IrqMasks::SIZE }>(
                ctx.spim,
                |masks: &mut IrqMasks| {
                    masks.tx_done = IrqMask::Enabled;
                    masks.rx_done = IrqMask::Enabled;
                },
            )
            .unwrap();
        self.rfm95
            .read_update_write_packed_struct::<_, _, { PaConfig::SIZE }>(
                ctx.spim,
                |config: &mut PaConfig| {
                    config.pa_select = PaSelect::PaBoost;
                },
            )
            .unwrap();
        self.rfm95
            .read_update_write_packed_struct::<_, _, { ModemConfig3::SIZE }>(
                ctx.spim,
                |config: &mut ModemConfig3| {
                    //config.agc = AGC::On;
                    config.low_data_rate_optimize = LowDataRateOptimize::On;
                },
            )
            .unwrap();

        // TODO put this in config?
        self.rfm95
            .set_rx_fifo_base_address(ctx.spim, 0).unwrap();
    }

    fn guard_start_transmitting(&mut self, _ctx: &mut TempContext) -> bool {
        defmt::trace!("guard start transmitting");
        self.tx_consumer.read().is_ok()
    }
    fn start_transmitting(&mut self, ctx: &mut TempContext) -> () {
        defmt::trace!("start transmitting");
        // clear the tx interrupt in the rfm95
        let mut data_to_transmit = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10];

        // TODO set the fifo tx pointer to the start of the address space

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

    fn start_cad(&mut self, ctx: &mut TempContext) -> (){

    }

    fn stop_cad(&mut self, ctx: &mut TempContext) -> (){

    }
    fn start_receiving(&mut self, ctx: &mut TempContext) -> () {
        defmt::trace!("rx_data");
        // TODO: set the rx pointer
        
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

        // get the number or rx bytes
        let fifo_address_and_size = self.rfm95.get_rx_fifo_address_and_size(&mut ctx.spim).unwrap();
        defmt::trace!("rx_fifo_size: {:?}", fifo_address_and_size.size);
        defmt::trace!("rx_fifo_address: {:?}", fifo_address_and_size.address);

        // reserve the bytes in the queue
        let mut grant = self.rx_producer.grant_exact(fifo_address_and_size.size as usize).expect("out of space in queue");
        // Pass the reserved bytes in and read the rx packet.
        self.rfm95.read_rx_fifo(&mut ctx.spim, &fifo_address_and_size, grant.buf()).unwrap();
        // Commit the bytes in the queue
        grant.commit(fifo_address_and_size.size as usize);

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

        irq_flags.rx_done && !irq_flags.payload_crc_error && irq_flags.valid_header
    }
}
