#![no_std]
#![no_main]

mod dts_drv;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    Config, bind_interrupts, can::{
        self, BufferedFdCanReceiver, BufferedFdCanSender, CanConfigurator, RxFdBuf, TxFdBuf,
        frame::FdFrame,
    }, dts::{self, Dts}, exti::ExtiInput, gpio::{Level, Output, Pull, Speed}, i2c, mode::Async, peripherals::{DMA1_CH1, DMA1_CH2, FDCAN1, I2C1, IWDG1, PB8, PB9}, rcc, spi::{self, Spi}, time::Hertz, wdg::IndependentWatchdog
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    channel::{Channel, DynamicSender, Receiver, Sender},
    mutex::Mutex,
};
use embassy_time::Timer;
use hscmrnn030pa::driver::Baro;
use lsm6dsv32::driver::{FifoDisabled, Int1Disabled, Int2Disabled, Lsm6dsv32};
use south_common::{
    TMValue, TelemetryContainer, TelemetryDefinition, can_config::CanPeriphConfig, telecommands,
    telemetry::upper_sensor as tm, telemetry_container, types::Telecommand,
};
use static_cell::StaticCell;

use crate::dts_drv::DtsDrv;

use {defmt_rtt as _, panic_probe as _};

// bind interrupts
bind_interrupts!(struct Irqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;

    I2C1_EV => i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<I2C1>;
    
    DTS => dts::InterruptHandler;
});

/// config rcc
fn get_rcc_config() -> rcc::Config {
    let mut rcc_config = rcc::Config::default();
    rcc_config.hsi = Some(rcc::HSIPrescaler::DIV1);
    rcc_config.sys = rcc::Sysclk::HSI;
    rcc_config.pll1 = Some(rcc::Pll {
        source: rcc::PllSource::HSI,
        prediv: rcc::PllPreDiv::DIV8,
        mul: rcc::PllMul::MUL40,
        divp: None,
        divq: Some(rcc::PllDiv::DIV10),
        divr: Some(rcc::PllDiv::DIV5),
    });
    rcc_config.mux.fdcansel = rcc::mux::Fdcansel::PLL1_Q;
    rcc_config.voltage_scale = rcc::VoltageScale::Scale1;
    rcc_config
}

// general setup stuff
const STARTUP_DELAY: u64 = 300;

// TM container
type UpperSensorTMContainer = telemetry_container!(tm);

// static concurrency sync management types
const TM_CHANNEL_BUF_SIZE: usize = 5;
const CMD_CHANNEL_BUF_SIZE: usize = 5;
static TMC: StaticCell<Channel<ThreadModeRawMutex, UpperSensorTMContainer, TM_CHANNEL_BUF_SIZE>> =
    StaticCell::new();
static CMDC: StaticCell<Channel<ThreadModeRawMutex, Telecommand, CMD_CHANNEL_BUF_SIZE>> =
    StaticCell::new();

// static paripherals
static SPI: StaticCell<Mutex<ThreadModeRawMutex, Spi<'static, Async>>> = StaticCell::new();

// can configuration
const RX_BUF_SIZE: usize = 64;
const TX_BUF_SIZE: usize = 64;

static RX_BUF: StaticCell<RxFdBuf<RX_BUF_SIZE>> = StaticCell::new();
static TX_BUF: StaticCell<TxFdBuf<TX_BUF_SIZE>> = StaticCell::new();

/// Watchdog petting task
#[embassy_executor::task]
async fn petter(mut watchdog: IndependentWatchdog<'static, IWDG1>) {
    loop {
        watchdog.pet();
        Timer::after_millis(200).await;
    }
}

// tm sending task
#[embassy_executor::task]
pub async fn tm_thread(
    mut can_sender: BufferedFdCanSender,
    tm_channel: Receiver<'static, ThreadModeRawMutex, UpperSensorTMContainer, TM_CHANNEL_BUF_SIZE>,
) {
    loop {
        let container = tm_channel.receive().await;
        match FdFrame::new_standard(container.id(), container.bytes()) {
            Ok(frame) => {
                can_sender.write(frame).await;
            }
            Err(e) => error!("error constructing can message: {}", e),
        }
    }
}

// baro
#[embassy_executor::task]
pub async fn baro_thread(
    tm_sender: DynamicSender<'static, UpperSensorTMContainer>,
    mut baro: Baro<'static, I2C1, PB8, PB9, Irqs, DMA1_CH1, DMA1_CH2>,
    mut led: Output<'static>,
) {
    const BARO_LOOP_LEN_MS: u64 = 100;
    loop {
        match baro.read_out().await {
            Ok(raw) => {
                // let temp = raw.baro_temp_convert();
                // let pressure = raw.baro_pressure_convert_pa();

                let container =
                    UpperSensorTMContainer::new(&tm::baro::Pressure, &raw.pressure_data).unwrap();
                tm_sender.send(container).await;

                let container =
                    UpperSensorTMContainer::new(&tm::baro::Temp, &raw.temperature_data).unwrap();
                tm_sender.send(container).await;
            }
            Err(e) => {
                error!("error reading baro: {}", e);
            }
        }

        led.toggle();
        Timer::after_millis(BARO_LOOP_LEN_MS).await;
    }
}

// imu polling task
#[embassy_executor::task(pool_size = 2)]
pub async fn imu_thread(
    tm_sender: DynamicSender<'static, UpperSensorTMContainer>,
    mut imu: Lsm6dsv32<'static, FifoDisabled, Int1Disabled, Int2Disabled>,
    mut led: Output<'static>,
    accel_low_range_def: &'static dyn TelemetryDefinition,
    accel_full_range_def: &'static dyn TelemetryDefinition,
    gyro_def: &'static dyn TelemetryDefinition,
    temp_def: &'static dyn TelemetryDefinition,
) {
    const IMU_LOOP_LEN_MS: u64 = 50;

    // config
    imu.config.accel.dual_channel = true;
    imu.config.accel.full_scale = lsm6dsv32::driver::AccelFS::G8;

    unwrap!(imu.config.accel.set_odr(lsm6dsv32::driver::AccelODR::KHz1_92));
    unwrap!(imu.config.gyro.set_odr(lsm6dsv32::driver::GyroODR::KHz1_92));

    imu.commit_config().await;

    loop {
        match imu.read_accel_dual_raw().await {
            Ok((low, full)) => {
                let container = UpperSensorTMContainer::new(accel_low_range_def, &low).unwrap();
                tm_sender.send(container).await;

                let container = UpperSensorTMContainer::new(accel_full_range_def, &full).unwrap();
                tm_sender.send(container).await;
            },
            Err(e) => error!("could not read accel: {}", e),
        }

        match imu.read_gyro_raw().await {
            Ok(data) => {
                let container = UpperSensorTMContainer::new(gyro_def, &data).unwrap();
                tm_sender.send(container).await;
            },
            Err(e) => error!("could not read gyro: {}", e),
        }

        match imu.read_temp_raw().await {
            Ok(data) => {
                let container = UpperSensorTMContainer::new(temp_def, &data).unwrap();
                tm_sender.send(container).await;
            },
            Err(e) => error!("could not read temp: {}", e),
        }

        led.toggle();
        Timer::after_millis(IMU_LOOP_LEN_MS).await;
    }
}

// temperature
#[embassy_executor::task]
pub async fn dts_thread(
    tm_sender: DynamicSender<'static, UpperSensorTMContainer>,
    mut dts: DtsDrv<'static>
) {
    const DTS_LOOP_LEN_MS: u64 = 1000;
    loop {
        let temp = dts.read_tenth_deg().await;
        let container = UpperSensorTMContainer::new(&tm::InternalTemperature, &temp).unwrap();
        tm_sender.send(container).await;

        Timer::after_millis(DTS_LOOP_LEN_MS).await;
    }
}

// tc receiving task
#[embassy_executor::task]
pub async fn tc_thread(
    can_receiver: BufferedFdCanReceiver,
    tc_channel: Sender<'static, ThreadModeRawMutex, Telecommand, TM_CHANNEL_BUF_SIZE>,
) {
    loop {
        match can_receiver.receive().await {
            Ok(envelope) => match Telecommand::from_bytes(envelope.frame.data()) {
                Ok(cmd) => tc_channel.send(cmd).await,
                Err(_) => error!("error parsing tc"),
            },
            Err(e) => error!("error in frame! {}", e),
        }
    }
}

/// program entry
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    config.rcc = get_rcc_config();
    let p = embassy_stm32::init(config);
    info!("Launching");

    // independent watchdog with timeout 300 MS
    let mut watchdog = IndependentWatchdog::new(p.IWDG1, 300_000);
    watchdog.unleash();

    // TM channel setup
    let tm_channel = TMC.init(Channel::new());
    let cmd_channel = CMDC.init(Channel::new());

    // set can standby pin to low
    let _can_standby = Output::new(p.PE2, Level::Low, Speed::Low);
    // let _can_2_standby = Output::new(p.PE3, Level::High, Speed::Low);

    // -- CAN configuration
    let mut can_configurator =
        CanPeriphConfig::new(CanConfigurator::new(p.FDCAN1, p.PD0, p.PD1, Irqs));

    can_configurator
        .add_receive_topic(telecommands::Telecommand.id())
        .unwrap();

    let can_interface = can_configurator.activate(
        TX_BUF.init(TxFdBuf::<TX_BUF_SIZE>::new()),
        RX_BUF.init(RxFdBuf::<RX_BUF_SIZE>::new()),
    );

    // i2c/baro setup
    let mut cfg = i2c::Config::default();
    cfg.frequency = Hertz(200_000);
    cfg.sda_pullup = true;
    cfg.scl_pullup = true;
    let baro = Baro::new(p.I2C1, p.PB8, p.PB9, Irqs, p.DMA1_CH1, p.DMA1_CH2, cfg);

    // spi/imu setup
    let mut spi_config = spi::Config::default();
    spi_config.frequency = Hertz(3_000_000);
    spi_config.gpio_speed = Speed::High;
    spi_config.mode = spi::Mode {
        polarity: spi::Polarity::IdleLow,            // CPOL=0
        phase: spi::Phase::CaptureOnFirstTransition, // CPHA=0
    }; // => SPI Mode 0

    let spi = SPI.init(Mutex::new(Spi::new(
        p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA2_CH3, p.DMA2_CH2, spi_config,
    )));

    let cs1 = Output::new(p.PB0, Level::High, Speed::High);
    let int1_1 = ExtiInput::new(p.PA9, p.EXTI9, Pull::Down);
    let int1_2 = ExtiInput::new(p.PA8, p.EXTI8, Pull::Down);

    let cs2 = Output::new(p.PB1, Level::High, Speed::High);
    let int2_1 = ExtiInput::new(p.PA10, p.EXTI10, Pull::Down);
    let int2_2 = ExtiInput::new(p.PA11, p.EXTI11, Pull::Down);

    let imu1 = Lsm6dsv32::new(spi, cs1, int1_1, int1_2).await;
    let imu2 = Lsm6dsv32::new(spi, cs2, int2_1, int2_2).await;

    // internal temperature sensor
    let mut dts_config = dts::Config::default();
    dts_config.sample_time = dts::SampleTime::ClockCycles15;
    let dts = Dts::new(p.DTS, Irqs, dts_config);
    let dts_drv = DtsDrv::new(dts, dts_config.sample_time);

    // debug leds
    // let mut green = Output::new(p.PD12, Level::Low, Speed::Medium);
    let yellow = Output::new(p.PD13, Level::Low, Speed::High);
    let red = Output::new(p.PD14, Level::Low, Speed::High);
    let blue = Output::new(p.PD15, Level::Low, Speed::High);

    // -- Thread spawning
    spawner.must_spawn(petter(watchdog));

    Timer::after_millis(STARTUP_DELAY).await;

    // driver threads
    spawner.must_spawn(imu_thread(
        tm_channel.dyn_sender(),
        imu1,
        yellow,
        &tm::imu1::AccelLowRange,
        &tm::imu1::AccelFullRange,
        &tm::imu1::Gyro,
        &tm::imu1::Temp,
    ));
    spawner.must_spawn(imu_thread(
        tm_channel.dyn_sender(),
        imu2,
        red,
        &tm::imu2::AccelLowRange,
        &tm::imu2::AccelFullRange,
        &tm::imu2::Gyro,
        &tm::imu2::Temp,
    ));
    spawner.must_spawn(baro_thread(tm_channel.dyn_sender(), baro, blue));
    spawner.must_spawn(dts_thread(tm_channel.dyn_sender(), dts_drv));

    // tmtc io threads
    spawner.must_spawn(tm_thread(can_interface.writer(), tm_channel.receiver()));
    spawner.must_spawn(tc_thread(can_interface.reader(), cmd_channel.sender()));

    // wait until all other threads finished (never)
    core::future::pending::<()>().await;
}
