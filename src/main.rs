#![no_std]
#![no_main]

mod dts_drv;
mod embassy_adapter;
mod sensor_fusion;
mod sensor_tasks;
use core::{array::from_fn, cell::RefCell, sync::atomic::Ordering};

use cortex_m_rt::interrupt;
use portable_atomic::AtomicU64;

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::{
    Config, bind_interrupts,
    can::{self, CanConfigurator, RxFdBuf, TxFdBuf},
    dma,
    dts::{self, Dts},
    exti::{self, ExtiInput, TriggerEdge},
    gpio::{Level, Output, Pull, Speed},
    i2c,
    interrupt::{
        self,
        typelevel::{EXTI4, EXTI15_10},
    },
    mode::{Async, Blocking},
    peripherals::{
        DMA1_CH1, DMA1_CH2, DMA1_CH3, DMA1_CH4, DMA1_CH5, DMA1_CH6, DMA2_CH2, DMA2_CH3, FDCAN1,
        FDCAN2, I2C1, IWDG1, USART6,
    },
    rcc,
    spi::{self, Spi, mode::Master},
    time::{khz, mhz},
    usart::{self, Uart},
    wdg::IndependentWatchdog,
};
use embassy_sync::{
    blocking_mutex::{
        self,
        raw::{CriticalSectionRawMutex, ThreadModeRawMutex},
    },
    mutex::Mutex,
    once_lock::OnceLock,
    pubsub::{PubSubChannel, Publisher, Subscriber, WaitResult},
};
use embassy_time::{Delay, Duration, Ticker, Timer};
use hscmrnn030pa::driver::Baro;
use lsm6dsv32::driver::Lsm6dsv32;
use phoenix::{
    gps::{DataRateInterval, F40Message, GpsDriver},
    phoenix::{OutputConfig, PhoenixService, StartupConfig, StartupMode},
};
use rm3100::driver::RM3100;
use south_common::{
    chell::ChellDefinition,
    configs::{can_config::CanPeriphConfig, mag_config},
    definitions::{internal_msgs, telemetry::upper_sensor as tm},
    gen_obdh_types,
    obdh::EmptyFunc,
    types::{Vector3i16, Vector3i32, upper_sensor::AccelRaw},
    utils::Oversampeling,
};
use static_cell::StaticCell;

use crate::{
    dts_drv::DtsDrv,
    embassy_adapter::EmbassyClock,
    sensor_tasks::helpers::{AccelOvsWrapper, GyroOvsWrapper},
};

use {defmt_rtt as _, panic_probe as _};

// bind interrupts
bind_interrupts!(struct Irqs {
    // CAN
    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;

    FDCAN2_IT0 => can::IT0InterruptHandler<FDCAN2>;
    FDCAN2_IT1 => can::IT1InterruptHandler<FDCAN2>;

    // I2C Baro
    I2C1_EV => i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<I2C1>;
    DMA1_STREAM1 => dma::InterruptHandler<DMA1_CH1>;
    DMA1_STREAM2 => dma::InterruptHandler<DMA1_CH2>;

    // SPI IMU
    DMA2_STREAM2 => dma::InterruptHandler<DMA2_CH2>;
    DMA2_STREAM3 => dma::InterruptHandler<DMA2_CH3>;

    // Interrupts IMU + Mag
    EXTI4 => exti::InterruptHandler<EXTI4>;
    EXTI15_10 => exti::InterruptHandler<EXTI15_10>;

    // SPI Mag
    DMA1_STREAM3 => dma::InterruptHandler<DMA1_CH3>;
    DMA1_STREAM4 => dma::InterruptHandler<DMA1_CH4>;

    // Temperature
    DTS => dts::InterruptHandler;

    // UART phoenix
    USART6 => usart::InterruptHandler<USART6>;
    DMA1_STREAM5 => dma::InterruptHandler<DMA1_CH5>;
    DMA1_STREAM6 => dma::InterruptHandler<DMA1_CH6>;
});

/// config rcc
fn get_rcc_config() -> rcc::Config {
    let mut rcc_config = rcc::Config::default();
    rcc_config.hsi = Some(rcc::HSIPrescaler::DIV1); // 64 MHz
    rcc_config.pll1 = Some(rcc::Pll {
        source: rcc::PllSource::HSI,
        prediv: rcc::PllPreDiv::DIV4,  // 16 MHz
        mul: rcc::PllMul::MUL30,       // 480 MHz
        divp: Some(rcc::PllDiv::DIV2), // 240 MHz
        divq: Some(rcc::PllDiv::DIV4), // 120 MHz
        divr: Some(rcc::PllDiv::DIV8), // 60 MHz
    });
    rcc_config.sys = rcc::Sysclk::PLL1_P; // cpu runns with 240 MHz
    rcc_config.mux.fdcansel = rcc::mux::Fdcansel::PLL1_Q; // can runns with 120 MHz
    rcc_config.voltage_scale = rcc::VoltageScale::Scale2; // voltage scale for max 300 MHz Pll out

    rcc_config.ahb_pre = rcc::AHBPrescaler::DIV2; // AHB runns at 120 MHz
    rcc_config.apb1_pre = rcc::APBPrescaler::DIV4; // APB 1-4 all run with 60 MHz
    rcc_config.apb2_pre = rcc::APBPrescaler::DIV4;
    rcc_config.apb3_pre = rcc::APBPrescaler::DIV4;
    rcc_config.apb4_pre = rcc::APBPrescaler::DIV4;
    rcc_config
}

// General setup stuff
const STARTUP_DELAY: u64 = 300;

const WATCHDOG_TIMEOUT_US: u32 = 300_000;
const WATCHDOG_PETTING_INTERVAL_US: u32 = WATCHDOG_TIMEOUT_US / 2;

// Time ref upd stuff
static TIME_REF_UPD_SECOND: AtomicU64 = AtomicU64::new(0);
const GPS_TIME_SRC_UPDATE_PRIO: u8 = 0;

// TM container
gen_obdh_types!(UpperSensor, tm);

// internal messaging channels
static COM_CHANNELS: UpperSensorComChannels = UpperSensorComChannels::new(4);

// Sensor data channel
#[derive(Clone)]
enum SensorData {
    Accel(u8, AccelRaw),
    Gyro(u8, Vector3i16),
    Mag(Vector3i32),
    Baro(u16),
    Gps(F40Message),
}
const SENS_CHANNEL_PUBS: usize = 5;
const SENS_CHANNEL_SUBS: usize = 2;
const MAX_SENS_CHANNEL_LEN: usize = 5;

type SensChannel = PubSubChannel<
    ThreadModeRawMutex,
    SensorData,
    MAX_SENS_CHANNEL_LEN,
    SENS_CHANNEL_SUBS,
    SENS_CHANNEL_PUBS,
>;
type SensPub = Publisher<
    'static,
    ThreadModeRawMutex,
    SensorData,
    MAX_SENS_CHANNEL_LEN,
    SENS_CHANNEL_SUBS,
    SENS_CHANNEL_PUBS,
>;
type SensSub = Subscriber<
    'static,
    ThreadModeRawMutex,
    SensorData,
    MAX_SENS_CHANNEL_LEN,
    SENS_CHANNEL_SUBS,
    SENS_CHANNEL_PUBS,
>;
static SENS_CHANNEL: SensChannel = PubSubChannel::new();

// Interrupt pin reference
static EXTI_INPUT: OnceLock<
    blocking_mutex::Mutex<CriticalSectionRawMutex, RefCell<ExtiInput<'static, Blocking>>>,
> = OnceLock::new();

// static paripherals
static SPI_IMU: StaticCell<Mutex<ThreadModeRawMutex, Spi<'static, Async, Master>>> =
    StaticCell::new();
static SPI_MAG: StaticCell<Mutex<ThreadModeRawMutex, Spi<'static, Async, Master>>> =
    StaticCell::new();

// can configuration
const C_RX_BUF_SIZE: usize = 64;
const C_TX_BUF_SIZE: usize = 64;

static C_RX_BUF: StaticCell<RxFdBuf<C_RX_BUF_SIZE>> = StaticCell::new();
static C_TX_BUF: StaticCell<TxFdBuf<C_TX_BUF_SIZE>> = StaticCell::new();

// Static uart buffer
const S_RX_BUF_SIZE: usize = 256;
static S_RX_BUF: StaticCell<[u8; S_RX_BUF_SIZE]> = StaticCell::new();

/// IRQS for time ref update. Fires on every integer second with guaranteed < 1us accuracy
/// (0.2us on average)
#[interrupt]
fn EXTI9_5() {
    EXTI_INPUT
        .try_get()
        .unwrap()
        .lock(|p| p.borrow_mut().clear_pending());

    let time_us = TIME_REF_UPD_SECOND.swap(0, Ordering::Acquire) * 1000 * 1000;
    if time_us == 0 {
        return;
    }

    COM_CHANNELS.set_utc_us(time_us, GPS_TIME_SRC_UPDATE_PRIO);
}

/// Watchdog petting task
#[embassy_executor::task]
async fn petter(mut watchdog: IndependentWatchdog<'static, IWDG1>) {
    loop {
        watchdog.pet();
        Timer::after_micros(WATCHDOG_PETTING_INTERVAL_US.into()).await;
    }
}

#[embassy_executor::task]
pub async fn can_receiver_task(mut can_receiver: UpperSensorCanReceiver) -> ! {
    can_receiver.run().await
}

#[embassy_executor::task]
pub async fn can_sender_task(mut can_sender: UpperSensorCanSender) -> ! {
    can_sender.run().await
}

// internal temperature
#[embassy_executor::task]
pub async fn dts_task(tm_sender: UpperSensorTMSender, mut dts: DtsDrv<'static>) {
    const DTS_LOOP_LEN: Duration = Duration::from_millis(1000);
    let mut ticker = Ticker::every(DTS_LOOP_LEN);
    loop {
        let temp = dts.read_tenth_deg().await;
        let container = UpperSensorChellUnion::new(&tm::InternalTemperature, &temp).unwrap();
        tm_sender.send(container).await;

        ticker.next().await;
    }
}

// relay sensor telemetry
#[embassy_executor::task]
pub async fn sensor_tm_task(mut sensor_sub: SensSub, tm_sender: UpperSensorTMSender) -> ! {
    macro_rules! send {
        ($def: expr, $value:expr) => {{
            let container = UpperSensorChellUnion::new($def, $value).unwrap();
            tm_sender.send(container).await;
        }};
    }

    // Imu data comes in at 96 Hz
    // software oversampeling 8 values
    // => 12 Hz
    const NUM_SAMPLES: usize = 8;
    let mut accel_oversampelers: [_; 2] =
        from_fn(|_| Oversampeling::new(NUM_SAMPLES, AccelOvsWrapper([[0i64; 3]; 2])));
    let mut gyro_oversampelers: [_; 2] =
        from_fn(|_| Oversampeling::new(NUM_SAMPLES, GyroOvsWrapper([0i64; 3])));

    loop {
        let WaitResult::Message(value) = sensor_sub.next_message().await else {
            defmt::unreachable!();
        };
        match value {
            SensorData::Accel(id, value) => {
                let Some(value) = accel_oversampelers[id as usize - 1].insert(value) else {
                    continue;
                };

                let chell_def: &'static dyn ChellDefinition = match id {
                    1 => &tm::imu1::Accel,
                    2 => &tm::imu2::Accel,
                    _ => defmt::unreachable!(),
                };
                send!(chell_def, &value);
            }
            SensorData::Gyro(id, value) => {
                let Some(value) = gyro_oversampelers[id as usize - 1].insert(value) else {
                    continue;
                };

                let chell_def: &'static dyn ChellDefinition = match id {
                    1 => &tm::imu1::Gyro,
                    2 => &tm::imu2::Gyro,
                    _ => defmt::unreachable!(),
                };
                send!(chell_def, &value);
            }
            SensorData::Mag(value) => send!(&tm::Magneto, &value),
            SensorData::Baro(value) => send!(&tm::Baro, &value),
            SensorData::Gps(value) => {
                let state = value.navigation_status | (value.tracked_satellites << 2);
                send!(&tm::gps::Status, &state);

                const NAV_STATUS_LOCK: u8 = 2;
                if value.navigation_status < NAV_STATUS_LOCK {
                    continue;
                }

                let ecef = Vector3i32 {
                    x: value.x_wgs84_cm as i32,
                    y: value.y_wgs84_cm as i32,
                    z: value.z_wgs84_cm as i32,
                };

                let vel = Vector3i32 {
                    x: value.vx_wgs84_1e5_mps as i32,
                    y: value.vy_wgs84_1e5_mps as i32,
                    z: value.vz_wgs84_1e5_mps as i32,
                };

                send!(&tm::gps::Pos, &ecef);
                send!(&tm::gps::Vel, &vel);
            }
        }
    }
}

/// program entry
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    config.rcc = get_rcc_config();
    let p = embassy_stm32::init(config);

    const FW_VERSION: &str = env!("FW_VERSION");
    const FW_HASH: &str = env!("FW_HASH");

    info!("Launching: FW version={} hash={}", FW_VERSION, FW_HASH);

    // unleash independent watchdog
    let mut watchdog = IndependentWatchdog::new(p.IWDG1, WATCHDOG_TIMEOUT_US);
    watchdog.unleash();

    // -- CAN configuration
    // can 1 configuration
    let mut can_configurator =
        CanPeriphConfig::new(CanConfigurator::new(p.FDCAN1, p.PD0, p.PD1, Irqs));

    // can 2 configuration
    // let mut can_configurator =
    //     CanPeriphConfig::new(CanConfigurator::new(p.FDCAN2, p.PB12, p.PB13, Irqs));

    let _can_1_standby = Output::new(p.PE2, Level::Low, Speed::Low);
    // let _can_2_standby = Output::new(p.PE3, Level::Low, Speed::Low);

    can_configurator
        .add_receive_topic(internal_msgs::Telecommand.id())
        .unwrap()
        .add_receive_topic(internal_msgs::TimesyncRequest.id())
        .unwrap();

    let can_instance = can_configurator.activate(
        C_TX_BUF.init(TxFdBuf::<C_TX_BUF_SIZE>::new()),
        C_RX_BUF.init(RxFdBuf::<C_RX_BUF_SIZE>::new()),
    );

    // Setup can sender and receiver runners
    let can_receiver = UpperSensorCanReceiver::new(can_instance.reader(), &COM_CHANNELS, EmptyFunc);

    let can_sender = UpperSensorCanSender::new(can_instance.writer(), &COM_CHANNELS);

    // i2c/baro setup
    let mut cfg = i2c::Config::default();
    cfg.frequency = khz(400);
    cfg.sda_pullup = true;
    cfg.scl_pullup = true;
    let baro = Baro::new(p.I2C1, p.PB8, p.PB9, Irqs, p.DMA1_CH1, p.DMA1_CH2, cfg);

    // spi/imu setup
    let mut spi_config = spi::Config::default();
    spi_config.frequency = mhz(30);
    spi_config.gpio_speed = Speed::High;
    spi_config.mode = spi::MODE_0;

    let spi = SPI_IMU.init(Mutex::new(Spi::new(
        p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA2_CH3, p.DMA2_CH2, Irqs, spi_config,
    )));

    let cs1 = Output::new(p.PB0, Level::High, Speed::High);
    let int1_1 = ExtiInput::new(p.PA10, p.EXTI10, Pull::Up, Irqs);
    let int1_2 = ExtiInput::new(p.PA11, p.EXTI11, Pull::Up, Irqs);

    let cs2 = Output::new(p.PB1, Level::High, Speed::High);
    let int2_1 = ExtiInput::new(p.PA15, p.EXTI15, Pull::Up, Irqs);
    let int2_2 = ExtiInput::new(p.PE4, p.EXTI4, Pull::Up, Irqs);

    let imu1 = Lsm6dsv32::new(spi, cs1, int1_1, int1_2).await;
    let imu2 = Lsm6dsv32::new(spi, cs2, int2_1, int2_2).await;

    // spi/mag setup
    let mut spi_config = spi::Config::default();
    spi_config.frequency = khz(300);
    spi_config.gpio_speed = Speed::High;
    spi_config.mode = spi::MODE_0;
    let spi = SPI_MAG.init(Mutex::new(Spi::new(
        p.SPI2, p.PA9, p.PB15, p.PB14, p.DMA1_CH3, p.DMA1_CH4, Irqs, spi_config,
    )));

    let cs = Output::new(p.PA2, Level::High, Speed::High);
    let int = ExtiInput::new(p.PA12, p.EXTI12, Pull::Down, Irqs);

    let magneto_config = mag_config::get_mag_config();

    let magneto = RM3100::new(spi, cs, int, magneto_config).await;

    // uart/phoenix setup
    let mut config = usart::Config::default();
    config.baudrate = 57600;
    let (uart_tx, uart_rx) =
        Uart::new(p.USART6, p.PC7, p.PC6, p.DMA1_CH5, p.DMA1_CH6, Irqs, config)
            .unwrap()
            .split();

    let startup = StartupConfig {
        mode: StartupMode::Regular,
        initial_position: None,
        initial_epoch: None,
        outputs: OutputConfig {
            f00: Some(DataRateInterval::Disabled),
            f40: Some(DataRateInterval::from_hz(1, 1).unwrap()),
            f48: Some(DataRateInterval::Disabled),
        },
    };

    let driver = GpsDriver::<_, _, _, 256>::new(
        uart_rx.into_ring_buffered(S_RX_BUF.init([0; _])),
        uart_tx,
        Delay,
    );
    let liftoff = Output::new(p.PA0, Level::Low, Speed::Low);
    let phoenix = PhoenixService::new(driver, EmbassyClock, liftoff, startup);

    // setup TIC irq
    let tic_pin = ExtiInput::new_blocking(p.PD6, p.EXTI6, Pull::None, TriggerEdge::Rising);
    if let Err(_) = EXTI_INPUT.init(blocking_mutex::Mutex::new(RefCell::new(tic_pin))) {
        panic!()
    }
    EXTI_INPUT
        .try_get()
        .unwrap()
        .lock(|p| p.borrow_mut().enable_interrupt());

    // internal temperature sensor
    let mut dts_config = dts::Config::default();
    dts_config.sample_time = dts::SampleTime::ClockCycles15;
    let dts = Dts::new(p.DTS, Irqs, dts_config);
    let dts_drv = DtsDrv::new(dts, dts_config.sample_time);

    // debug leds
    let green = Output::new(p.PD12, Level::Low, Speed::Medium);
    let yellow = Output::new(p.PD13, Level::Low, Speed::High);
    let red = Output::new(p.PD14, Level::Low, Speed::High);
    let blue = Output::new(p.PD15, Level::Low, Speed::High);

    // -- Thread spawning
    spawner.spawn(petter(watchdog).unwrap());
    spawner.spawn(dts_task(COM_CHANNELS.get_tm_sender(), dts_drv).unwrap());

    Timer::after_millis(STARTUP_DELAY).await;

    // driver tasks
    spawner
        .spawn(sensor_tasks::imu_task(1, SENS_CHANNEL.publisher().unwrap(), imu1, yellow).unwrap());
    spawner.spawn(sensor_tasks::imu_task(2, SENS_CHANNEL.publisher().unwrap(), imu2, red).unwrap());
    spawner.spawn(sensor_tasks::baro_task(SENS_CHANNEL.publisher().unwrap(), baro, blue).unwrap());
    spawner
        .spawn(sensor_tasks::mag_task(SENS_CHANNEL.publisher().unwrap(), magneto, green).unwrap());
    spawner.spawn(sensor_tasks::phoenix_task(SENS_CHANNEL.publisher().unwrap(), phoenix).unwrap());

    spawner.spawn(can_sender_task(can_sender).unwrap());
    spawner.spawn(can_receiver_task(can_receiver).unwrap());

    spawner.spawn(
        sensor_tm_task(
            SENS_CHANNEL.subscriber().unwrap(),
            COM_CHANNELS.get_tm_sender(),
        )
        .unwrap(),
    );
    spawner.spawn(sensor_fusion::fusion_task(SENS_CHANNEL.subscriber().unwrap()).unwrap());

    // wait until all other tasks finished (never)
    core::future::pending::<()>().await;
}
