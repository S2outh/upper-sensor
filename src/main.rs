#![no_std]
#![no_main]

mod dts_drv;
mod embassy_adapter;
mod io_threads;
mod sensor_threads;

use core::{cell::RefCell, sync::atomic::Ordering};

use cortex_m_rt::interrupt;
use portable_atomic::AtomicU64;

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::{
    Config, bind_interrupts, can::{self, CanConfigurator, RxFdBuf, TxFdBuf}, dma, dts::{self, Dts}, exti::{self, ExtiInput, TriggerEdge}, gpio::{Level, Output, Pull, Speed}, i2c, interrupt::{self, typelevel::{EXTI4, EXTI15_10}}, mode::{Async, Blocking}, peripherals::{DMA1_CH1, DMA1_CH2, DMA1_CH3, DMA1_CH4, DMA2_CH2, DMA2_CH3, FDCAN1, FDCAN2, I2C1, IWDG1, USART6}, rcc, spi::{self, Spi, mode::Master}, time::{khz, mhz}, usart::{self, Uart}, wdg::IndependentWatchdog
};
use embassy_sync::{
    blocking_mutex::{self, raw::{CriticalSectionRawMutex, ThreadModeRawMutex}},
    channel::{Channel, DynamicSender},
    mutex::Mutex, once_lock::OnceLock,
};
use embassy_time::{Duration, Instant, Ticker, Timer};
use hscmrnn030pa::driver::Baro;
use lsm6dsv32::driver::Lsm6dsv32;
use phoenix::{
    gps::{DataRateInterval, GpsDriver},
    phoenix::{OutputConfig, PhoenixService, StartupConfig, StartupMode},
};
use south_common::{
    chell::{ChellDefinition, fd_compat_chell_union}, configs::can_config::CanPeriphConfig, definitions::{internal_msgs, telemetry::upper_sensor as tm}, types::Telecommand
};
use static_cell::StaticCell;

use crate::{
    dts_drv::DtsDrv,
    embassy_adapter::{EmbassyClock, EmbassyTimer, LiftoffPin},
};

use {defmt_rtt as _, panic_probe as _};

// bind interrupts
bind_interrupts!(struct Irqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;

    FDCAN2_IT0 => can::IT0InterruptHandler<FDCAN2>;
    FDCAN2_IT1 => can::IT1InterruptHandler<FDCAN2>;

    I2C1_EV => i2c::EventInterruptHandler<I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<I2C1>;
    DMA1_STREAM1 => dma::InterruptHandler<DMA1_CH1>;
    DMA1_STREAM2 => dma::InterruptHandler<DMA1_CH2>;

    DMA2_STREAM2 => dma::InterruptHandler<DMA2_CH2>;
    DMA2_STREAM3 => dma::InterruptHandler<DMA2_CH3>;

    DTS => dts::InterruptHandler;

    EXTI4 => exti::InterruptHandler<EXTI4>;
    EXTI15_10 => exti::InterruptHandler<EXTI15_10>;

    USART6 => usart::InterruptHandler<USART6>;
    DMA1_STREAM3 => dma::InterruptHandler<DMA1_CH3>;
    DMA1_STREAM4 => dma::InterruptHandler<DMA1_CH4>;
});

/// config rcc
fn get_rcc_config() -> rcc::Config {
    let mut rcc_config = rcc::Config::default();
    rcc_config.hsi = Some(rcc::HSIPrescaler::DIV1); // 64 MHz
    rcc_config.sys = rcc::Sysclk::HSI; // cpu runns with 64 MHz
    rcc_config.pll1 = Some(rcc::Pll {
        source: rcc::PllSource::HSI,
        prediv: rcc::PllPreDiv::DIV8,   // 8 MHz
        mul: rcc::PllMul::MUL40,        // 320 MHz
        divp: None,                     // 320 MHz
        divq: Some(rcc::PllDiv::DIV10), // 32 MHz
        divr: Some(rcc::PllDiv::DIV5),  // 64 MHz
    });
    rcc_config.mux.fdcansel = rcc::mux::Fdcansel::PLL1_Q; // can runns with 32 MHz
    rcc_config.voltage_scale = rcc::VoltageScale::Scale1;
    rcc_config
}

// General setup stuff
const STARTUP_DELAY: u64 = 300;

const WATCHDOG_TIMEOUT_US: u32 = 300_000;
const WATCHDOG_PETTING_INTERVAL_US: u32 = WATCHDOG_TIMEOUT_US / 2;

static TIME_REF: AtomicU64 = AtomicU64::new(0);
static TIME_REF_UPD_SECOND: AtomicU64 = AtomicU64::new(0);

// TM container
type UpperSensorTMContainer = fd_compat_chell_union!(tm);

// static concurrency sync management types
const TM_CHANNEL_BUF_SIZE: usize = 5;
const CMD_CHANNEL_BUF_SIZE: usize = 5;
static TMC: StaticCell<Channel<ThreadModeRawMutex, UpperSensorTMContainer, TM_CHANNEL_BUF_SIZE>> =
    StaticCell::new();
static CMDC: StaticCell<Channel<ThreadModeRawMutex, Telecommand, CMD_CHANNEL_BUF_SIZE>> =
    StaticCell::new();

static EXTI_INPUT: OnceLock<blocking_mutex::Mutex<CriticalSectionRawMutex, RefCell<ExtiInput<'static, Blocking>>>> = OnceLock::new();

// static paripherals
static SPI: StaticCell<Mutex<ThreadModeRawMutex, Spi<'static, Async, Master>>> = StaticCell::new();

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
    EXTI_INPUT.try_get().unwrap().lock(|p| p.borrow_mut().clear_pending());

    let time_us = TIME_REF_UPD_SECOND.swap(0, Ordering::Acquire) * 1000 * 1000;
    if time_us == 0 { return; }
    
    TIME_REF.store(time_us - Instant::now().as_micros(), Ordering::Release);
}


/// Watchdog petting task
#[embassy_executor::task]
async fn petter(mut watchdog: IndependentWatchdog<'static, IWDG1>) {
    loop {
        watchdog.pet();
        Timer::after_micros(WATCHDOG_PETTING_INTERVAL_US.into()).await;
    }
}

// internal temperature
#[embassy_executor::task]
pub async fn dts_thread(
    tm_sender: DynamicSender<'static, UpperSensorTMContainer>,
    mut dts: DtsDrv<'static>,
) {
    const DTS_LOOP_LEN: Duration = Duration::from_millis(1000);
    let mut ticker = Ticker::every(DTS_LOOP_LEN);
    loop {
        let temp = dts.read_tenth_deg().await;
        let container = UpperSensorTMContainer::new(&tm::InternalTemperature, &temp).unwrap();
        tm_sender.send(container).await;

        ticker.next().await;
    }
}

/// program entry
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    config.rcc = get_rcc_config();
    let p = embassy_stm32::init(config);
    
    //let mut cp = cortex_m::Peripherals::take().unwrap();

    const FW_VERSION: &str = env!("FW_VERSION");
    const FW_HASH: &str = env!("FW_HASH");

    info!("Launching: FW version={} hash={}", FW_VERSION, FW_HASH);

    // unleash independent watchdog
    let mut watchdog = IndependentWatchdog::new(p.IWDG1, WATCHDOG_TIMEOUT_US);
    watchdog.unleash();

    // TM channel setup
    let tm_channel = TMC.init(Channel::new());
    let cmd_channel = CMDC.init(Channel::new());

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

    let can_interface = can_configurator.activate(
        C_TX_BUF.init(TxFdBuf::<C_TX_BUF_SIZE>::new()),
        C_RX_BUF.init(RxFdBuf::<C_RX_BUF_SIZE>::new()),
    );

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
    spi_config.mode = spi::Mode {
        polarity: spi::Polarity::IdleLow,            // CPOL=0
        phase: spi::Phase::CaptureOnFirstTransition, // CPHA=0
    }; // => SPI Mode 0

    let spi = SPI.init(Mutex::new(Spi::new(
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

    // uart/phoenix setup
    let mut config = usart::Config::default();
    config.baudrate = 57600;
    let (uart_tx, uart_rx) =
        Uart::new(p.USART6, p.PC7, p.PC6, p.DMA1_CH3, p.DMA1_CH4, Irqs, config)
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
        EmbassyTimer,
    );
    let liftoff = LiftoffPin(Output::new(p.PA0, Level::Low, Speed::Low));
    let phoenix = PhoenixService::new(driver, EmbassyClock, liftoff, startup);

    // setup TIC irq
    let tic_pin = ExtiInput::new_blocking(p.PD6, p.EXTI6, Pull::None, TriggerEdge::Rising);
    if let Err(_) = EXTI_INPUT.init(blocking_mutex::Mutex::new(RefCell::new(tic_pin))) { panic!() }
    EXTI_INPUT.try_get().unwrap().lock(|p| p.borrow_mut().enable_interrupt());

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
    spawner.spawn(petter(watchdog).unwrap());
    spawner.spawn(dts_thread(tm_channel.dyn_sender(), dts_drv).unwrap());

    Timer::after_millis(STARTUP_DELAY).await;

    // driver threads
    spawner.spawn(sensor_threads::imu_thread(
        tm_channel.dyn_sender(),
        imu1,
        yellow,
        &tm::imu1::Accel,
        &tm::imu1::Gyro,
    ).unwrap());
    spawner.spawn(sensor_threads::imu_thread(
        tm_channel.dyn_sender(),
        imu2,
        red,
        &tm::imu2::Accel,
        &tm::imu2::Gyro,
    ).unwrap());
    spawner.spawn(sensor_threads::baro_thread(
        tm_channel.dyn_sender(),
        baro,
        blue,
    ).unwrap());
    spawner.spawn(sensor_threads::phoenix_thread(
        tm_channel.dyn_sender(),
        phoenix,
    ).unwrap());

    // tmtc io threads
    spawner.spawn(io_threads::can_sender_thread(
        can_interface.writer(),
        tm_channel.dyn_receiver(),
    ).unwrap());
    spawner.spawn(io_threads::can_receiver_thread(
        can_interface.reader(),
        cmd_channel.dyn_sender(),
    ).unwrap());

    // wait until all other threads finished (never)
    core::future::pending::<()>().await;
}
