#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    Config, bind_interrupts, can::{self, BufferedFdCanSender, CanConfigurator, RxFdBuf, TxFdBuf, frame::FdFrame}, gpio::{Level, Output, Speed}, peripherals::{FDCAN1, IWDG1}, rcc, wdg::IndependentWatchdog
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::{Channel, DynamicSender, Receiver}};
use embassy_time::Timer;
use lsm6dsv32::driver::{FifoDisabled, Int1Disabled, Int2Disabled, Lsm6dsv32};
use static_cell::StaticCell;
use south_common::{telemetry_container, TelemetryContainer, telemetry::upper_sensor as tm, can_config::CanPeriphConfig};

use {defmt_rtt as _, panic_probe as _};

// bind interrupts
bind_interrupts!(struct Irqs {
    FDCAN1_IT0 => can::IT0InterruptHandler<FDCAN1>;
    FDCAN1_IT1 => can::IT1InterruptHandler<FDCAN1>;
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
    rcc_config
}

// TM container
type UpperSensorTMContainer = telemetry_container!(tm);

// static concurrency sync management types
const TM_CHANNEL_BUF_SIZE: usize = 5;
//const CMD_CHANNEL_BUF_SIZE: usize = 5;
static TMC: StaticCell<Channel<ThreadModeRawMutex, UpperSensorTMContainer, TM_CHANNEL_BUF_SIZE>> = StaticCell::new();
//static CMDC: StaticCell<Channel<ThreadModeRawMutex, Telecommand, CMD_CHANNEL_BUF_SIZE>> = StaticCell::new();

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
pub async fn tm_thread(mut can_sender: BufferedFdCanSender, tm_channel: Receiver<'static, ThreadModeRawMutex, UpperSensorTMContainer, TM_CHANNEL_BUF_SIZE>) {
    loop {
        let container = tm_channel.receive().await;
        match FdFrame::new_standard(container.id(), container.bytes()) {
            Ok(frame) => can_sender.write(frame).await,
            Err(e) => error!("error constructing can message: {}", e),
        }
    }
}

// imu polling task
#[embassy_executor::task]
pub async fn imu_thread(tm_sender: DynamicSender<'static, UpperSensorTMContainer>, mut imu: Lsm6dsv32<'static, FifoDisabled, Int1Disabled, Int2Disabled>) {
    const IMU_LOOP_LEN_MS: u64 = 50;
    loop {
        if let Ok((low, high)) = imu.read_accel_dual_raw().await {
            let container = UpperSensorTMContainer::new(&tm::imu1::AccelLowRange, &low).unwrap();
            tm_sender.send(container).await;

            let container = UpperSensorTMContainer::new(&tm::imu1::AccelFullRange, &high).unwrap();
            tm_sender.send(container).await;
        }

        if let Ok(data) = imu.read_gyro_raw().await {
            let container = UpperSensorTMContainer::new(&tm::imu1::Gyro, &data).unwrap();
            tm_sender.send(container).await;
        }

        if let Ok(data) = imu.read_temp_raw().await {
            let container = UpperSensorTMContainer::new(&tm::imu1::Temp, &data).unwrap();
            tm_sender.send(container).await;
        }

        Timer::after_millis(IMU_LOOP_LEN_MS).await;
    }
}

// tc receiving task
// #[embassy_executor::task]
// pub async fn tc_thread(
//     can_receiver: BufferedFdCanReceiver,
//     tc_channel: Sender<'static, ThreadModeRawMutex, Telecommand, TM_CHANNEL_BUF_SIZE>
//     ) {
//     loop {
//         match can_receiver.receive().await {
//             Ok(envelope) => {
//                 match Telecommand::parse(envelope.frame.data()) {
//                     Ok(cmd) => tc_channel.send(cmd).await,
//                     Err(e) => error!("error parsing tc {}", e),
//                 }
//             }
//             Err(e) => error!("error in frame! {}", e),
//         }
//     }
// }


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
    //let cmd_channel = CMDC.init(Channel::new());

    // set can standby pin to low
    let _can_standby = Output::new(p.PA10, Level::Low, Speed::Low);
    //let _can_2_standby = Output::new(p.PB2, Level::High, Speed::Low);

    // -- CAN configuration
    let can_configurator = CanPeriphConfig::new(CanConfigurator::new(p.FDCAN1, p.PA11, p.PA12, Irqs));

    //can_configurator
    //    .add_receive_topic(telecommands::Telecommand.id())
    //    .unwrap();

    let can_interface = can_configurator.activate(
        TX_BUF.init(TxFdBuf::<TX_BUF_SIZE>::new()),
        RX_BUF.init(RxFdBuf::<RX_BUF_SIZE>::new()),
    );

    spawner.must_spawn(petter(watchdog));

    spawner.must_spawn(tm_thread(can_interface.writer(), tm_channel.receiver()));
    // spawner.must_spawn(tc_thread(can_interface.reader(), cmd_channel.sender()));
    
    // wait until all other threads finished (never)
    core::future::pending::<()>().await;
}
