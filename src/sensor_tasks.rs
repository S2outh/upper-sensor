pub mod helpers;

use defmt::{Debug2Format, error, info, warn};
use embassy_stm32::{
    gpio::Output,
    mode::Async,
    peripherals::{DMA1_CH1, DMA1_CH2, I2C1, PB8, PB9},
    usart::{RingBufferedUartRx, UartTx},
};
use embassy_time::{Delay, Duration, Ticker};

use hscmrnn030pa::driver::Baro;
use lsm6dsv32::driver::{FifoDisabled, Int1Disabled, Int2Disabled, LogicOp, Lsm6dsv32};

use phoenix::phoenix::{PhoenixEvent, PhoenixService};
use rm3100::driver::RM3100;
use south_common::utils::Oversampeling;

use crate::{Irqs, SensPub, SensorData, embassy_adapter::EmbassyClock};

use helpers::*;

// baro polling task
#[embassy_executor::task]
pub async fn baro_task(
    sender: SensPub,
    mut baro: Baro<'static, I2C1, PB8, PB9, Irqs, DMA1_CH1, DMA1_CH2>,
    mut led: Output<'static>,
) {
    // reading baro every 300 Millis (3.333Hz)
    const BARO_LOOP_LEN: Duration = Duration::from_millis(300);
    let mut ticker = Ticker::every(BARO_LOOP_LEN);
    loop {
        match baro.read_out().await {
            Ok(raw) => sender.publish(SensorData::Baro(raw.pressure_data)).await,
            Err(e) => error!("error reading baro: {}", e),
        }

        led.toggle();
        ticker.next().await;
    }
}

// mag polling task
#[embassy_executor::task]
pub async fn mag_task(sender: SensPub, mut mag: RM3100<'static>, mut led: Output<'static>) {
    // mag is configured for 18 Hz
    // software oversampeling 9 values
    // => 2 Hz
    const NUM_SAMPLES: usize = 9;
    let mut mag_oversampeler = Oversampeling::new(NUM_SAMPLES, MagOvsWrapper([0i64; 3]));
    loop {
        match mag.read_data_when_ready_interrupt().await {
            Ok(data) => {
                if let Some(value) = mag_oversampeler.insert(data.into()) {
                    sender.publish(SensorData::Mag(value)).await;
                }
            }
            Err(e) => error!("could not read magneto: {}", e),
        }

        led.toggle();
    }
}

// imu polling task
#[embassy_executor::task(pool_size = 2)]
pub async fn imu_task(
    id: u8,
    sender: SensPub,
    mut imu: Lsm6dsv32<'static, FifoDisabled, Int1Disabled, Int2Disabled>,
    mut led: Output<'static>,
) {
    imu.config = south_common::configs::imu_config::get_imu_config();

    let mut imu = imu.enable_interrupt1();
    imu.commit_config().await;

    // Imu is configured for 1.92 KHz ODR
    // software oversampeling 20 values
    // => 96 Hz
    const NUM_SAMPLES: usize = 20;
    let mut accel_oversampeler = Oversampeling::new(NUM_SAMPLES, AccelOvsWrapper([[0i64; 3]; 2]));
    let mut gyro_oversampeler = Oversampeling::new(NUM_SAMPLES, GyroOvsWrapper([0i64; 3]));

    loop {
        if let Err(e) = imu
            .wait_for_data_ready_interrupt1(
                true,  // Accel
                true,  // Gyro
                false, // Temp
                LogicOp::AND,
            )
            .await
        {
            error!("could not wait for imu: {}", e);
            continue;
        }

        match imu.read_accel_dual_raw().await {
            Ok(data) => {
                if let Some(value) = accel_oversampeler.insert(data.into()) {
                    sender.publish(SensorData::Accel(id, value)).await;
                }
            }
            Err(e) => error!("could not read accel: {}", e),
        }

        match imu.read_gyro_raw().await {
            Ok(data) => {
                if let Some(value) = gyro_oversampeler.insert(data.into()) {
                    sender.publish(SensorData::Gyro(id, value)).await;
                }
            }
            Err(e) => error!("could not read gyro: {}", e),
        }

        led.toggle();
    }
}

fn gps_to_unix_second_ceil(week: u16, ms_of_week: u64) -> u64 {
    const LEAP_SECONDS: u64 = 18; // leap seconds between 1980 and 2026
    const SECONDS_PER_GPS_WEEK: u64 = 7 * 24 * 60 * 60;
    const EPOCH_DIFF_SECONDS: u64 = 315964800;

    EPOCH_DIFF_SECONDS - LEAP_SECONDS + SECONDS_PER_GPS_WEEK * week as u64 + ms_of_week / 1000 + 1
}
// phoenix polling task
#[embassy_executor::task]
pub async fn phoenix_task(
    sender: SensPub,
    mut phoenix: PhoenixService<
        RingBufferedUartRx<'static>,
        UartTx<'static, Async>,
        Delay,
        Output<'static>,
        EmbassyClock,
        256,
    >,
) {
    loop {
        match phoenix.next_event().await {
            Ok(PhoenixEvent::StateChanged(state)) => {
                info!("phoenix state={:?}", Debug2Format(&state));
            }
            Ok(PhoenixEvent::Fix3D { tt3d_ms }) => {
                info!("phoenix 3d lock tt3d_ms={=u64}", tt3d_ms);
            }
            Ok(PhoenixEvent::Warning(w)) => {
                warn!("phoenix warning={:?}", Debug2Format(&w));
            }
            Ok(PhoenixEvent::CommandResponse(resp)) => {
                info!("phoenix cmd resp: {}", resp.text.as_str());
            }
            Ok(PhoenixEvent::Message(msg)) => {
                match msg {
                    phoenix::gps::GpsMessage::F40(msg) => {
                        // syncing time
                        super::TIME_REF_UPD_SECOND.store(
                            gps_to_unix_second_ceil(msg.gps_week, msg.gps_seconds_of_week_ms),
                            core::sync::atomic::Ordering::Release,
                        );

                        sender.publish(SensorData::Gps(msg)).await;
                    }
                    _ => (),
                }
            }
            Err(e) => {
                warn!("phoenix event error: {:?}", Debug2Format(&e));
            }
        }
    }
}
