mod helpers;

use defmt::{Debug2Format, error, info, warn};
use embassy_stm32::{
    gpio::Output,
    mode::Async,
    peripherals::{DMA1_CH1, DMA1_CH2, I2C1, PB8, PB9},
    usart::{RingBufferedUartRx, UartTx},
};
use embassy_time::{Duration, Ticker};

use hscmrnn030pa::driver::Baro;
use lsm6dsv32::driver::{FifoDisabled, Int1Disabled, Int2Disabled, LogicOp, Lsm6dsv32};

use phoenix::phoenix::{PhoenixEvent, PhoenixService};
use rm3100::driver::RM3100;
use south_common::{
    chell::ChellDefinition,
    definitions::telemetry::upper_sensor as tm,
    types::{Vector3i32, upper_sensor::AccelRaw}, utils::Oversampeling,
};

use crate::{
    Irqs, TMSender, UpperSensorTMContainer,
    embassy_adapter::{EmbassyClock, EmbassyTimer, LiftoffPin},
};

use helpers::*;

// baro polling task
#[embassy_executor::task]
pub async fn baro_thread(
    tm_sender: TMSender,
    mut baro: Baro<'static, I2C1, PB8, PB9, Irqs, DMA1_CH1, DMA1_CH2>,
    mut led: Output<'static>,
) {
    // reading baro every 500 Millis (2Hz)
    const BARO_LOOP_LEN: Duration = Duration::from_millis(500);
    let mut ticker = Ticker::every(BARO_LOOP_LEN);
    loop {
        match baro.read_out().await {
            Ok(raw) => {
                // let temp = raw.baro_temp_convert();
                // let pressure = raw.baro_pressure_convert_pa();
                let container = UpperSensorTMContainer::new(&tm::Baro, &raw.pressure_data).unwrap();
                tm_sender.send(container).await;
            }
            Err(e) => {
                error!("error reading baro: {}", e);
            }
        }

        led.toggle();
        ticker.next().await;
    }
}

// mag polling task
#[embassy_executor::task]
pub async fn mag_thread(tm_sender: TMSender, mut mag: RM3100<'static>, mut led: Output<'static>) {
    // mag is configured for 18 Hz
    // software oversampeling 9 values
    // => 2 Hz
    let mut mag_oversampeler = Oversampeling::new(9, MagOvsWrapper([0i64; 3]));
    loop {
        match mag.read_data_when_ready_interrupt().await {
            Ok(data) => {
                if let Some(value) = mag_oversampeler.insert(data) {
                    let container = UpperSensorTMContainer::new(&tm::Magneto, &value).unwrap();
                    tm_sender.send(container).await;
                }
            }
            Err(e) => error!("could not read magneto: {}", e),
        }

        led.toggle();
    }
}

// imu polling task
#[embassy_executor::task(pool_size = 2)]
pub async fn imu_thread(
    tm_sender: TMSender,
    mut imu: Lsm6dsv32<'static, FifoDisabled, Int1Disabled, Int2Disabled>,
    mut led: Output<'static>,
    accel_def: &'static dyn ChellDefinition,
    gyro_def: &'static dyn ChellDefinition,
) {
    imu.config = south_common::configs::imu_config::get_imu_config();

    let mut imu = imu.enable_interrupt1();
    imu.commit_config().await;

    // Imu is configured for 1.92 KHz ODR
    // (for now) software oversampeling 192 values
    // => 10 Hz
    let mut accel_oversampeler = Oversampeling::new(192, AccelOvsWrapper([[0i64; 3]; 2]));
    let mut gyro_oversampeler = Oversampeling::new(192, GyroOvsWrapper([0i64; 3]));

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
                if let Some(value) = accel_oversampeler.insert(data) {
                    let value: AccelRaw = value.into();
                    let container = UpperSensorTMContainer::new(accel_def, &value).unwrap();
                    tm_sender.send(container).await;
                }
            }
            Err(e) => error!("could not read accel: {}", e),
        }

        match imu.read_gyro_raw().await {
            Ok(data) => {
                if let Some(value) = gyro_oversampeler.insert(data) {
                    let container = UpperSensorTMContainer::new(gyro_def, &value).unwrap();
                    tm_sender.send(container).await;
                }
            }
            Err(e) => error!("could not read gyro: {}", e),
        }

        // match imu.read_temp_raw().await {
        //     Ok(data) => {
        //         let container = UpperSensorTMContainer::new(temp_def, &data).unwrap();
        //         tm_sender.send(container).await;
        //     }
        //     Err(e) => error!("could not read temp: {}", e),
        // }

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
pub async fn phoenix_thread(
    tm_sender: TMSender,
    mut phoenix: PhoenixService<
        RingBufferedUartRx<'static>,
        UartTx<'static, Async>,
        EmbassyTimer,
        LiftoffPin<'static>,
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

                        // sending tm
                        let ecef = Vector3i32 {
                            x: msg.x_wgs84_cm as i32,
                            y: msg.y_wgs84_cm as i32,
                            z: msg.z_wgs84_cm as i32,
                        };

                        let vel = Vector3i32 {
                            x: msg.vx_wgs84_1e5_mps as i32,
                            y: msg.vy_wgs84_1e5_mps as i32,
                            z: msg.vz_wgs84_1e5_mps as i32,
                        };

                        let state = msg.navigation_status | (msg.tracked_satellites << 2);

                        let container = UpperSensorTMContainer::new(&tm::gps::Pos, &ecef).unwrap();
                        tm_sender.send(container).await;

                        let container = UpperSensorTMContainer::new(&tm::gps::Vel, &vel).unwrap();
                        tm_sender.send(container).await;

                        let container =
                            UpperSensorTMContainer::new(&tm::gps::Status, &state).unwrap();
                        tm_sender.send(container).await;
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
