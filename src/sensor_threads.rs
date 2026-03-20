use defmt::{Debug2Format, error, info, warn};
use embassy_stm32::{
    gpio::Output,
    mode::Async,
    peripherals::{DMA1_CH1, DMA1_CH2, I2C1, PB8, PB9},
    usart::{RingBufferedUartRx, UartTx},
};
use embassy_sync::channel::DynamicSender;
use embassy_time::{Duration, Instant, Ticker};

use hscmrnn030pa::driver::Baro;
use lsm6dsv32::driver::{FifoDisabled, Int1Disabled, Int2Disabled, LogicOp, Lsm6dsv32};

use phoenix::phoenix::{PhoenixEvent, PhoenixService};
use south_common::{
    definitions::telemetry::upper_sensor as tm,
    chell::ChellDefinition,
    types::{Vector3i32, upper_sensor::AccelRaw},
};

use crate::{
    Irqs, UpperSensorTMContainer,
    embassy_adapter::{EmbassyClock, EmbassyTimer, LiftoffPin},
};

// baro polling task
#[embassy_executor::task]
pub async fn baro_thread(
    tm_sender: DynamicSender<'static, UpperSensorTMContainer>,
    mut baro: Baro<'static, I2C1, PB8, PB9, Irqs, DMA1_CH1, DMA1_CH2>,
    mut led: Output<'static>,
) {
    const BARO_LOOP_LEN: Duration = Duration::from_millis(200);
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

// imu polling task
#[embassy_executor::task(pool_size = 2)]
pub async fn imu_thread(
    tm_sender: DynamicSender<'static, UpperSensorTMContainer>,
    mut imu: Lsm6dsv32<'static, FifoDisabled, Int1Disabled, Int2Disabled>,
    mut led: Output<'static>,
    accel_def: &'static dyn ChellDefinition,
    gyro_def: &'static dyn ChellDefinition,
) {
    imu.config = south_common::configs::imu_config::get_imu_config();

    let mut imu = imu.enable_interrupt1();
    imu.commit_config().await;

    loop {
        match imu
            .wait_for_data_ready_interrupt1(
                true,  // Accel
                true,  // Gyro
                false, // Temp
                LogicOp::AND,
            )
            .await
        {
            Ok(_) => {
                match imu.read_accel_dual_raw().await {
                    Ok(data) => {
                        let data: AccelRaw = data.into();
                        let container = UpperSensorTMContainer::new(accel_def, &data).unwrap();
                        tm_sender.send(container).await;
                    }
                    Err(e) => error!("could not read accel: {}", e),
                }

                match imu.read_gyro_raw().await {
                    Ok(data) => {
                        let container = UpperSensorTMContainer::new(gyro_def, &data).unwrap();
                        tm_sender.send(container).await;
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
            }
            Err(e) => error!("could not read temp: {}", e),
        }

        led.toggle();
    }
}

fn gps_to_unix_us(week: u16, ms_of_week: u64) -> u64 {
    const LEAP_SECONDS: u64 = 18; // leap seconds between 1980 and 2026
    const MS_PER_GPS_WEEK: u64 = 7 * 24 * 60 * 60 * 1000;
    const EPOCH_DIFF_MS: u64 = 315964800000;

    let unix_ms = EPOCH_DIFF_MS - LEAP_SECONDS * 1000 + MS_PER_GPS_WEEK * week as u64 + ms_of_week;

    unix_ms * 1000
}
// phoenix polling task
#[embassy_executor::task]
pub async fn phoenix_thread(
    tm_sender: DynamicSender<'static, UpperSensorTMContainer>,
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
                        super::TIME_REF.store(
                            gps_to_unix_us(msg.gps_week, msg.gps_seconds_of_week_ms)
                                - Instant::now().as_micros(),
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
