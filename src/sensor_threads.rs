use embassy_stm32::{gpio::Output, mode::Async, peripherals::{DMA1_CH1, DMA1_CH2, I2C1, PB8, PB9}, usart::{RingBufferedUartRx, UartTx}};
use embassy_sync::channel::DynamicSender;
use embassy_time::{Duration, Ticker};
use defmt::{Debug2Format, error, info, unwrap, warn};

use lsm6dsv32::driver::{FifoDisabled, HighAccuracyODR, Int1Disabled, Int2Disabled, LogicOp, Lsm6dsv32};
use hscmrnn030pa::driver::Baro;

use phoenix::phoenix::{PhoenixEvent, PhoenixService};
use south_common::{
    definitions::telemetry::upper_sensor as tm, tmtc_system::TelemetryDefinition, types::upper_sensor::BaroRaw
};

use crate::{Irqs, UpperSensorTMContainer, embassy_adapter::{EmbassyClock, EmbassyTimer, LiftoffPin}};

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
                let raw = BaroRaw {
                    status: raw.status,
                    pressure_data: raw.pressure_data,
                    temperature_data: raw.temperature_data,
                };

                let container =
                    UpperSensorTMContainer::new(&tm::Baro, &raw).unwrap();
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
    accel_low_range_def: &'static dyn TelemetryDefinition,
    accel_full_range_def: &'static dyn TelemetryDefinition,
    gyro_def: &'static dyn TelemetryDefinition,
) {
    //const READ_TEMPERATURE_INTERVAL: Duration = Duration::from_secs(1);
    // high accuracy mode
    imu.config.use_high_accuracy_mode(HighAccuracyODR::Standard);

    // config
    imu.config.accel.dual_channel = true;
    imu.config.accel.full_scale = lsm6dsv32::driver::AccelFS::G8;

    // active low interrupt
    imu.config.general.interrupt_lvl = true;

    unwrap!(
        imu.config
            .accel
            .set_odr(lsm6dsv32::driver::AccelODR::KHz1_92)
    );
    unwrap!(imu.config.gyro.set_odr(lsm6dsv32::driver::GyroODR::KHz1_92));

    
    imu.commit_config().await;
    

    let mut imu = imu.enable_interrupt1();
    imu.config.int1.data_ready_accel = true;
    imu.config.int1.data_ready_gyro = true;

    imu.commit_config().await;

    loop {
        match imu.wait_for_data_ready_interrupt1(
            true, // Accel
            true, // Gyro
            false, // Temp
            LogicOp::AND,
        ).await {
            Ok(_) => {
                match imu.read_accel_dual_raw().await {
                    Ok((low, full)) => {
                        let container = UpperSensorTMContainer::new(accel_low_range_def, &low).unwrap();
                        tm_sender.send(container).await;

                        let container = UpperSensorTMContainer::new(accel_full_range_def, &full).unwrap();
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

// phoenix polling task
#[embassy_executor::task]
pub async fn phoenix_thread(
    tm_sender: DynamicSender<'static, UpperSensorTMContainer>,
    mut phoenix: PhoenixService<RingBufferedUartRx<'static>, UartTx<'static, Async>, EmbassyTimer, LiftoffPin<'static>, EmbassyClock, 256>,
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
                        let ecef = [
                            msg.x_wgs84_cm as i32,
                            msg.y_wgs84_cm as i32,
                            msg.z_wgs84_cm as i32,
                        ];

                        let vel = [
                            msg.vx_wgs84_1e5_mps as i32,
                            msg.vy_wgs84_1e5_mps as i32,
                            msg.vz_wgs84_1e5_mps as i32,
                        ];

                        let state = msg.navigation_status
                            | (msg.tracked_satellites << 2);

                        let container = UpperSensorTMContainer::new(&tm::gps::ECEF, &ecef).unwrap();
                        tm_sender.send(container).await;

                        let container = UpperSensorTMContainer::new(&tm::gps::Vel, &vel).unwrap();
                        tm_sender.send(container).await;

                        let container = UpperSensorTMContainer::new(&tm::gps::Status, &state).unwrap();
                        tm_sender.send(container).await;
                    }
                    _ => ()
                }
            }
            Err(e) => {
                warn!("phoenix event error: {:?}", Debug2Format(&e));
            }
        }
    }
}
