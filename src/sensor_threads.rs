
use embassy_stm32::{gpio::Output, peripherals::{DMA1_CH1, DMA1_CH2, I2C1, PB8, PB9}};
use embassy_sync::channel::DynamicSender;
use embassy_time::{Duration, Ticker};
use defmt::{error, unwrap, info, warn};

use lsm6dsv32::driver::{FifoDisabled, HighAccuracyODR, Int1Disabled, Int2Disabled, LogicOp, Lsm6dsv32};
use hscmrnn030pa::driver::Baro;

use phoenix::driver::{Message, Phoenix};
use south_common::{
    tmtc_system::TelemetryDefinition,
    definitions::telemetry::upper_sensor as tm
};

use crate::{Irqs, UpperSensorTMContainer, PHOENIX_RX_BUF_SIZE};

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
    _temp_def: &'static dyn TelemetryDefinition,
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
    mut phoenix: Phoenix<'static, PHOENIX_RX_BUF_SIZE>,
) {
    loop {
        match phoenix.next_message().await {
            Ok(msg) => {
                match msg {
                    Message::F00(f00) => {
                        info!("Received F00");
                        let container =
                            UpperSensorTMContainer::new(&tm::gps::Pos, &[f00.lat, f00.lon, f00.height]).unwrap();
                        tm_sender.send(container).await;
                        info!("nav: {}, system: {}, sv: {}", f00.nav_status, f00.system_status, f00.sv);

                        let status = (f00.nav_status & 0b11) << 0
                                   | (f00.system_status & 0b11) << 2
                                   | f00.sv.min(0b1111) << 4;

                        let container =
                            UpperSensorTMContainer::new(&tm::gps::Status, &status).unwrap();
                        tm_sender.send(container).await;
                    }
                    Message::F44(_f44) => {
                        info!("Received F44");
                    }
                    Message::Unknown(id) => {
                        warn!("Received unknown message with ID: {}", id);
                    }
                }
            }
            Err(e) => {
                error!("Error reading message: {:?}", e);
            }
        }
    }
}
