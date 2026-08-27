use embassy_sync::pubsub::WaitResult;
use embassy_time::{Instant, TICK_HZ};
use south_common::chell::ParsableChellValue;
use nalgebra as na;

use crate::{
    SensSub, SensorData,
    sensor_fusion::{
        fusion::FlightData,
        math_utils::{NedConvert, ecef_to_ned},
    },
};

use south_common::definitions::telemetry::upper_sensor as tm;

pub mod fusion;
pub mod math_utils;

async fn get_init_data(sensor_sub: &mut SensSub) -> FlightData {
    let mut pressure: f64 = 0.;
    loop {
        let WaitResult::Message(sensor_value) = sensor_sub.next_message().await else {
            defmt::unreachable!();
        };
        match sensor_value {
            SensorData::Baro(value) => {
                pressure = value.parser(tm::Baro).pa() as f64;
            }
            SensorData::Gps(value) => {
                const NAV_STATUS_LOCK: u8 = 2;
                if value.navigation_status < NAV_STATUS_LOCK {
                    continue;
                }

                let ecef = na::Vector3::new(
                    value.x_wgs84_cm as i32,
                    value.y_wgs84_cm as i32,
                    value.z_wgs84_cm as i32,
                );

                let ecef_m = ecef.parser(tm::gps::Pos).m();
                let llh = ecef.parser(tm::gps::Pos).llh();

                let mut fd = FlightData::default();
                fd.x = ecef_m.x;
                fd.y = ecef_m.y;
                fd.z = ecef_m.z;
                fd.lat = llh.lat;
                fd.lon = llh.lon;
                fd.alt = llh.h;

                if pressure != 0. {
                    fd.pressure = pressure;
                    return fd;
                }
            }
            _ => (),
        }
    }
}

fn secs_now() -> f64 {
    Instant::now().as_ticks() as f64 / TICK_HZ as f64
}

fn populate_flight_data(value: SensorData, ned_convert: &NedConvert) -> Result<FlightData, ()> {
    let mut fd = FlightData::default();
    match value {
        SensorData::Accel(id, value) => match id {
            1 => {
                let accel_mps = value.parser(tm::imu1::Accel).mps();
                fd.accel_x_1 = accel_mps.x as f64;
                fd.accel_y_1 = accel_mps.y as f64;
                fd.accel_z_1 = accel_mps.z as f64;
            }
            2 => {
                let accel_mps = value.parser(tm::imu2::Accel).mps();
                fd.accel_x_2 = accel_mps.x as f64;
                fd.accel_y_2 = accel_mps.y as f64;
                fd.accel_z_2 = accel_mps.z as f64;
            }
            _ => defmt::unreachable!(),
        },
        SensorData::Gyro(id, value) => match id {
            1 => {
                let gyro_rps = value.parser(tm::imu1::Gyro).rps();
                fd.yaw_1 = gyro_rps.x as f64;
                fd.pitch_1 = gyro_rps.y as f64;
                fd.roll_1 = gyro_rps.z as f64;
            }
            2 => {
                let gyro_rps = value.parser(tm::imu2::Gyro).rps();
                fd.yaw_2 = gyro_rps.x as f64;
                fd.pitch_2 = gyro_rps.y as f64;
                fd.roll_2 = gyro_rps.z as f64;
            }
            _ => defmt::unreachable!(),
        },
        SensorData::Baro(value) => {
            let pressure_pa = value.parser(tm::Baro).pa();
            fd.pressure = pressure_pa as f64;
        }
        SensorData::Gps(value) => {
            const NAV_STATUS_LOCK: u8 = 2;
            if value.navigation_status < NAV_STATUS_LOCK {
                return Err(());
            }

            let ecef = na::Vector3::new(
                value.x_wgs84_cm as i32,
                value.y_wgs84_cm as i32,
                value.z_wgs84_cm as i32,
            );
            let ecef_m = ecef.parser(tm::gps::Pos).m();
            let ned = ecef_to_ned(ecef_m.x, ecef_m.y, ecef_m.z, &ned_convert);
            let llh = ecef.parser(tm::gps::Pos).llh();
            fd.x = ned[0];
            fd.y = ned[1];
            fd.z = ned[2];
            fd.lat = llh.lat;
            fd.lon = llh.lon;
            fd.alt = llh.h;
        }
        _ => (),
    }
    Ok(fd)
}

async fn run_ekf(sensor_sub: &mut SensSub, init_data: FlightData) {
    // Init EKF
    let start_pressure = init_data.pressure;
    let (ned_convert, mut ekf) = fusion::FlightManager::new(init_data);

    let start_time = secs_now();
    let mut last_time = 0.;

    loop {
        let WaitResult::Message(sensor_value) = sensor_sub.next_message().await else {
            defmt::unreachable!();
        };
        let Ok(fd) = populate_flight_data(sensor_value, &ned_convert) else {
            continue;
        };

        let current_time = secs_now() - start_time;
        let dt = current_time - last_time;
        last_time = current_time;
        ekf.run_ekf_on_flightdata(fd, current_time, dt, start_pressure);
    }
}

// EKF task
#[embassy_executor::task]
pub async fn fusion_task(mut sensor_sub: SensSub) {
    let init_flight_data = get_init_data(&mut sensor_sub).await;

    run_ekf(&mut sensor_sub, init_flight_data).await;
}
