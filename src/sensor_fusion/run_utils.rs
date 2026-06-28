#![cfg(not(target_arch = "arm"))] // Verhindert, dass der ARM-Cortex-Compiler diese Datei anfasst!

use std::fs::File;
use std::error::Error;
use nalgebra::SVector;
use csv::Reader;
use crate::sensor_fusion::math_utils::{FlightData, FlightManager, RocketEKF};


pub fn run_embedded_simulation(
    input_path: &str,
    output_path: &str,
    ekf: &mut RocketEKF,
    manager: &mut FlightManager,
) -> Result<(), Box<dyn Error>> {
    let file = File::open(input_path)?;
    let mut rdr = csv::Reader::from_reader(file);

    let mut estimated_states = Vec::new();
    let mut prev_time: Option<f64> = None;
    let mut start_pressure: Option<f64> = None;

    println!("Starte Embedded-Filter Simulation mit synchronisierten Daten...");

    for result in rdr.deserialize() {
        let row: VsInputRow = result?;

        let current_time = row.timestamp;
        let dt = match prev_time {
            Some(t) => current_time - t,
            None => 0.0,
        };

        if start_pressure.is_none() {
            // Da du in der vsinput.csv bereits die umgerechnete barometrische Höhe 
            // oder den Druck hast, nutzen wir den ersten Wert als Referenz
            start_pressure = Some(row.pressure_alt as f64);
        }

        // 3. Verpacken in das schlanke, neue FlightData-Einzel-Struct
        // Da du in vsinput.csv nur IMU_1 exportiert hast, spiegeln wir die Daten hier 
        // für IMU_2, damit deine Sensor-Fusion (Mittelwert) exakte Werte bekommt.
        let mut single_sample = FlightData {
            accel_x_1: row.accel_x_1,
            accel_y_1: row.accel_y_1,
            accel_z_1: row.accel_z_1,
            
            accel_x_2: row.accel_x_1, // Redundanz simuliert
            accel_y_2: row.accel_y_1,
            accel_z_2: row.accel_z_1,

            roll_1: row.gyro_roll_1,
            pitch_1: row.gyro_pitch_1,
            yaw_1: row.gyro_yaw_1,
            
            roll_2: row.gyro_roll_1,
            pitch_2: row.gyro_pitch_1,
            yaw_2: row.gyro_yaw_1,

            x: row.ecef_x,
            y: row.ecef_y,
            z: row.ecef_z,
            pressure: row.pressure_alt, // Enthält deine gefilterte Höhe/Druck
        };

        // 4. Den neuen Echtzeit-Filter füttern
        manager.run_ekf_on_flightdata(
            &mut single_sample,
            current_time,
            dt,
            start_pressure.unwrap(),
            ekf,
        );

        // 5. Den veränderten Zustand aus dem EKF-Speicher klonen
        estimated_states.push(ekf.state.clone());

        prev_time = Some(current_time);
    }

    // 6. Ergebnisse wegschreiben
    save_output_to_csv(&estimated_states, output_path)?;

    Ok(())
}

fn save_output_to_csv(results: &Vec<SVector<f64, 23>>, file_path: &str) -> Result<(), Box<dyn Error>> {
    let file = File::create(file_path)?;
    let mut wtr = csv::Writer::from_writer(file);

    // Spaltenüberschriften für das EKF-Ergebnis
    wtr.write_record(&[
        "pos_x", "pos_y", "pos_z", 
        "vel_x", "vel_y", "vel_z", 
        "acc_x", "acc_y", "acc_z",
        "gyro_x", "gyro_y", "gyro_z",
        "q_w", "q_x", "q_y", "q_z",
        "bias_1", "bias_2", "bias_3", "bias_4", "bias_5", "bias_6",
        "baro_bias"
    ])?;

    for state in results {
        let row: Vec<String> = state.iter().map(|val| val.to_string()).collect();
        wtr.write_record(&row)?;
    }

    wtr.flush()?;
    println!("Simulation beendet. EKF-Ergebnisse unter '{}' gespeichert.", file_path);
    Ok(())
}
