use crate::sensor_fusion::math_utils::{
    NedConvert, measurement_function, measurement_jacobian, normalize_quaternion, state_transition,
    state_transition_jacobian,
};
use nalgebra::{ComplexField, SMatrix, SVector, UnitQuaternion, Vector3};

const CALIB_TIME: f64 = 5.;

pub struct FlightData {
    pub accel_x_1: f64,
    pub accel_y_1: f64,
    pub accel_z_1: f64,
    pub accel_x_2: f64,
    pub accel_y_2: f64,
    pub accel_z_2: f64,

    pub roll_1: f64,
    pub pitch_1: f64,
    pub yaw_1: f64,

    pub roll_2: f64,
    pub pitch_2: f64,
    pub yaw_2: f64,

    pub lat: f64,
    pub lon: f64,
    pub alt: f64,

    pub x: f64,
    pub y: f64,
    pub z: f64,

    pub pressure: f64,
}
impl Default for FlightData {
    fn default() -> Self {
        Self {
            accel_x_1: 0.,
            accel_y_1: 0.,
            accel_z_1: 0.,
            accel_x_2: 0.,
            accel_y_2: 0.,
            accel_z_2: 0.,
            roll_1: 0.,
            pitch_1: 0.,
            yaw_1: 0.,
            roll_2: 0.,
            pitch_2: 0.,
            yaw_2: 0.,
            lat: 0.,
            lon: 0.,
            alt: 0.,
            x: 0.,
            y: 0.,
            z: 0.,
            pressure: 0.
        }
    }
}

struct RocketEKF {
    state: SVector<f64, 23>,
    p: SMatrix<f64, 23, 23>,
    q: SMatrix<f64, 23, 23>,
    r: SMatrix<f64, 10, 10>,
    baro_needs_sync: bool,
}

impl RocketEKF {
    fn new(data: FlightData) -> (NedConvert, Self) {
        let g_ned = Vector3::new(0.0, 0.0, 9.8);
        let g_body = Vector3::new(data.accel_x_1, data.accel_y_1, data.accel_z_1);
        let g_ned_norm = g_ned.normalize();
        let g_body_norm = g_body.normalize();

        // Quaternion x, y, z, w
        let q_i2b = UnitQuaternion::rotation_between(&g_ned_norm, &g_body_norm).unwrap();

        //Kalman matrix initalization
        type StateVector = SVector<f64, 23>;
        let mut x = StateVector::zeros();

        // Position
        x[0] = 0.0;
        x[1] = 0.0;
        x[2] = 0.0;
        // 3, 4, 5 = 0 -> Speed
        // Acceleration
        x[6] = data.accel_x_1;
        x[7] = data.accel_y_1;
        x[8] = data.accel_z_1;
        // 9, 10, 11 = 0 -> Gyroscop
        // Quaternions
        x[12] = q_i2b.w;
        x[13] = q_i2b.i;
        x[14] = q_i2b.j;
        x[15] = q_i2b.k;
        //Biases, 16, 17, 18, 19, 20, 21 = 0
        x[22] = 0.0;

        let p = SMatrix::<f64, 23, 23>::identity() * 0.1; // covariance
        let mut q = SMatrix::<f64, 23, 23>::identity() * 0.01; // process noise
        let mut r = SMatrix::<f64, 10, 10>::identity() * 0.5; // measurment noise

        // old initialization values
        q[(22, 22)] = 1e-3;
        r[(0, 0)] = 0.01;
        r[(1, 1)] = 0.01;
        r[(2, 2)] = 0.01;
        r[(9, 9)] = 10_000.0;

        (
            NedConvert {
                x_ref: data.x,
                y_ref: data.y,
                z_ref: data.z,
                lat_ref: data.lat,
                lon_ref: data.lon,
            },
            Self {
                state: x,
                p,
                q,
                r,
                baro_needs_sync: false,
            },
        )
    }

    fn predict(&mut self, dt: f64) {
        self.p = (&self.p + self.p.transpose()) * 0.5;
        let f = state_transition_jacobian(&self.state, dt);
        self.state = state_transition(&self.state, dt);

        // P(k+1|k) = A*P(k)*A^T + Q -> Vorhersage Kovarianz
        self.p = f * self.p * f.transpose() + self.q;

        let q_slice = self.state.fixed_rows::<4>(12);
        let q_raw: [f64; 4] = [q_slice[0], q_slice[1], q_slice[2], q_slice[3]];

        let q_norm = normalize_quaternion(q_raw);
        self.state.fixed_rows_mut::<4>(12).copy_from_slice(&q_norm);
    }
    fn update(&mut self, z_measured: &SVector<f64, 10>, mask: &[bool; 10]) {
        let z_pred_full = measurement_function(&self.state, false);
        let h_full = measurement_jacobian(&self.state);

        if !mask.iter().any(|&active| active) {
            return;
        }

        let mut z_pred = SVector::<f64, 10>::zeros();
        let mut h = SMatrix::<f64, 10, 23>::zeros();
        let mut r = SMatrix::<f64, 10, 10>::zeros();
        let mut innovation = SVector::<f64, 10>::zeros();

        for i in 0..10 {
            if mask[i] {
                // active sensor = real number
                z_pred[i] = z_pred_full[i];
                h.set_row(i, &h_full.row(i));
                innovation[i] = z_measured[i] - z_pred[i];

                for j in 0..10 {
                    if mask[j] {
                        r[(i, j)] = self.r[(i, j)];
                    }
                }
            } else {
                // Inactiv sensor -> r = infinty, so kalman gain equals nearly 0
                r[(i, i)] = 1e15;
            }
        }

        let mut s = &h * &self.p * h.transpose() + &r;
        s = (&s + s.transpose()) / 2.0;

        let s_inv = s
            .clone()
            .lu()
            .try_inverse()
            .expect("S matrix inversion failed");

        let mut k = &self.p * h.transpose() * s_inv;

        // quat slow with little gain
        for i in 0..4 {
            for col in 0..10 {
                k[(12 + i, col)] *= 0.05;
            }
        }

        if mask[2] {
            let h_innovation = innovation[2];

            if h_innovation.abs() > 1000.0 {
                self.state[0] = z_measured[0];
                self.state[1] = z_measured[1];
                self.state[2] = z_measured[2];

                // increasing baro bias
                self.p[(22, 22)] = 100.0;
                innovation[2] = 0.0;

                // increasing gps uncertainty
                self.p
                    .fixed_view_mut::<3, 3>(0, 0)
                    .copy_from(&(self.r.fixed_view::<3, 3>(0, 0) * 5.0));
                r.fixed_view_mut::<3, 3>(0, 0).scale_mut(5.0);

                self.p.fixed_view_mut::<3, 3>(0, 3).scale_mut(0.05);
                self.p.fixed_view_mut::<3, 3>(3, 0).scale_mut(0.05);
                self.baro_needs_sync = true;
            }
        }

        if mask[9] {
            if self.baro_needs_sync {
                let baro_meas = z_measured[9];
                self.state[22] = self.state[2] - baro_meas;
                self.baro_needs_sync = false;
                self.p[(22, 22)] = 100.0;
                innovation[9] = 0.0;
            }
        }

        let correction = &k * innovation;
        self.state += correction;

        // Kovarianz (Joseph Form)
        // P = (I - K @ H) @ P @ (I - K @ H).T + K @ R @ K.T
        let i = SMatrix::<f64, 23, 23>::identity();
        let i_kh = i - (&k * h);
        self.p = &i_kh * &self.p * i_kh.transpose() + &k * r * k.transpose();

        // quaternion normalize
        // w, x, y, z
        let q_raw = [
            self.state[12],
            self.state[13],
            self.state[14],
            self.state[15],
        ];
        let q_norm = normalize_quaternion(q_raw);
        self.state.fixed_rows_mut::<4>(12).copy_from_slice(&q_norm);

        // covarianzmatrix symmetrical
        self.p = (&self.p + self.p.transpose()) / 2.0;
    }
}

pub struct FlightManager {
    rocket_ekf: RocketEKF,
    rocket_started: bool,
    ascent_flag: bool,
    calibration_active: bool,
    calibration_start_time: f64,
    calibration_count: usize,
    last_valid_gps: usize,

    z_prev: Option<SVector<f64, 10>>,

    accel_gyro_buffer: [[f64; 6]; 20],
    accel_gyro_head: usize,
    accel_gyro_len: usize,

    altitude_buffer: [f64; 200],
    altitude_head: usize,
    altitude_len: usize,
}

impl FlightManager {
    pub fn new(data: FlightData) -> (NedConvert, Self) {
        let (ned_convert, rocket_ekf) = RocketEKF::new(data);
        (
            ned_convert,
            Self {
                rocket_ekf,
                rocket_started: false,
                ascent_flag: true,
                calibration_active: true,
                calibration_start_time: 0.0,
                calibration_count: 0,
                last_valid_gps: 0,
                z_prev: None,
                accel_gyro_buffer: [[0.0; 6]; 20],
                accel_gyro_head: 0,
                accel_gyro_len: 0,
                altitude_buffer: [0.0; 200],
                altitude_head: 0,
                altitude_len: 0,
            },
        )
    }

    // Help function: Add Number to IMU ring buffer
    fn push_accel_gyro(&mut self, val: [f64; 6]) {
        self.accel_gyro_buffer[self.accel_gyro_head] = val;
        self.accel_gyro_head = (self.accel_gyro_head + 1) % 20; // Modulo lässt Index im Kreis laufen
        if self.accel_gyro_len < 20 {
            self.accel_gyro_len += 1;
        }
    }

    // Help fucntion: calculate median of IMU
    fn mean_accel_gyro(&self) -> [f64; 6] {
        let mut mean = [0.0; 6];
        if self.accel_gyro_len == 0 {
            return mean;
        }

        for i in 0..self.accel_gyro_len {
            for j in 0..6 {
                mean[j] += self.accel_gyro_buffer[i][j];
            }
        }
        for j in 0..6 {
            mean[j] /= self.accel_gyro_len as f64;
        }
        mean
    }

    fn push_altitude(&mut self, alt: f64) {
        self.altitude_buffer[self.altitude_head] = alt;
        self.altitude_head = (self.altitude_head + 1) % 200;
        if self.altitude_len < 200 {
            self.altitude_len += 1;
        }
    }

    fn mean_altitude(&self) -> f64 {
        if self.altitude_len == 0 {
            return 0.0;
        }
        let sum: f64 = self.altitude_buffer[0..self.altitude_len].iter().sum();
        sum / self.altitude_len as f64
    }
    pub fn run_ekf_on_flightdata(
        &mut self,
        data: FlightData,
        current_time: f64,
        dt: f64,
        start_pressure: f64,
    ) {
        let cur_accel = [
            (data.accel_x_1 + data.accel_x_2) as f64 / 2.0,
            (data.accel_y_1 + data.accel_y_2) as f64 / 2.0,
            (data.accel_z_1 + data.accel_z_2) as f64 / 2.0,
        ];
        let cur_gyro = [
            (data.roll_1 + data.roll_2) as f64 / 2.0,
            (data.pitch_1 + data.pitch_2) as f64 / 2.0,
            (data.yaw_1 + data.yaw_2) as f64 / 2.0,
        ];

        // low pass
        self.push_accel_gyro([
            cur_accel[0],
            cur_accel[1],
            cur_accel[2],
            cur_gyro[0],
            cur_gyro[1],
            cur_gyro[2],
        ]);

        let mut mean_measurement = [0.0; 6];
        if self.accel_gyro_len > 0 {
            for i in 0..self.accel_gyro_len {
                for j in 0..6 {
                    mean_measurement[j] += self.accel_gyro_buffer[i][j];
                }
            }
            for j in 0..6 {
                mean_measurement[j] /= self.accel_gyro_len as f64;
            }
        }

        // calibration
        if self.calibration_active && (current_time - self.calibration_start_time <= CALIB_TIME) {
            // 5s Dauer
            self.calibration_count += 1;
            self.rocket_ekf
                .q
                .fixed_view_mut::<4, 4>(12, 12)
                .copy_from(&(SMatrix::<f64, 4, 4>::identity() * 1e-9));
            for j in 3..6 {
                mean_measurement[j] = 0.0;
            }
        } else if self.calibration_active {
            self.calibration_active = false;
        }

        let total_accel = (cur_accel[0] * cur_accel[0]
            + cur_accel[1] * cur_accel[1]
            + cur_accel[2] * cur_accel[2])
            .sqrt();
        if total_accel > 12.0 {
            self.rocket_started = true;
        }

        if self.rocket_started {
            self.push_altitude(self.rocket_ekf.state[2]);

            let mean_alt = self.mean_altitude();

            if self.ascent_flag && (self.rocket_ekf.state[2] * 1.05 < mean_alt) {
                self.ascent_flag = false;
            }
        }

        // predict
        if dt > 0.0 {
            self.rocket_ekf.predict(dt);
        }

        let mut z_measured = SVector::<f64, 10>::zeros();
        z_measured[0] = data.x;
        z_measured[1] = data.y;
        z_measured[2] = data.z;
        for j in 0..6 {
            z_measured[3 + j] = mean_measurement[j];
        }
        z_measured[9] = data.pressure - start_pressure;

        let mut mask = [false; 10];
        if let Some(prev) = self.z_prev {
            for j in 0..10 {
                if (z_measured[j] - prev[j]).abs() > 1e-9 {
                    mask[j] = true;
                }
            }
        } else {
            mask = [true; 10];
        }
        if mask[9] {
            let baro_vs_gps = (z_measured[9] - z_measured[2]).abs();
            if baro_vs_gps > 100.0 {
                mask[9] = false;
            }
        }
        self.rocket_ekf.update(&z_measured, &mask);
    }
}
