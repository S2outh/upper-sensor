use core::f32;
use libm::{cos, powf, sin, sincos, sqrt};
use nalgebra::{
    Matrix3, Matrix4, Quaternion, Rotation3, SMatrix, SVector, UnitQuaternion, Vector3,
};

pub struct NedConvert {
    pub x_ref: f64,
    pub y_ref: f64,
    pub z_ref: f64,
    pub lat_ref: f64,
    pub lon_ref: f64,
}

pub fn pres_to_alt(pres: f32) -> f32 {
    44_330.0 * (1.0 - powf(pres / 104_315.0, 1.0 / 5.255)) //HyEnD
}

pub fn quaternion_rotation_matrix(q: &[f64; 4]) -> [[f64; 3]; 3] {
    //Covert a quaternion into a full three-dimensional rotation matrix.
    let quaternion =
        UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(q[0], q[1], q[2], q[3]));

    let rotation_matrix: Rotation3<f64> = quaternion.to_rotation_matrix();
    let m = rotation_matrix.matrix();
    [
        [m[(0, 0)], m[(0, 1)], m[(0, 2)]],
        [m[(1, 0)], m[(1, 1)], m[(1, 2)]],
        [m[(2, 0)], m[(2, 1)], m[(2, 2)]],
    ]
}

pub fn compute_d_rotation_d_quaternion(q: &[f64; 4]) -> [Matrix3<f64>; 4] {
    // q[0]=w, q[1]=x, q[2]=y, q[3]=z
    let w = q[0];
    let x = q[1];
    let y = q[2];
    let z = q[3];

    // dR/dw
    let dr_dw = Matrix3::new(
        0.0,
        -4.0 * z,
        4.0 * y,
        4.0 * z,
        0.0,
        -4.0 * x,
        -4.0 * y,
        4.0 * x,
        0.0,
    );

    // dR/dx
    let dr_dx = Matrix3::new(
        0.0,
        4.0 * y,
        4.0 * z,
        4.0 * y,
        -8.0 * x,
        -4.0 * w,
        4.0 * z,
        4.0 * w,
        -8.0 * x,
    );

    // dR/dy
    let dr_dy = Matrix3::new(
        -8.0 * y,
        4.0 * x,
        4.0 * w,
        4.0 * x,
        0.0,
        4.0 * z,
        -4.0 * w,
        4.0 * z,
        -8.0 * y,
    );

    // dR/dz
    let dr_dz = Matrix3::new(
        -8.0 * z,
        -4.0 * w,
        4.0 * x,
        4.0 * w,
        -8.0 * z,
        4.0 * y,
        4.0 * x,
        4.0 * y,
        0.0,
    );

    [dr_dw, dr_dx, dr_dy, dr_dz]
}

pub fn normalize_quaternion(quaternion: [f64; 4]) -> [f64; 4] {
    let norm = sqrt(
        quaternion[0] * quaternion[0]
            + quaternion[1] * quaternion[1]
            + quaternion[2] * quaternion[2]
            + quaternion[3] * quaternion[3],
    );

    [
        quaternion[0] / norm,
        quaternion[1] / norm,
        quaternion[2] / norm,
        quaternion[3] / norm,
    ]
}

pub fn latlonh_to_ecef(lat_deg: f64, lon_deg: f64, h_m: f64) -> [f64; 3] {
    // WGS84 Konstanten
    let a: f64 = 6_378_137.0;
    let e_sq: f64 = 0.006_694_379_990_14;

    let lat = lat_deg.to_radians();
    let lon = lon_deg.to_radians();

    let (sin_lat, cos_lat) = sincos(lat);
    let (sin_lon, cos_lon) = sincos(lon);

    let n = a / sqrt(1.0 - e_sq * sin_lat * sin_lat);
    let x = (n + h_m) * cos_lat * cos_lon;
    let y = (n + h_m) * cos_lat * sin_lon;
    let z = (n * (1.0 - e_sq) + h_m) * sin_lat;

    [x, y, z]
}

pub fn ecef_to_ned(x: f64, y: f64, z: f64, ned_convert: &NedConvert) -> [f64; 3] {
    let ecef_ref = Vector3::new(ned_convert.x_ref, ned_convert.y_ref, ned_convert.z_ref);
    let ecef_current = Vector3::new(x, y, z);
    let delta_ecef = ecef_current - ecef_ref;
    let rotation_matrix = ecef_to_ned_matrix(ned_convert.lat_ref, ned_convert.lon_ref);
    let ned = rotation_matrix * delta_ecef;

    [ned.x, ned.y, ned.z]
}

pub fn ecef_to_ned_matrix(lat_deg: f64, lon_deg: f64) -> Matrix3<f64> {
    let lat = lat_deg.to_radians();
    let lon = lon_deg.to_radians();

    let s_lat = sin(lat);
    let c_lat = cos(lat);
    let s_lon = sin(lon);
    let c_lon = cos(lon);

    // Standard Rotationsmatrix ECEF -> NED
    Matrix3::new(
        -s_lat * c_lon,
        -s_lat * s_lon,
        c_lat,
        -s_lon,
        c_lon,
        0.0,
        -c_lat * c_lon,
        -c_lat * s_lon,
        -s_lat,
    )
}

pub fn state_transition(state: &SVector<f64, 23>, dt: f64) -> SVector<f64, 23> {
    let mut next_state = *state;

    next_state[0] += state[3] * dt;
    next_state[1] += state[4] * dt;
    next_state[2] += state[5] * dt;

    let q = UnitQuaternion::from_quaternion(Quaternion::new(
        state[12], state[13], state[14], state[15],
    ));
    let r_body_to_ned = q.to_rotation_matrix();

    let a_sensor_body = Vector3::new(
        state[6] + state[16],
        state[7] + state[17],
        state[8] + state[18],
    );

    let a_measured_ned = r_body_to_ned * a_sensor_body;

    let g_ned = Vector3::new(0.0, 0.0, 9.8);
    let mut a_ned = a_measured_ned + g_ned;

    if a_ned.norm_squared() < (0.05 * 0.05) {
        a_ned = Vector3::zeros();
    }

    next_state[3] += a_ned.x * dt;
    next_state[4] += a_ned.y * dt;
    next_state[5] += a_ned.z * dt;

    let gx = (state[9] + state[19]) * dt;
    let gy = (state[10] + state[20]) * dt;
    let gz = (state[11] + state[21]) * dt;

    let q_alt = Matrix4::new(
        1.0,
        -gx / 2.0,
        -gy / 2.0,
        -gz / 2.0,
        gx / 2.0,
        1.0,
        gz / 2.0,
        -gy / 2.0,
        gy / 2.0,
        -gz / 2.0,
        1.0,
        gx / 2.0,
        gz / 2.0,
        gy / 2.0,
        -gx / 2.0,
        1.0,
    );

    let current_q = SVector::<f64, 4>::new(state[13], state[14], state[15], state[12]);
    let next_q = (q_alt * current_q).normalize();

    next_state[12] = next_q.w; // w ist das 4. Element im current_q Vektor
    next_state[13] = next_q.x; // x
    next_state[14] = next_q.y; // y
    next_state[15] = next_q.z; // z

    next_state
}

pub fn state_transition_jacobian(state: &SVector<f64, 23>, dt: f64) -> SMatrix<f64, 23, 23> {
    let mut f = SMatrix::<f64, 23, 23>::identity(); // Diagonaleinträge = 1
    f[(0, 3)] = dt;
    f[(1, 4)] = dt;
    f[(2, 5)] = dt;

    // Quaternion of state
    //let q_vec = SVector::<f64, 4>::new(state[12], state[13], state[14], state[15]);
    let q_x = state[13];
    let q_y = state[14];
    let q_z = state[15];
    let q_w = state[12];

    // x, y, z, w
    let q_vec = SVector::<f64, 4>::new(q_x, q_y, q_z, q_w);
    let q = UnitQuaternion::from_quaternion(Quaternion::from_vector(q_vec));
    let r = q.to_rotation_matrix();

    for row in 0..3 {
        for col in 0..3 {
            let val = r[(row, col)] * dt;
            f[(3 + row, 6 + col)] = val;
            f[(3 + row, 16 + col)] = -val; //Bias
        }
    }

    // Gyro + Bias
    let gx = state[9] + state[19];
    let gy = state[10] + state[20];
    let gz = state[11] + state[21];

    // Partial differentiation
    f[(12, 13)] = -0.5 * gx * dt;
    f[(12, 14)] = -0.5 * gy * dt;
    f[(12, 15)] = -0.5 * gz * dt;
    f[(13, 12)] = 0.5 * gx * dt;
    f[(13, 14)] = 0.5 * gz * dt;
    f[(13, 15)] = -0.5 * gy * dt;
    f[(14, 12)] = 0.5 * gy * dt;
    f[(14, 13)] = -0.5 * gz * dt;
    f[(14, 15)] = 0.5 * gx * dt;
    f[(15, 12)] = 0.5 * gz * dt;
    f[(15, 13)] = 0.5 * gy * dt;
    f[(15, 14)] = -0.5 * gx * dt;

    let derivs = [
        [-0.5 * q_x, -0.5 * q_y, -0.5 * q_z], // d_qw / d_omega
        [0.5 * q_w, -0.5 * q_z, 0.5 * q_y],   // d_qx / d_omega
        [0.5 * q_z, 0.5 * q_w, -0.5 * q_x],   // d_qy / d_omega
        [-0.5 * q_y, 0.5 * q_x, 0.5 * q_w],   // d_qz / d_omega
    ];

    for i in 0..4 {
        for j in 0..3 {
            let val = derivs[i][j] * dt;
            f[(12 + i, 19 + j)] = val; // Bias
            f[(12 + i, 9 + j)] = val; // Gyro
        }
    }
    f
}

pub fn measurement_function(
    state: &SVector<f64, 23>,
    calibration_active: bool,
) -> SVector<f64, 10> {
    let ax_expected = state[6] + state[16];
    let ay_expected = state[7] + state[17];
    let az_expected = state[8] + state[18];

    let (g_roll_expected, g_pitch_expected, g_yaw_expected) = if calibration_active {
        //during calibration, only bias/ noise
        (state[9], state[10], state[11])
    } else {
        //during flight true rate of rotation -bias
        (
            state[9] + state[19],
            state[10] + state[20],
            state[11] + state[21],
        )
    };

    let baro_expected = -state[2] - state[22];

    SVector::<f64, 10>::from_column_slice(&[
        state[0],         // gps_lat / North
        state[1],         // gps_lon / East
        state[2],         // gps_alt / Down
        ax_expected,      // low_g_ax
        ay_expected,      // low_g_ay
        az_expected,      // low_g_az
        g_roll_expected,  // low_g_gx
        g_pitch_expected, // low_g_gy
        g_yaw_expected,   // low_g_gz
        baro_expected,    // baro_alt
    ])
}

pub fn measurement_jacobian(state: &SVector<f64, 23>) -> SMatrix<f64, 10, 23> {
    let mut h = SMatrix::<f64, 10, 23>::zeros();

    // GPS Position
    h[(0, 0)] = 1.0;
    h[(1, 1)] = 1.0;
    h[(2, 2)] = 1.0;

    h[(9, 2)] = 1.0;
    h[(9, 22)] = -1.0;

    h[(3, 6)] = 1.0;
    h[(4, 7)] = 1.0;
    h[(5, 8)] = 1.0;

    h[(3, 16)] = -1.0;
    h[(4, 17)] = -1.0;
    h[(5, 18)] = -1.0;

    h[(6, 9)] = 1.0;
    h[(7, 10)] = 1.0;
    h[(8, 11)] = 1.0;
    h[(6, 19)] = -1.0;
    h[(7, 20)] = -1.0;
    h[(8, 21)] = -1.0;

    let accel_norm = (state.fixed_rows::<3>(6)).norm();
    if (accel_norm - 9.81).abs() < 1e-2 {
        let q = [state[12], state[13], state[14], state[15]]; // x, y, z, w
        let d_r_dq = compute_d_rotation_d_quaternion(&q);
        let g_ned = SVector::<f64, 3>::new(0.0, 0.0, 9.8);

        for j in 0..4 {
            let dr_dq_j = d_r_dq[j];
            let dg_body_dqj = dr_dq_j.transpose() * g_ned;

            for i in 0..3 {
                h[(3 + i, 12 + j)] = -dg_body_dqj[i];
            }
        }
    }
    h
}
