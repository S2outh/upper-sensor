use core::{
    array::repeat,
    convert::Infallible,
    ops::{AddAssign, Div},
};

use south_common::types::upper_sensor::AccelRaw;
use nalgebra as na;

#[derive(Clone)]
pub struct AccelOvsWrapper(pub [[i64; 3]; 2]);
impl AddAssign<AccelOvsWrapper> for AccelOvsWrapper {
    fn add_assign(&mut self, rhs: AccelOvsWrapper) {
        for t in 0..2 {
            for i in 0..3 {
                self.0[t][i] += rhs.0[t][i];
            }
        }
    }
}
impl Div<AccelOvsWrapper> for AccelOvsWrapper {
    type Output = AccelRaw;
    fn div(self, rhs: AccelOvsWrapper) -> Self::Output {
        let mut out = ([0i16; 3], [0i16; 3]);
        for i in 0..3 {
            out.0[i] = (self.0[0][i] / rhs.0[0][i]) as i16;
        }
        for i in 0..3 {
            out.1[i] = (self.0[1][i] / rhs.0[1][i]) as i16;
        }
        out.into()
    }
}
impl TryFrom<usize> for AccelOvsWrapper {
    type Error = Infallible;
    fn try_from(value: usize) -> Result<Self, Self::Error> {
        Ok(Self(repeat(repeat(value as i64))))
    }
}
impl From<AccelRaw> for AccelOvsWrapper {
    fn from(t: AccelRaw) -> Self {
        Self([
            [
                t.accel_low_range.x.into(),
                t.accel_low_range.y.into(),
                t.accel_low_range.z.into(),
            ],
            [
                t.accel_full_range.x.into(),
                t.accel_full_range.y.into(),
                t.accel_full_range.z.into(),
            ],
        ])
    }
}

#[derive(Clone)]
pub struct GyroOvsWrapper(pub [i64; 3]);
impl AddAssign<GyroOvsWrapper> for GyroOvsWrapper {
    fn add_assign(&mut self, rhs: GyroOvsWrapper) {
        for i in 0..3 {
            self.0[i] += rhs.0[i];
        }
    }
}
impl Div<GyroOvsWrapper> for GyroOvsWrapper {
    type Output = na::Vector3<i16>;
    fn div(self, rhs: GyroOvsWrapper) -> Self::Output {
        let mut out = [0i16; 3];
        for i in 0..3 {
            out[i] = (self.0[i] / rhs.0[i]) as i16;
        }
        out.into()
    }
}
impl TryFrom<usize> for GyroOvsWrapper {
    type Error = Infallible;
    fn try_from(value: usize) -> Result<Self, Self::Error> {
        Ok(Self(repeat(value as i64)))
    }
}
impl From<na::Vector3<i16>> for GyroOvsWrapper {
    fn from(t: na::Vector3<i16>) -> Self {
        Self([t.x.into(), t.y.into(), t.z.into()])
    }
}

#[derive(Clone)]
pub struct MagOvsWrapper(pub [i64; 3]);
impl AddAssign<MagOvsWrapper> for MagOvsWrapper {
    fn add_assign(&mut self, rhs: MagOvsWrapper) {
        for i in 0..3 {
            self.0[i] += rhs.0[i];
        }
    }
}
impl Div<MagOvsWrapper> for MagOvsWrapper {
    type Output = na::Vector3<i32>;
    fn div(self, rhs: MagOvsWrapper) -> Self::Output {
        let mut out = [0i32; 3];
        for i in 0..3 {
            out[i] = (self.0[i] / rhs.0[i]) as i32;
        }
        out.into()
    }
}
impl TryFrom<usize> for MagOvsWrapper {
    type Error = Infallible;
    fn try_from(value: usize) -> Result<Self, Self::Error> {
        Ok(Self(repeat(value as i64)))
    }
}
impl From<na::Vector3<i32>> for MagOvsWrapper {
    fn from(t: na::Vector3<i32>) -> Self {
        Self([t.x.into(), t.y.into(), t.z.into()])
    }
}
