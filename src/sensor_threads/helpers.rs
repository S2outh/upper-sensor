use core::{
    array::repeat,
    convert::Infallible,
    ops::{AddAssign, Div},
};

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
    type Output = ([i16; 3], [i16; 3]);
    fn div(self, rhs: AccelOvsWrapper) -> Self::Output {
        let mut out = ([0i16; 3], [0i16; 3]);
        for i in 0..3 {
            out.0[i] = (self.0[0][i] / rhs.0[0][i]) as i16;
        }
        for i in 0..3 {
            out.1[i] = (self.0[1][i] / rhs.0[1][i]) as i16;
        }
        out
    }
}
impl TryFrom<usize> for AccelOvsWrapper {
    type Error = Infallible;
    fn try_from(value: usize) -> Result<Self, Self::Error> {
        Ok(Self(repeat(repeat(value as i64))))
    }
}
impl From<([i16; 3], [i16; 3])> for AccelOvsWrapper {
    fn from(t: ([i16; 3], [i16; 3])) -> Self {
        Self::from([t.0, t.1])
    }
}
impl From<[[i16; 3]; 2]> for AccelOvsWrapper {
    fn from(value: [[i16; 3]; 2]) -> Self {
        Self(value.map(|inner| inner.map(|v| v as i64)))
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
    type Output = [i16; 3];
    fn div(self, rhs: GyroOvsWrapper) -> Self::Output {
        let mut out = [0i16; 3];
        for i in 0..3 {
            out[i] = (self.0[i] / rhs.0[i]) as i16;
        }
        out
    }
}
impl TryFrom<usize> for GyroOvsWrapper {
    type Error = Infallible;
    fn try_from(value: usize) -> Result<Self, Self::Error> {
        Ok(Self(repeat(value as i64)))
    }
}
impl From<[i16; 3]> for GyroOvsWrapper {
    fn from(value: [i16; 3]) -> Self {
        Self(value.map(|v| v as i64))
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
    type Output = [i32; 3];
    fn div(self, rhs: MagOvsWrapper) -> Self::Output {
        let mut out = [0i32; 3];
        for i in 0..3 {
            out[i] = (self.0[i] / rhs.0[i]) as i32;
        }
        out
    }
}
impl TryFrom<usize> for MagOvsWrapper {
    type Error = Infallible;
    fn try_from(value: usize) -> Result<Self, Self::Error> {
        Ok(Self(repeat(value as i64)))
    }
}
impl From<[i32; 3]> for MagOvsWrapper {
    fn from(value: [i32; 3]) -> Self {
        Self(value.map(|v| v as i64))
    }
}
