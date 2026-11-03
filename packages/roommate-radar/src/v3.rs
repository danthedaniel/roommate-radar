//! 3D points and vectors (m): the head frame (x right, y up, looking along -z) and the standing universe.
use std::ops::{Add, Mul, Sub};

use crate::openvr::HmdMatrix34_t;

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct V3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

pub const fn v3(x: f32, y: f32, z: f32) -> V3 {
    V3 { x, y, z }
}

impl V3 {
    pub fn len(self) -> f32 {
        self.dot(self).sqrt()
    }

    pub fn dot(self, b: V3) -> f32 {
        self.x * b.x + self.y * b.y + self.z * b.z
    }
}

impl Add for V3 {
    type Output = V3;
    fn add(self, b: V3) -> V3 {
        v3(self.x + b.x, self.y + b.y, self.z + b.z)
    }
}

impl Sub for V3 {
    type Output = V3;
    fn sub(self, b: V3) -> V3 {
        v3(self.x - b.x, self.y - b.y, self.z - b.z)
    }
}

impl Mul<f32> for V3 {
    type Output = V3;
    fn mul(self, s: f32) -> V3 {
        v3(self.x * s, self.y * s, self.z * s)
    }
}

/// The point p moved by the pose m.
pub fn apply(m: &HmdMatrix34_t, p: V3) -> V3 {
    let r = |i: usize| m.m[i][0] * p.x + m.m[i][1] * p.y + m.m[i][2] * p.z + m.m[i][3];
    v3(r(0), r(1), r(2))
}

/// The pose m's origin.
pub fn position(m: &HmdMatrix34_t) -> V3 {
    v3(m.m[0][3], m.m[1][3], m.m[2][3])
}
