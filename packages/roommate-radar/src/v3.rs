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

/// The point moved by the pose.
pub fn transform(pose: &HmdMatrix34_t, point: V3) -> V3 {
    let r = |i: usize| pose.m[i][0] * point.x + pose.m[i][1] * point.y + pose.m[i][2] * point.z + pose.m[i][3];
    v3(r(0), r(1), r(2))
}

/// The pose's origin.
pub fn position(pose: &HmdMatrix34_t) -> V3 {
    v3(pose.m[0][3], pose.m[1][3], pose.m[2][3])
}
