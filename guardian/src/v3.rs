//! 3D points and vectors (m): the head frame (x right, y up, looking along -z) and the standing universe.
use std::ops::{Add, Mul, Sub};

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
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
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
