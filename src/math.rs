use std::f64::consts::PI;

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn dot(self, rhs: Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y
    }

    pub fn norm2(self) -> f64 {
        self.dot(self)
    }

    pub fn norm(self) -> f64 {
        self.norm2().sqrt()
    }

    pub fn clamp_len(self, max_len: f64) -> Self {
        let len = self.norm();
        if len > max_len && len > 0.0 {
            self * (max_len / len)
        } else {
            self
        }
    }
}

impl std::ops::Add for Vec2 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl std::ops::Sub for Vec2 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl std::ops::Mul<f64> for Vec2 {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self::Output {
        Self::new(self.x * rhs, self.y * rhs)
    }
}

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Pose2 {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl Pose2 {
    pub const fn new(x: f64, y: f64, yaw: f64) -> Self {
        Self { x, y, yaw }
    }

    pub fn transform_point(&self, local: Vec2) -> Vec2 {
        let c = self.yaw.cos();
        let s = self.yaw.sin();
        Vec2::new(
            self.x + local.x * c - local.y * s,
            self.y + local.x * s + local.y * c,
        )
    }
}

pub fn clamp(v: f64, min: f64, max: f64) -> f64 {
    if v < min {
        min
    } else if v > max {
        max
    } else {
        v
    }
}

pub fn clamp_unit(v: f64) -> f64 {
    clamp(v, -1.0, 1.0)
}

pub fn clamp01(v: f64) -> f64 {
    clamp(v, 0.0, 1.0)
}

pub fn wrap_angle(mut a: f64) -> f64 {
    while a > PI {
        a -= 2.0 * PI;
    }
    while a < -PI {
        a += 2.0 * PI;
    }
    a
}

pub fn distance_point_segment(p: Vec2, a: Vec2, b: Vec2) -> f64 {
    let ab = b - a;
    let denom = ab.norm2();
    if denom <= f64::EPSILON {
        return (p - a).norm();
    }
    let t = clamp((p - a).dot(ab) / denom, 0.0, 1.0);
    let closest = a + ab * t;
    (p - closest).norm()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn point_segment_distance_on_line_is_zero() {
        let p = Vec2::new(0.5, 0.0);
        let a = Vec2::new(0.0, 0.0);
        let b = Vec2::new(1.0, 0.0);
        assert!(distance_point_segment(p, a, b) < 1e-12);
    }

    #[test]
    fn point_segment_distance_clamps_endpoints() {
        let p = Vec2::new(2.0, 0.0);
        let a = Vec2::new(0.0, 0.0);
        let b = Vec2::new(1.0, 0.0);
        assert!((distance_point_segment(p, a, b) - 1.0).abs() < 1e-12);
    }
}
