//! IMU-based attitude estimation for stabilizing the point-cloud view.
//!
//! Estimates the rotation from the LiDAR body frame to a gravity-aligned
//! world frame (yaw fixed at startup) using a Mahony complementary filter:
//! the gyroscope is integrated for short-term orientation, and the
//! accelerometer corrects roll/pitch drift by aligning the estimated up
//! direction to measured gravity. Yaw is unobservable without a magnetometer
//! and is therefore left relative to the LiDAR's orientation at startup.
//!
//! Limitation: only *orientation* is corrected. Translation cannot be
//! recovered from an IMU (double-integrating acceleration drifts rapidly),
//! so walking the LiDAR sideways will still shift the cloud.

use std::collections::VecDeque;

use glam::{Quat, Vec3};

/// Mahony complementary-filter attitude estimator.
pub struct AttitudeEstimator {
    /// Rotation from body frame to world frame.
    pub q: Quat,
    integral: Vec3,
    kp: f32,
    ki: f32,
    last_ts: Option<u64>,
}

impl AttitudeEstimator {
    pub fn new() -> Self {
        Self {
            q: Quat::IDENTITY,
            integral: Vec3::ZERO,
            kp: 1.0,
            ki: 0.005,
            last_ts: None,
        }
    }

    pub fn with_gains(kp: f32, ki: f32) -> Self {
        Self {
            kp,
            ki,
            ..Self::new()
        }
    }

    /// Feed one IMU sample. `gyro` is in rad/s, `acc` in g, `ts` in ns.
    pub fn update(&mut self, gyro: Vec3, acc: Vec3, ts: u64) {
        let dt = match self.last_ts {
            Some(prev) => (ts.saturating_sub(prev)) as f32 * 1e-9,
            None => {
                self.last_ts = Some(ts);
                return;
            }
        };
        self.last_ts = Some(ts);
        // Clamp to a sane range so a timestamp glitch can't blow up integration.
        let dt = dt.clamp(1e-4, 0.1);

        let mut gyro_corr = gyro;
        let acc_mag = acc.length();
        // Trust the accelerometer only when it is dominated by gravity
        // (i.e. the LiDAR is not being sharply accelerated).
        if acc_mag > 0.5 && acc_mag < 2.0 {
            let measured_up = acc / acc_mag;
            let predicted_up = self.q.conjugate() * Vec3::Z;
            let error = measured_up.cross(predicted_up);
            self.integral += error * dt;
            gyro_corr = gyro + self.kp * error + self.ki * self.integral;
        }

        let dq = Quat::from_scaled_axis(gyro_corr * dt);
        self.q = (self.q * dq).normalize();
    }
}

impl Default for AttitudeEstimator {
    fn default() -> Self {
        Self::new()
    }
}

/// Time-indexed buffer of orientations, used to look up the attitude at a
/// point-cloud packet's timestamp (which may fall between IMU samples).
pub struct OrientationHistory {
    samples: VecDeque<(u64, Quat)>,
    max_len: usize,
}

impl OrientationHistory {
    pub fn new(max_len: usize) -> Self {
        Self {
            samples: VecDeque::with_capacity(max_len),
            max_len,
        }
    }

    pub fn push(&mut self, ts: u64, q: Quat) {
        self.samples.push_back((ts, q));
        while self.samples.len() > self.max_len {
            self.samples.pop_front();
        }
    }

    /// Interpolate the orientation at `ts`, clamping to the buffer ends.
    pub fn at(&self, ts: u64) -> Quat {
        if self.samples.is_empty() {
            return Quat::IDENTITY;
        }
        let first = *self.samples.front().unwrap();
        let last = *self.samples.back().unwrap();
        if ts <= first.0 {
            return first.1;
        }
        if ts >= last.0 {
            return last.1;
        }
        for i in 0..self.samples.len() - 1 {
            let (t0, q0) = self.samples[i];
            let (t1, q1) = self.samples[i + 1];
            if ts >= t0 && ts <= t1 {
                let span = (t1 - t0) as f32;
                let s = if span > 0.0 { (ts - t0) as f32 / span } else { 0.0 };
                return q0.slerp(q1, s);
            }
        }
        last.1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stationary_stays_level() {
        let mut e = AttitudeEstimator::new();
        let mut ts = 0u64;
        for _ in 0..100 {
            ts += 5_000_000; // 5 ms -> 200 Hz
            e.update(Vec3::ZERO, Vec3::new(0.0, 0.0, 1.0), ts);
        }
        let up = e.q.conjugate() * Vec3::Z;
        assert!(
            (up - Vec3::Z).length() < 1e-3,
            "expected level, got up = {up:?}"
        );
    }

    #[test]
    fn gyro_yaw_accumulates() {
        let mut e = AttitudeEstimator::new();
        let mut ts = 0u64;
        let omega = 1.0; // rad/s about Z
        for _ in 0..200 {
            ts += 5_000_000;
            // Level gravity so the accel correction does not fight the yaw.
            e.update(Vec3::new(0.0, 0.0, omega), Vec3::new(0.0, 0.0, 1.0), ts);
        }
        // 200 * 5 ms * 1 rad/s = 1.0 rad.
        let rotated_x = e.q * Vec3::X;
        let expected = Vec3::new(1.0_f32.cos(), 1.0_f32.sin(), 0.0);
        assert!(
            (rotated_x - expected).length() < 1e-2,
            "got {rotated_x:?} expected {expected:?}"
        );
    }

    #[test]
    fn accel_levels_a_tilt() {
        // Body tilted +30 deg about X: the accelerometer's "up" (specific
        // force) in the body frame is (0, sin30, cos30).
        let mut e = AttitudeEstimator::with_gains(5.0, 0.0);
        let mut ts = 0u64;
        let measured_up = Vec3::new(0.0, 0.5, 0.866_025_4);
        for _ in 0..1000 {
            ts += 5_000_000;
            e.update(Vec3::ZERO, measured_up, ts);
        }
        let predicted = e.q.conjugate() * Vec3::Z;
        assert!(
            (predicted - measured_up).length() < 1e-2,
            "predicted up = {predicted:?}"
        );
    }

    #[test]
    fn history_interpolates() {
        let mut h = OrientationHistory::new(16);
        let q90 = Quat::from_rotation_z(std::f32::consts::FRAC_PI_2);
        h.push(100, Quat::IDENTITY);
        // 90 deg about Z at t = 1 s + 100 ns.
        h.push(1_000_000_100, q90);

        // Clamping: before the first sample returns the first orientation.
        assert_eq!(h.at(5), Quat::IDENTITY);
        // Clamping: after the last sample returns the last orientation.
        assert_eq!(h.at(2_000_000_000), q90);

        // Midpoint interpolation -> ~45 deg about Z (cos = sin = 1/sqrt(2)).
        let half = h.at(500_000_100);
        let v = half * Vec3::X;
        let r2 = std::f32::consts::FRAC_1_SQRT_2;
        assert!((v.x - r2).abs() < 1e-2 && (v.y - r2).abs() < 1e-2);
    }
}
