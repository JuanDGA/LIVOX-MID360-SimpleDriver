// Copyright 2026 Juan David Guevara Arévalo
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.

//! Rolling point-cloud buffer shared by the viewer and the headless encoder.
//!
//! Points are stored in viewer-world coordinates `(x, z, -y)` (see
//! [`Cloud::add`]) together with their packet timestamp, and expired once
//! they fall outside the retention window.

use glam::{Quat, Vec3};

use crate::points::Point;

/// Hard cap on buffered points so a runaway stream never blows up memory.
const MAX_POINTS: usize = 600_000;

/// Accumulated point cloud with the latest LiDAR timestamp used for expiry.
pub struct Cloud {
    /// (world metres, packet timestamp ns). World = (x, z, -y) after attitude
    /// rotation, so the LiDAR Z (height) becomes the vertical viewer axis.
    points: Vec<([f32; 3], u64)>,
    latest_ts: u64,
}

impl Cloud {
    pub fn new() -> Self {
        Self {
            points: Vec::with_capacity(MAX_POINTS),
            latest_ts: 0,
        }
    }

    /// Add a batch of points from one packet, rotated by the attitude `q`
    /// (body -> world) estimated at the packet timestamp. Points older than
    /// `max_age_ns` are dropped afterwards; the `MAX_POINTS` safety cap
    /// still bounds memory.
    pub fn add(&mut self, pts: &[Point], ts: u64, max_age_ns: u64, q: Quat) {
        if ts > self.latest_ts {
            self.latest_ts = ts;
        }
        for p in pts {
            let (x, y, z) = p.coords_m();
            // Stabilize: body -> gravity-aligned world frame.
            let pw = q * Vec3::new(x, y, z);
            // Remap to viewer world (x, z, -y): Z up, +LiDAR-Y corrected.
            self.points.push(([pw.x, pw.z, -pw.y], ts));
        }
        self.expire(max_age_ns);
        // Safety cap so a runaway stream never blows up memory.
        if self.points.len() > MAX_POINTS {
            let drop = self.points.len() - MAX_POINTS;
            self.points.drain(..drop);
        }
    }

    fn expire(&mut self, max_age_ns: u64) {
        let cutoff = self.latest_ts.saturating_sub(max_age_ns);
        // Points are appended in timestamp order, so the old ones are at the
        // front; drop them in bulk.
        let mut keep = 0;
        while keep < self.points.len() && self.points[keep].1 < cutoff {
            keep += 1;
        }
        if keep > 0 {
            self.points.drain(..keep);
        }
    }

    pub fn clear(&mut self) {
        self.points.clear();
    }

    /// Number of buffered points.
    pub fn len(&self) -> usize {
        self.points.len()
    }

    /// Whether the buffer is empty.
    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Buffered points as (viewer-world position, packet timestamp ns).
    pub fn points(&self) -> &[([f32; 3], u64)] {
        &self.points
    }

    /// Buffered positions without timestamps (render/encode input).
    pub fn positions(&self) -> Vec<[f32; 3]> {
        self.points.iter().map(|(p, _)| *p).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::points::{Cartesian32Point, Tag};

    fn pt(z_mm: i32) -> Point {
        Point::Cartesian32(Cartesian32Point {
            x_mm: 0,
            y_mm: 0,
            z_mm,
            reflectivity: 0,
            tag: Tag(0),
        })
    }

    #[test]
    fn add_expires_points_older_than_window() {
        let mut c = Cloud::new();
        c.add(&[pt(1000)], 0, 500_000_000, Quat::IDENTITY);
        c.add(&[pt(2000)], 1_000_000_000, 500_000_000, Quat::IDENTITY);
        assert_eq!(c.len(), 1, "the 1 s-old point exceeds the 500 ms window");
        // Identity attitude: stored viewer-world = (x, z, -y).
        assert_eq!(c.points()[0].0, [0.0, 2.0, 0.0]);
    }

    #[test]
    fn positions_drop_timestamps() {
        let mut c = Cloud::new();
        c.add(&[pt(1000)], 0, u64::MAX, Quat::IDENTITY);
        assert_eq!(c.positions(), vec![[0.0, 1.0, 0.0]]);
    }
}
