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

use glam::Vec3;
use livox_rs::cloud::Cloud;
use livox_rs::imu::{AttitudeEstimator, OrientationHistory};
use pyo3::prelude::*;

use crate::types::PyPoint;

/// Mahony attitude estimate. `update` takes gyro in rad/s, acceleration in g,
/// and a timestamp in nanoseconds.
#[pyclass(name = "AttitudeEstimator", module = "livox_mid360")]
pub struct PyAttitudeEstimator {
    inner: AttitudeEstimator,
}

#[pymethods]
impl PyAttitudeEstimator {
    #[new]
    fn new() -> Self {
        Self {
            inner: AttitudeEstimator::new(),
        }
    }

    fn update(&mut self, gyro: (f32, f32, f32), acc: (f32, f32, f32), ts: u64) {
        self.inner.update(
            Vec3::new(gyro.0, gyro.1, gyro.2),
            Vec3::new(acc.0, acc.1, acc.2),
            ts,
        );
    }

    fn __repr__(&self) -> &'static str {
        "AttitudeEstimator()"
    }
}

/// Orientations indexed by timestamp. An empty history reports identity.
#[pyclass(name = "OrientationHistory", module = "livox_mid360")]
pub struct PyOrientationHistory {
    inner: OrientationHistory,
}

#[pymethods]
impl PyOrientationHistory {
    #[new]
    fn new(max_len: usize) -> Self {
        Self {
            inner: OrientationHistory::new(max_len),
        }
    }

    /// Store the estimator's current orientation at `ts` (nanoseconds).
    fn push(&mut self, ts: u64, estimator: &PyAttitudeEstimator) {
        self.inner.push(ts, estimator.inner.q);
    }

    fn __repr__(&self) -> &'static str {
        "OrientationHistory()"
    }
}

/// Rolling point buffer in the same frame `lidar_encoder` writes.
///
/// Each point is rotated by the orientation at its packet timestamp, then
/// stored as `(x, z, -y)`. Points older than `max_age_ns` are dropped.
#[pyclass(name = "Cloud", module = "livox_mid360")]
pub struct PyCloud {
    inner: Cloud,
}

#[pymethods]
impl PyCloud {
    #[new]
    fn new() -> Self {
        Self {
            inner: Cloud::new(),
        }
    }

    /// Add one packet. `ts` and `max_age_ns` are nanoseconds.
    fn add(
        &mut self,
        points: Vec<PyPoint>,
        ts: u64,
        max_age_ns: u64,
        history: &PyOrientationHistory,
    ) {
        let pts: Vec<_> = points.into_iter().map(PyPoint::to_point).collect();
        let q = history.inner.at(ts);
        self.inner.add(&pts, ts, max_age_ns, q);
    }

    fn __len__(&self) -> usize {
        self.inner.len()
    }

    /// Buffered positions in metres, without timestamps.
    fn positions(&self) -> Vec<[f32; 3]> {
        self.inner.positions()
    }

    fn __repr__(&self) -> String {
        format!("Cloud(len={})", self.inner.len())
    }
}
