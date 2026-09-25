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

use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

/// Encode a point cloud into a PCA-octree token vector.
///
/// `cloud` is a sequence of `(x, y, z)` positions in metres. `rounds` is
/// 1 through 5. The result has `4 * 8**rounds` floats.
#[pyfunction]
pub fn encode(cloud: Vec<[f32; 3]>, rounds: u8) -> PyResult<Vec<f32>> {
    livox_rs::encode::encode(&cloud, rounds).map_err(|err| PyValueError::new_err(err.to_string()))
}
