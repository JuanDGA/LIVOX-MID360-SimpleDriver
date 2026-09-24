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

mod client;
mod error;
mod session;
mod types;

use pyo3::prelude::*;

use client::{PyDiscoveredDevice, PyLivoxClient};
use error::{init_runtime, LidarError};
use session::PyLiveReader;
use types::{
    PyDataFrameHeader, PyDataPacket, PyDataType, PyImuSample, PyPoint, PySample, PyTag,
    PyTimestampType,
};

#[pymodule]
fn livox_mid360(m: &Bound<'_, PyModule>) -> PyResult<()> {
    init_runtime();
    m.add("LidarError", m.py().get_type::<LidarError>())?;
    m.add_class::<PyLiveReader>()?;
    m.add_class::<PyLivoxClient>()?;
    m.add_class::<PyDiscoveredDevice>()?;
    m.add_class::<PySample>()?;
    m.add_class::<PyDataPacket>()?;
    m.add_class::<PyDataFrameHeader>()?;
    m.add_class::<PyPoint>()?;
    m.add_class::<PyTag>()?;
    m.add_class::<PyImuSample>()?;
    m.add_class::<PyDataType>()?;
    m.add_class::<PyTimestampType>()?;

    m.add("DISCOVERY_PORT", livox_rs::DISCOVERY_PORT)?;
    m.add("CMD_PORT", livox_rs::CMD_PORT)?;
    m.add("PUSH_PORT", livox_rs::PUSH_PORT)?;
    m.add("DATA_PORT", livox_rs::DATA_PORT)?;
    m.add("IMU_PORT", livox_rs::IMU_PORT)?;
    m.add("LOG_PORT", livox_rs::LOG_PORT)?;
    m.add("HOST_CMD_PORT", livox_rs::HOST_CMD_PORT)?;
    m.add("HOST_PUSH_PORT", livox_rs::HOST_PUSH_PORT)?;
    m.add("HOST_DATA_PORT", livox_rs::HOST_DATA_PORT)?;
    m.add("HOST_IMU_PORT", livox_rs::HOST_IMU_PORT)?;
    m.add("HOST_LOG_PORT", livox_rs::HOST_LOG_PORT)?;
    Ok(())
}
