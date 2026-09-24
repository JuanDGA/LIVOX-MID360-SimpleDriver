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

use std::net::SocketAddr;
use std::sync::Arc;

use livox_rs::{LiveConfig, LiveReader};
use pyo3::prelude::*;
use pyo3::Bound;
use pyo3_async_runtimes::tokio::future_into_py;

use crate::error::{duration_secs, map_err, parse_ipv4};
use crate::types::{parse_data_type, PyDataPacket, PySample};

/// Async live MID360 session. Methods return asyncio awaitables.
#[pyclass(name = "LiveReader", module = "livox_mid360")]
pub struct PyLiveReader {
    inner: Arc<LiveReader>,
}

#[pymethods]
impl PyLiveReader {
    /// Bind default host ports and start Cartesian32 + IMU sampling.
    #[staticmethod]
    fn connect<'py>(
        py: Python<'py>,
        host_ip: &str,
        lidar_ip: &str,
    ) -> PyResult<Bound<'py, PyAny>> {
        let host_ip = parse_ipv4(host_ip, "host_ip")?;
        let lidar_ip = parse_ipv4(lidar_ip, "lidar_ip")?;
        future_into_py(py, async move {
            let inner = LiveReader::connect(host_ip, lidar_ip)
                .await
                .map_err(map_err)?;
            Ok(PyLiveReader {
                inner: Arc::new(inner),
            })
        })
    }

    /// Bind, configure stream destinations, and enter sampling.
    #[staticmethod]
    #[pyo3(signature = (
        host_ip,
        lidar_ip,
        data_type=None,
        command_timeout=2.0,
        lidar_cmd_port=None,
        cmd_port=None,
        data_port=None,
        imu_port=None
    ))]
    fn connect_with<'py>(
        py: Python<'py>,
        host_ip: &str,
        lidar_ip: &str,
        data_type: Option<&Bound<'_, PyAny>>,
        command_timeout: f64,
        lidar_cmd_port: Option<u16>,
        cmd_port: Option<u16>,
        data_port: Option<u16>,
        imu_port: Option<u16>,
    ) -> PyResult<Bound<'py, PyAny>> {
        let host_ip = parse_ipv4(host_ip, "host_ip")?;
        let lidar_ip = parse_ipv4(lidar_ip, "lidar_ip")?;
        let mut config = LiveConfig::new(host_ip, lidar_ip);
        config.data_type = parse_data_type(data_type)?;
        config.command_timeout = duration_secs(command_timeout, "command_timeout")?;
        if let Some(port) = lidar_cmd_port {
            config.lidar_cmd_port = port;
        }
        if let Some(port) = cmd_port {
            config.cmd_bind = SocketAddr::from((host_ip, port));
        }
        if let Some(port) = data_port {
            config.data_bind = SocketAddr::from((host_ip, port));
            config.data_dst = SocketAddr::from((host_ip, port));
        }
        if let Some(port) = imu_port {
            config.imu_bind = SocketAddr::from((host_ip, port));
            config.imu_dst = SocketAddr::from((host_ip, port));
        }

        future_into_py(py, async move {
            let inner = LiveReader::connect_with(config).await.map_err(map_err)?;
            Ok(PyLiveReader {
                inner: Arc::new(inner),
            })
        })
    }

    /// Wait until the next point-cloud or IMU packet arrives.
    fn recv<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyAny>> {
        let inner = Arc::clone(&self.inner);
        future_into_py(py, async move {
            inner.recv().await.map(PySample::from).map_err(map_err)
        })
    }

    /// Wait for the next point-cloud packet.
    #[pyo3(signature = (wait=1.0))]
    fn next_point_cloud<'py>(&self, py: Python<'py>, wait: f64) -> PyResult<Bound<'py, PyAny>> {
        let wait = duration_secs(wait, "wait")?;
        let inner = Arc::clone(&self.inner);
        future_into_py(py, async move {
            inner
                .next_point_cloud(wait)
                .await
                .map(PyDataPacket::from)
                .map_err(map_err)
        })
    }

    /// Wait for the next IMU packet.
    #[pyo3(signature = (wait=1.0))]
    fn next_imu<'py>(&self, py: Python<'py>, wait: f64) -> PyResult<Bound<'py, PyAny>> {
        let wait = duration_secs(wait, "wait")?;
        let inner = Arc::clone(&self.inner);
        future_into_py(py, async move {
            inner
                .next_imu(wait)
                .await
                .map(PyDataPacket::from)
                .map_err(map_err)
        })
    }

    fn __repr__(&self) -> &'static str {
        "LiveReader()"
    }
}
