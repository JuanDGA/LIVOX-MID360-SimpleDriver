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

use std::net::{Ipv4Addr, SocketAddr};
use std::sync::Arc;

use livox_rs::{DiscoveredDevice, LivoxClient, DISCOVERY_PORT, HOST_CMD_PORT};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3_async_runtimes::tokio::future_into_py;

use crate::error::{duration_secs, map_err, parse_ipv4};

#[pyclass(frozen, name = "DiscoveredDevice")]
#[derive(Clone)]
pub struct PyDiscoveredDevice {
    inner: DiscoveredDevice,
}

#[pymethods]
impl PyDiscoveredDevice {
    #[getter]
    fn dev_type(&self) -> u8 {
        self.inner.dev_type
    }

    #[getter]
    fn serial_number(&self) -> String {
        String::from_utf8_lossy(&self.inner.serial_number)
            .trim_end_matches('\0')
            .to_string()
    }

    #[getter]
    fn lidar_cmd_addr(&self) -> String {
        self.inner.lidar_cmd_addr.to_string()
    }

    fn __repr__(&self) -> String {
        format!(
            "DiscoveredDevice(dev_type={}, serial_number='{}', lidar_cmd_addr='{}')",
            self.dev_type(),
            self.serial_number(),
            self.lidar_cmd_addr()
        )
    }
}

/// Async discovery and control client. Methods return asyncio awaitables.
#[pyclass(name = "LivoxClient", module = "livox_mid360")]
pub struct PyLivoxClient {
    inner: Arc<LivoxClient>,
}

#[pymethods]
impl PyLivoxClient {
    /// Bind the command socket. Default host command port is 56101.
    #[staticmethod]
    #[pyo3(signature = (bind_ip, port=None))]
    fn bind<'py>(
        py: Python<'py>,
        bind_ip: &str,
        port: Option<u16>,
    ) -> PyResult<Bound<'py, PyAny>> {
        let bind_ip = parse_ipv4(bind_ip, "bind_ip")?;
        let addr = SocketAddr::from((bind_ip, port.unwrap_or(HOST_CMD_PORT)));
        future_into_py(py, async move {
            let inner = LivoxClient::new(addr).await.map_err(map_err)?;
            Ok(PyLivoxClient {
                inner: Arc::new(inner),
            })
        })
    }

    /// Broadcast a discovery request and collect responding devices.
    #[pyo3(signature = (wait=1.0, broadcast=None))]
    fn discover<'py>(
        &self,
        py: Python<'py>,
        wait: f64,
        broadcast: Option<&str>,
    ) -> PyResult<Bound<'py, PyAny>> {
        let wait = duration_secs(wait, "wait")?;
        let broadcast = match broadcast {
            Some(s) => s.parse::<SocketAddr>().map_err(|_| {
                PyValueError::new_err(format!("invalid broadcast address: {s}"))
            })?,
            None => SocketAddr::from((Ipv4Addr::BROADCAST, DISCOVERY_PORT)),
        };
        let inner = Arc::clone(&self.inner);
        future_into_py(py, async move {
            let devices = inner.discover(broadcast, wait).await.map_err(map_err)?;
            Ok(devices
                .into_iter()
                .map(|device| PyDiscoveredDevice { inner: device })
                .collect::<Vec<_>>())
        })
    }

    fn __repr__(&self) -> &'static str {
        "LivoxClient()"
    }
}
