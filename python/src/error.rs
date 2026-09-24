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

use std::net::Ipv4Addr;
use std::time::Duration;

use pyo3::create_exception;
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;

create_exception!(livox_mid360, LidarError, pyo3::exceptions::PyException);

pub fn map_err(err: livox_rs::LidarError) -> PyErr {
    LidarError::new_err(err.to_string())
}

pub fn parse_ipv4(value: &str, name: &str) -> PyResult<Ipv4Addr> {
    value
        .parse()
        .map_err(|_| PyValueError::new_err(format!("invalid {name}: {value}")))
}

pub fn duration_secs(secs: f64, name: &str) -> PyResult<Duration> {
    if !secs.is_finite() || secs < 0.0 {
        return Err(PyValueError::new_err(format!("{name} must be a finite value >= 0")));
    }
    Ok(Duration::from_secs_f64(secs))
}

pub fn init_runtime() {
    static INIT: std::sync::Once = std::sync::Once::new();
    INIT.call_once(|| {
        let mut builder = tokio::runtime::Builder::new_multi_thread();
        builder.enable_all();
        builder.thread_name("livox-mid360");
        pyo3_async_runtimes::tokio::init(builder);
    });
}
