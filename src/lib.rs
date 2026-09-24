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

//! Livox MID360 communication and packet parsing.
//!
//! The crate speaks the sensor's UDP protocol, parses command and data frames,
//! and exposes [`LiveReader`] for a live session that yields point-cloud and
//! IMU samples. Optional features add a rolling point buffer (`cloud`), a
//! PCA octree tokenizer (`encode`), and IMU attitude estimation (`imu`).

pub mod control;
pub mod data;
pub mod error;
pub mod protocol;
pub mod session;

#[cfg(feature = "cloud")]
pub mod cloud;
#[cfg(feature = "encode")]
pub mod encode;
#[cfg(feature = "imu")]
pub mod imu;

pub use control::{DiscoveredDevice, LivoxClient};
pub use data::{
    Cartesian16Point, Cartesian32Point, DataFrameHeader, DataPacket, DataPayload, DataStream,
    ImuSample, Point, SphericalPoint, Tag,
};
pub use error::{LidarError, Result};
pub use protocol::command::CommandFrame;
pub use protocol::{
    CMD_PORT, CmdId, CmdType, DATA_PORT, DISCOVERY_PORT, DataType, HOST_CMD_PORT, HOST_DATA_PORT,
    HOST_IMU_PORT, HOST_LOG_PORT, HOST_PUSH_PORT, IMU_PORT, LOG_PORT, LidarState, PUSH_PORT,
    ParameterKey, ReturnCode, SenderType, TimestampType,
};
pub use session::{LiveConfig, LiveReader, Sample};
