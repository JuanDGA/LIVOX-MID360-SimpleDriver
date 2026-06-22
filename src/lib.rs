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

pub mod client;
pub mod command;
pub mod crc;
pub mod error;
pub mod packet;
pub mod points;
pub mod protocol;
pub mod recorder;
#[cfg(feature = "viewer")]
pub mod imu;

pub use client::{DataStream, DiscoveredDevice, LivoxClient};
pub use command::CommandFrame;
pub use error::{LidarError, Result};
pub use packet::{DataFrameHeader, DataPacket, DataPayload};
pub use points::{Cartesian16Point, Cartesian32Point, ImuSample, Point, SphericalPoint, Tag};
pub use recorder::CsvRecorder;
pub use protocol::{
    CmdId, CmdType, DataType, LidarState, ParameterKey, ReturnCode, SenderType, TimestampType,
    CMD_PORT, DATA_PORT, DISCOVERY_PORT, IMU_PORT, LOG_PORT, PUSH_PORT,
};
