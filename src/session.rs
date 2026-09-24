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
use std::time::Duration;

use crate::DataFrameHeader;
use crate::control::LivoxClient;
use crate::data::{DataPacket, DataPayload, DataStream, ImuSample, Point};
use crate::error::{LidarError, Result};
use crate::protocol::{CMD_PORT, DataType, HOST_CMD_PORT, HOST_DATA_PORT, HOST_IMU_PORT};

/// Options for opening a live MID360 session.
#[derive(Debug, Clone)]
pub struct LiveConfig {
    pub host_ip: Ipv4Addr,
    pub lidar_ip: Ipv4Addr,
    pub data_type: DataType,
    pub command_timeout: Duration,
    pub lidar_cmd_port: u16,
    pub cmd_bind: SocketAddr,
    pub data_bind: SocketAddr,
    pub imu_bind: SocketAddr,
    pub data_dst: SocketAddr,
    pub imu_dst: SocketAddr,
}

impl LiveConfig {
    /// Default host ports, Cartesian32 points, IMU enabled, 2 s command timeout.
    pub fn new(host_ip: Ipv4Addr, lidar_ip: Ipv4Addr) -> Self {
        Self {
            host_ip,
            lidar_ip,
            data_type: DataType::PointCloudCartesian32,
            command_timeout: Duration::from_secs(2),
            lidar_cmd_port: CMD_PORT,
            cmd_bind: SocketAddr::from((host_ip, HOST_CMD_PORT)),
            data_bind: SocketAddr::from((host_ip, HOST_DATA_PORT)),
            imu_bind: SocketAddr::from((host_ip, HOST_IMU_PORT)),
            data_dst: SocketAddr::from((host_ip, HOST_DATA_PORT)),
            imu_dst: SocketAddr::from((host_ip, HOST_IMU_PORT)),
        }
    }
}

/// A parsed live sample: a point-cloud packet or an IMU packet.
#[derive(Debug, Clone, PartialEq)]
pub enum Sample {
    Points {
        header: DataFrameHeader,
        points: Vec<Point>,
    },
    Imu {
        header: DataFrameHeader,
        sample: ImuSample,
    },
}

impl TryFrom<DataPacket> for Sample {
    type Error = LidarError;

    fn try_from(packet: DataPacket) -> Result<Self> {
        match packet.payload {
            DataPayload::Points(points) => Ok(Sample::Points {
                header: packet.header,
                points,
            }),
            DataPayload::Imu(sample) => Ok(Sample::Imu {
                header: packet.header,
                sample,
            }),
        }
    }
}

/// Live MID360 session: command socket plus point-cloud and IMU receivers.
pub struct LiveReader {
    client: LivoxClient,
    stream: DataStream,
}

impl LiveReader {
    /// Bind default host ports and start Cartesian32 + IMU sampling.
    pub async fn connect(host_ip: Ipv4Addr, lidar_ip: Ipv4Addr) -> Result<Self> {
        Self::connect_with(LiveConfig::new(host_ip, lidar_ip)).await
    }

    /// Bind, configure stream destinations, and enter sampling.
    pub async fn connect_with(config: LiveConfig) -> Result<Self> {
        if config.host_ip.is_unspecified() {
            return Err(LidarError::InvalidHost);
        }

        let client = LivoxClient::new(config.cmd_bind)
            .await
            .map_err(|e| bind_err("command", config.cmd_bind, e))?;
        let stream = DataStream::new(config.data_bind, config.imu_bind)
            .await
            .map_err(|e| bind_err("data/imu", config.data_bind, e))?;

        let lidar_cmd = SocketAddr::from((config.lidar_ip, config.lidar_cmd_port));
        client
            .start_streaming(
                lidar_cmd,
                config.data_dst,
                config.imu_dst,
                config.data_type,
                config.command_timeout,
            )
            .await?;

        Ok(Self { client, stream })
    }

    pub fn client(&self) -> &LivoxClient {
        &self.client
    }

    pub fn stream(&self) -> &DataStream {
        &self.stream
    }

    /// Wait until the next point-cloud or IMU packet arrives.
    pub async fn recv(&self) -> Result<Sample> {
        tokio::select! {
            pkt = self.stream.recv_point_cloud() => Sample::try_from(pkt?),
            pkt = self.stream.recv_imu() => Sample::try_from(pkt?),
        }
    }

    pub async fn next_point_cloud(&self, wait: Duration) -> Result<DataPacket> {
        self.stream.next_point_cloud(wait).await
    }

    pub async fn next_imu(&self, wait: Duration) -> Result<DataPacket> {
        self.stream.next_imu(wait).await
    }
}

fn bind_err(resource: &'static str, addr: SocketAddr, err: LidarError) -> LidarError {
    match err {
        LidarError::Io(source) => LidarError::Bind {
            resource,
            addr,
            source,
        },
        other => other,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[tokio::test]
    async fn connect_rejects_unspecified_host() {
        let err = match LiveReader::connect(Ipv4Addr::UNSPECIFIED, Ipv4Addr::LOCALHOST).await {
            Err(e) => e,
            Ok(_) => panic!("expected InvalidHost"),
        };
        assert!(matches!(err, LidarError::InvalidHost));
    }
}
