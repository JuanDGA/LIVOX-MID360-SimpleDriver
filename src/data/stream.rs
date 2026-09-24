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

use tokio::net::UdpSocket;
use tokio::time::timeout;

use crate::data::packet::{DATA_HEADER_SIZE, DataPacket};
use crate::error::{LidarError, Result};
use crate::protocol::{HOST_DATA_PORT, HOST_IMU_PORT};

/// Async receiver for MID360 point-cloud and IMU data streams.
pub struct DataStream {
    data_socket: UdpSocket,
    imu_socket: UdpSocket,
}

impl DataStream {
    /// Bind to the default data and IMU host ports.
    pub async fn with_default_ports(bind_ip: Ipv4Addr) -> Result<Self> {
        Self::new(
            SocketAddr::from((bind_ip, HOST_DATA_PORT)),
            SocketAddr::from((bind_ip, HOST_IMU_PORT)),
        )
        .await
    }

    /// Bind to custom data and IMU ports.
    pub async fn new(data_bind: SocketAddr, imu_bind: SocketAddr) -> Result<Self> {
        Ok(Self {
            data_socket: UdpSocket::bind(data_bind).await?,
            imu_socket: UdpSocket::bind(imu_bind).await?,
        })
    }

    /// Wait for the next point-cloud packet with no timeout.
    pub async fn recv_point_cloud(&self) -> Result<DataPacket> {
        let mut buf = [0u8; 8192];
        let (len, _) = self.data_socket.recv_from(&mut buf).await?;
        match DataPacket::parse(&buf[..len]) {
            Ok(p) => Ok(p),
            Err(e) => {
                if std::env::var("LIDAR_DEBUG").is_ok() {
                    diagnose_packet("point_cloud", &buf[..len], &e);
                }
                Err(e)
            }
        }
    }

    /// Wait for the next IMU packet with no timeout.
    pub async fn recv_imu(&self) -> Result<DataPacket> {
        let mut buf = [0u8; 1024];
        let (len, _) = self.imu_socket.recv_from(&mut buf).await?;
        DataPacket::parse(&buf[..len])
    }

    /// Wait for the next point-cloud packet.
    pub async fn next_point_cloud(&self, wait: Duration) -> Result<DataPacket> {
        match timeout(wait, self.recv_point_cloud()).await {
            Ok(r) => r,
            Err(_) => Err(LidarError::NoResponse {
                addr: self.data_socket.local_addr()?,
            }),
        }
    }

    /// Wait for the next IMU packet.
    pub async fn next_imu(&self, wait: Duration) -> Result<DataPacket> {
        match timeout(wait, self.recv_imu()).await {
            Ok(r) => r,
            Err(_) => Err(LidarError::NoResponse {
                addr: self.imu_socket.local_addr()?,
            }),
        }
    }
}

/// Print a diagnostic dump of a received data frame when LIDAR_DEBUG is set.
fn diagnose_packet(label: &str, buf: &[u8], err: &LidarError) {
    use crate::protocol::crc::crc32;

    eprintln!("--- LIDAR_DEBUG [{label}] parse error: {err} ---");
    if buf.len() < DATA_HEADER_SIZE {
        eprintln!("buffer too short: {} bytes", buf.len());
        return;
    }
    let length = u16::from_le_bytes([buf[1], buf[2]]) as usize;
    let dot_num = u16::from_le_bytes([buf[5], buf[6]]);
    let udp_cnt = u16::from_le_bytes([buf[7], buf[8]]);
    let stored = u32::from_le_bytes([buf[24], buf[25], buf[26], buf[27]]);
    let timestamp = u64::from_le_bytes([
        buf[28], buf[29], buf[30], buf[31], buf[32], buf[33], buf[34], buf[35],
    ]);

    eprintln!(
        "recv_len={} declared_length={} data_type={} dot_num={} udp_cnt={} frame_cnt={} time_type={} ts={}",
        buf.len(),
        length,
        buf[10],
        dot_num,
        udp_cnt,
        buf[9],
        buf[11],
        timestamp
    );
    eprintln!("stored crc32 = 0x{stored:08X}");

    let end = length.min(buf.len());
    let candidates: &[(&str, &[u8])] = &[
        ("[28..length] timestamp+data", &buf[28..end]),
        ("[36..length] data only", &buf[36..end]),
    ];
    for (name, slice) in candidates {
        let c = crc32(slice);
        eprintln!(
            "crc {name:32} = 0x{c:08X} {}",
            if c == stored { "<-- MATCH" } else { "" }
        );
    }

    let mut combined = Vec::with_capacity(end - 4);
    combined.extend_from_slice(&buf[0..24]);
    combined.extend_from_slice(&buf[28..end]);
    let c = crc32(&combined);
    eprintln!(
        "crc [0..24]+[28..length] header+ts+data = 0x{c:08X} {}",
        if c == stored { "<-- MATCH" } else { "" }
    );

    let c2 = crc32(&buf[0..end]);
    eprintln!(
        "crc [0..length] whole incl crc field   = 0x{c2:08X} {}",
        if c2 == stored { "<-- MATCH" } else { "" }
    );

    eprintln!(
        "first 48 bytes: {}",
        buf[..48.min(buf.len())]
            .iter()
            .map(|b| format!("{b:02X}"))
            .collect::<Vec<_>>()
            .join(" ")
    );
    eprintln!("--- end dump ---");
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::data::packet::{DataFrameHeader, DataPayload};
    use crate::data::points::{Cartesian32Point, Tag};
    use crate::protocol::{DataType, TimestampType};

    #[tokio::test]
    async fn receive_point_cloud_packet() {
        let stream = DataStream::new(
            SocketAddr::from((Ipv4Addr::LOCALHOST, 0)),
            SocketAddr::from((Ipv4Addr::LOCALHOST, 0)),
        )
        .await
        .unwrap();
        let data_addr = stream.data_socket.local_addr().unwrap();

        let header = DataFrameHeader {
            version: 0,
            length: 0,
            time_interval: 10,
            dot_num: 1,
            udp_cnt: 0,
            frame_cnt: 0,
            data_type: DataType::PointCloudCartesian32,
            time_type: TimestampType::None,
            timestamp: 12345,
        };
        let point = Cartesian32Point {
            x_mm: 100,
            y_mm: 200,
            z_mm: 300,
            reflectivity: 128,
            tag: Tag(0),
        };
        let payload = DataPayload::Points(vec![crate::data::points::Point::Cartesian32(point)]);

        tokio::spawn(async move {
            let mock = UdpSocket::bind(SocketAddr::from((Ipv4Addr::LOCALHOST, 0)))
                .await
                .unwrap();
            let mut buf = vec![0u8; 1024];
            DataPacket::build(&header, &payload, &mut buf).unwrap();
            let total = DATA_HEADER_SIZE + Cartesian32Point::SIZE;
            mock.send_to(&buf[..total], data_addr).await.unwrap();
        });

        let packet = stream
            .next_point_cloud(Duration::from_secs(1))
            .await
            .unwrap();
        assert_eq!(packet.header.timestamp, 12345);
        assert_eq!(packet.header.dot_num, 1);
        match packet.payload {
            DataPayload::Points(points) => {
                assert_eq!(points.len(), 1);
                assert_eq!(points[0].coords_m(), (0.1, 0.2, 0.3));
            }
            _ => panic!("expected points"),
        }
    }
}
