use std::collections::HashSet;
use std::net::{Ipv4Addr, SocketAddr};
use std::sync::atomic::{AtomicU32, Ordering};
use std::time::Duration;

use tokio::net::UdpSocket;
use tokio::time::timeout;

use crate::command::{
    build_param_config, build_param_inquire, parse_param_config_ack, parse_param_inquire_ack,
    CommandFrame, DiscoveryAck, host_ip_cfg_value,
};
use crate::error::{LidarError, Result};
use crate::packet::DataPacket;
use crate::protocol::{
    CmdId, DataType, ParameterKey, ReturnCode, HOST_CMD_PORT, HOST_DATA_PORT, HOST_IMU_PORT,
};

/// A LiDAR discovered through the broadcast discovery command.
#[derive(Debug, Clone)]
pub struct DiscoveredDevice {
    pub dev_type: u8,
    pub serial_number: [u8; 16],
    pub lidar_cmd_addr: SocketAddr,
}

/// Async UDP client for Livox MID360 control commands.
pub struct LivoxClient {
    cmd_socket: UdpSocket,
    seq: AtomicU32,
}

impl LivoxClient {
    /// Bind the client to a local address. The default command port is 56101.
    pub async fn new(bind: SocketAddr) -> Result<Self> {
        let socket = UdpSocket::bind(bind).await?;
        socket.set_broadcast(true)?;
        Ok(Self {
            cmd_socket: socket,
            seq: AtomicU32::new(1),
        })
    }

    /// Create a client bound to the default command port on `bind_ip`.
    pub async fn with_default_cmd_port(bind_ip: Ipv4Addr) -> Result<Self> {
        Self::new(SocketAddr::from((bind_ip, HOST_CMD_PORT))).await
    }

    fn next_seq(&self) -> u32 {
        self.seq.fetch_add(1, Ordering::Relaxed)
    }

    /// Broadcast a discovery request and collect all responding devices.
    pub async fn discover(
        &self,
        broadcast_addr: SocketAddr,
        wait: Duration,
    ) -> Result<Vec<DiscoveredDevice>> {
        let req = CommandFrame::new_req(0, CmdId::Discovery.as_u16(), Vec::new());
        let mut buf = vec![0u8; 1024];
        req.write(&mut buf)?;
        self.cmd_socket.send_to(&buf[..req.length as usize], broadcast_addr).await?;

        let local_addr = self.cmd_socket.local_addr()?;
        if let Some(directed) = directed_broadcast_for(local_addr.ip().to_string().parse().ok()) {
            let directed_addr = SocketAddr::from((directed, broadcast_addr.port()));
            if directed_addr != broadcast_addr {
                self.cmd_socket.send_to(&buf[..req.length as usize], directed_addr).await?;
            }
        }

        let mut devices = Vec::new();
        let mut seen = HashSet::new();
        let deadline = tokio::time::Instant::now() + wait;

        while tokio::time::Instant::now() < deadline {
            let remaining = deadline - tokio::time::Instant::now();
            let mut recv_buf = [0u8; 1024];
            match timeout(remaining, self.cmd_socket.recv_from(&mut recv_buf)).await {
                Ok(Ok((len, _))) => {
                    if let Ok(frame) = CommandFrame::parse(&recv_buf[..len])
                        && frame.cmd_id == CmdId::Discovery.as_u16()
                        && frame.cmd_type == crate::protocol::CmdType::Ack
                        && let Ok(ack) = DiscoveryAck::parse(&frame.data)
                    {
                        let addr = SocketAddr::from((ack.lidar_ip, ack.cmd_port));
                        if seen.insert(addr) {
                            devices.push(DiscoveredDevice {
                                dev_type: ack.dev_type,
                                serial_number: ack.serial_number,
                                lidar_cmd_addr: addr,
                            });
                        }
                    }
                }
                Ok(Err(e)) => return Err(e.into()),
                Err(_) => break,
            }
        }
        Ok(devices)
    }

    /// Send a command and wait for its matching ACK.
    pub async fn send_command(
        &self,
        addr: SocketAddr,
        cmd_id: u16,
        data: Vec<u8>,
        wait: Duration,
    ) -> Result<CommandFrame> {
        let req = CommandFrame::new_req(self.next_seq(), cmd_id, data);
        let mut send_buf = vec![0u8; 2048];
        req.write(&mut send_buf)?;
        self.cmd_socket.send_to(&send_buf[..req.length as usize], addr).await?;

        let deadline = tokio::time::Instant::now() + wait;
        loop {
            let remaining = deadline - tokio::time::Instant::now();
            if remaining.is_zero() {
                return Err(LidarError::NoResponse { addr });
            }
            let mut recv_buf = [0u8; 2048];
            let (len, from) = match timeout(remaining, self.cmd_socket.recv_from(&mut recv_buf)).await
            {
                Ok(Ok(v)) => v,
                Ok(Err(e)) => return Err(e.into()),
                Err(_) => return Err(LidarError::NoResponse { addr }),
            };

            // Only accept ACKs from the LiDAR we contacted.
            if from.ip() != addr.ip() {
                continue;
            }

            let frame = match CommandFrame::parse(&recv_buf[..len]) {
                Ok(f) => f,
                Err(_) => continue,
            };

            if frame.is_ack_for(&req) {
                return Ok(frame);
            }
        }
    }

    /// Query one or more parameters from the LiDAR.
    pub async fn query_parameter(
        &self,
        addr: SocketAddr,
        keys: &[ParameterKey],
        wait: Duration,
    ) -> Result<Vec<(u16, Vec<u8>)>> {
        let payload = build_param_inquire(keys);
        let ack = self
            .send_command(addr, CmdId::ParamInquire.as_u16(), payload, wait)
            .await?;
        let (ret, values) = parse_param_inquire_ack(&ack.data)?;
        if ret != ReturnCode::Success {
            return Err(LidarError::CommandFailed { code: ret });
        }
        Ok(values)
    }

    /// Set one or more parameters on the LiDAR.
    pub async fn set_parameter(
        &self,
        addr: SocketAddr,
        items: &[(ParameterKey, Vec<u8>)],
        wait: Duration,
    ) -> Result<()> {
        let payload = build_param_config(items);
        let ack = self
            .send_command(addr, CmdId::ParamConfig.as_u16(), payload, wait)
            .await?;
        let (ret, error_key) = parse_param_config_ack(&ack.data)?;
        if ret != ReturnCode::Success {
            return Err(LidarError::CommandFailed { code: ret });
        }
        let _ = error_key;
        Ok(())
    }

    /// Configure the LiDAR to send point-cloud and IMU data to the given destinations.
    pub async fn set_stream_destinations(
        &self,
        addr: SocketAddr,
        data_dst: SocketAddr,
        imu_dst: SocketAddr,
        wait: Duration,
    ) -> Result<()> {
        let data_value = host_ip_cfg_value(
            data_dst.ip().to_string().parse::<Ipv4Addr>().unwrap_or(Ipv4Addr::UNSPECIFIED),
            data_dst.port(),
            HOST_DATA_PORT,
        );
        let imu_value = host_ip_cfg_value(
            imu_dst.ip().to_string().parse::<Ipv4Addr>().unwrap_or(Ipv4Addr::UNSPECIFIED),
            imu_dst.port(),
            HOST_IMU_PORT,
        );
        self.set_parameter(
            addr,
            &[
                (ParameterKey::PointCloudHostIpCfg, data_value),
                (ParameterKey::ImuHostIpCfg, imu_value),
            ],
            wait,
        )
        .await
    }

    /// Set the LiDAR working mode (e.g. sampling, idle, ready).
    pub async fn set_work_mode(
        &self,
        addr: SocketAddr,
        state: crate::protocol::LidarState,
        wait: Duration,
    ) -> Result<()> {
        self.set_parameter(addr, &[(ParameterKey::WorkTgtMode, vec![state as u8])], wait)
            .await
    }

    /// Select the point-cloud data format.
    pub async fn set_data_type(
        &self,
        addr: SocketAddr,
        data_type: DataType,
        wait: Duration,
    ) -> Result<()> {
        self.set_parameter(addr, &[(ParameterKey::PclDataType, vec![data_type as u8])], wait)
            .await
    }

    /// Convenience: configure stream destinations, data type, and start sampling.
    pub async fn start_streaming(
        &self,
        addr: SocketAddr,
        data_dst: SocketAddr,
        imu_dst: SocketAddr,
        data_type: DataType,
        wait: Duration,
    ) -> Result<()> {
        self.set_stream_destinations(addr, data_dst, imu_dst, wait).await?;
        self.set_data_type(addr, data_type, wait).await?;
        self.set_work_mode(addr, crate::protocol::LidarState::Sampling, wait)
            .await?;
        Ok(())
    }
}

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

    /// Wait for the next point-cloud packet.
    pub async fn next_point_cloud(&self, wait: Duration) -> Result<DataPacket> {
        let mut buf = [0u8; 8192];
        let (len, _) = match timeout(wait, self.data_socket.recv_from(&mut buf)).await {
            Ok(Ok(v)) => v,
            Ok(Err(e)) => return Err(e.into()),
            Err(_) => return Err(LidarError::NoResponse {
                addr: self.data_socket.local_addr()?,
            }),
        };
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

    /// Wait for the next IMU packet.
    pub async fn next_imu(&self, wait: Duration) -> Result<DataPacket> {
        let mut buf = [0u8; 1024];
        let (len, _) = match timeout(wait, self.imu_socket.recv_from(&mut buf)).await {
            Ok(Ok(v)) => v,
            Ok(Err(e)) => return Err(e.into()),
            Err(_) => return Err(LidarError::NoResponse {
                addr: self.imu_socket.local_addr()?,
            }),
        };
        DataPacket::parse(&buf[..len])
    }
}

/// Try to find the directed broadcast address for a bound local IPv4 address.
fn directed_broadcast_for(local_ip: Option<Ipv4Addr>) -> Option<Ipv4Addr> {
    let local_ip = local_ip?;
    if local_ip.is_unspecified() || local_ip.is_loopback() {
        return None;
    }
    if_addrs::get_if_addrs().ok()?.into_iter().find_map(|iface| {
        if let if_addrs::IfAddr::V4(v4) = iface.addr {
            if v4.ip == local_ip {
                v4.broadcast
            } else {
                None
            }
        } else {
            None
        }
    })
}

/// Print a diagnostic dump of a received data frame when LIDAR_DEBUG is set.
/// Tries several plausible CRC ranges so we can see which one the LiDAR uses.
fn diagnose_packet(label: &str, buf: &[u8], err: &LidarError) {
    use crate::crc::crc32;
    use crate::packet::DATA_HEADER_SIZE;

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
        eprintln!("crc {name:32} = 0x{c:08X} {}", if c == stored { "<-- MATCH" } else { "" });
    }

    // header[0..24] + timestamp+data[28..length]
    let mut combined = Vec::with_capacity(end - 4);
    combined.extend_from_slice(&buf[0..24]);
    combined.extend_from_slice(&buf[28..end]);
    let c = crc32(&combined);
    eprintln!(
        "crc [0..24]+[28..length] header+ts+data = 0x{c:08X} {}",
        if c == stored { "<-- MATCH" } else { "" }
    );

    // whole packet except crc field
    let mut whole = Vec::with_capacity(end - 4);
    whole.extend_from_slice(&buf[0..24]);
    whole.extend_from_slice(&buf[28..end]);
    // (same as above; also try including crc field region as zeros)
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
    use crate::command::CommandFrame;
    use crate::packet::{DataFrameHeader, DataPayload};
    use crate::points::{Cartesian32Point, Tag};
    use crate::protocol::{CmdType, DataType, SenderType, TimestampType};

    #[tokio::test]
    async fn discover_mock_lidar() {
        let client = LivoxClient::new(SocketAddr::from((Ipv4Addr::LOCALHOST, 0)))
            .await
            .unwrap();
        let mock = UdpSocket::bind(SocketAddr::from((Ipv4Addr::LOCALHOST, 0)))
            .await
            .unwrap();
        let mock_addr = mock.local_addr().unwrap();

        let response_data = {
            let mut d = vec![0u8; 24];
            d[0] = ReturnCode::Success as u8;
            d[1] = 9; // Mid360
            d[2..18].copy_from_slice(b"MID360-TEST12345");
            d[18..22].copy_from_slice(&[127, 0, 0, 1]);
            d[22..24].copy_from_slice(&56100u16.to_le_bytes());
            d
        };
        let ack = CommandFrame {
            version: 0,
            length: (24 + response_data.len()) as u16,
            seq_num: 0,
            cmd_id: CmdId::Discovery.as_u16(),
            cmd_type: CmdType::Ack,
            sender_type: SenderType::Lidar,
            data: response_data,
        };

        tokio::spawn(async move {
            let mut buf = [0u8; 1024];
            let (len, from) = mock.recv_from(&mut buf).await.unwrap();
            let req = CommandFrame::parse(&buf[..len]).unwrap();
            assert_eq!(req.cmd_id, CmdId::Discovery.as_u16());

            let mut out = vec![0u8; 1024];
            ack.write(&mut out).unwrap();
            mock.send_to(&out[..ack.length as usize], from).await.unwrap();
        });

        let devices = client.discover(mock_addr, Duration::from_secs(1)).await.unwrap();
        assert_eq!(devices.len(), 1);
        assert_eq!(devices[0].dev_type, 9);
        assert_eq!(
            devices[0].lidar_cmd_addr,
            SocketAddr::from((Ipv4Addr::LOCALHOST, 56100))
        );
    }

    #[tokio::test]
    async fn send_command_and_receive_ack() {
        let client = LivoxClient::new(SocketAddr::from((Ipv4Addr::LOCALHOST, 0)))
            .await
            .unwrap();
        let mock = UdpSocket::bind(SocketAddr::from((Ipv4Addr::LOCALHOST, 0)))
            .await
            .unwrap();
        let mock_addr = mock.local_addr().unwrap();

        tokio::spawn(async move {
            let mut buf = [0u8; 2048];
            let (len, from) = mock.recv_from(&mut buf).await.unwrap();
            let req = CommandFrame::parse(&buf[..len]).unwrap();
            assert_eq!(req.cmd_id, CmdId::ParamConfig.as_u16());

            let ack = CommandFrame {
                version: 0,
                length: 25,
                seq_num: req.seq_num,
                cmd_id: req.cmd_id,
                cmd_type: CmdType::Ack,
                sender_type: SenderType::Lidar,
                data: vec![ReturnCode::Success as u8],
            };
            let mut out = vec![0u8; 1024];
            ack.write(&mut out).unwrap();
            mock.send_to(&out[..ack.length as usize], from).await.unwrap();
        });

        let payload = build_param_config(&[(ParameterKey::PclDataType, vec![1])]);
        let ack = client
            .send_command(mock_addr, CmdId::ParamConfig.as_u16(), payload, Duration::from_secs(1))
            .await
            .unwrap();
        assert_eq!(ack.cmd_type, CmdType::Ack);
    }

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
        let payload = DataPayload::Points(vec![crate::points::Point::Cartesian32(point)]);

        tokio::spawn(async move {
            let mock = UdpSocket::bind(SocketAddr::from((Ipv4Addr::LOCALHOST, 0)))
                .await
                .unwrap();
            let mut buf = vec![0u8; 1024];
            DataPacket::build(&header, &payload, &mut buf).unwrap();
            let total = crate::packet::DATA_HEADER_SIZE + Cartesian32Point::SIZE;
            mock.send_to(&buf[..total], data_addr).await.unwrap();
        });

        let packet = stream.next_point_cloud(Duration::from_secs(1)).await.unwrap();
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
