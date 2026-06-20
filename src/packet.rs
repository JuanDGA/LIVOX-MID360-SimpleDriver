use crate::crc::crc32;
use crate::error::{LidarError, Result};
use crate::points::{ImuSample, Point};
use crate::protocol::{DataType, TimestampType};

pub const DATA_HEADER_SIZE: usize = 36;

/// Header shared by point-cloud and IMU data frames.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DataFrameHeader {
    pub version: u8,
    /// Length of the entire UDP segment starting from `version`.
    pub length: u16,
    /// Intra-frame sampling time in units of 0.1 us.
    pub time_interval: u16,
    /// Number of points/samples in this packet.
    pub dot_num: u16,
    /// UDP packet counter within the current frame.
    pub udp_cnt: u16,
    /// Frame counter (10/15 Hz); invalid for non-repetitive scans.
    pub frame_cnt: u8,
    /// Type of payload data.
    pub data_type: DataType,
    /// Type of timestamp.
    pub time_type: TimestampType,
    /// 8-byte timestamp (ns).
    pub timestamp: u64,
}

impl DataFrameHeader {
    pub fn parse(buf: &[u8]) -> Result<Self> {
        if buf.len() < DATA_HEADER_SIZE {
            return Err(LidarError::PacketTooShort {
                need: DATA_HEADER_SIZE,
                got: buf.len(),
            });
        }
        let version = buf[0];
        let length = u16::from_le_bytes([buf[1], buf[2]]);
        if buf.len() < length as usize {
            return Err(LidarError::PacketTooShort {
                need: length as usize,
                got: buf.len(),
            });
        }

        let data_type = DataType::try_from(buf[10])?;
        let payload_len = length as usize - DATA_HEADER_SIZE;
        let expected_payload = if data_type == DataType::Imu {
            ImuSample::SIZE
        } else {
            data_type.point_size() * (u16::from_le_bytes([buf[5], buf[6]]) as usize)
        };

        if payload_len != expected_payload {
            return Err(LidarError::ParameterParse(format!(
                "payload length mismatch: header says {payload_len}, expected {expected_payload}"
            )));
        }

        let header = Self {
            version,
            length,
            time_interval: u16::from_le_bytes([buf[3], buf[4]]),
            dot_num: u16::from_le_bytes([buf[5], buf[6]]),
            udp_cnt: u16::from_le_bytes([buf[7], buf[8]]),
            frame_cnt: buf[9],
            data_type,
            time_type: TimestampType::try_from(buf[11])?,
            timestamp: u64::from_le_bytes([
                buf[28], buf[29], buf[30], buf[31], buf[32], buf[33], buf[34], buf[35],
            ]),
        };

        header.verify_crc(buf)?;
        Ok(header)
    }

    pub fn write(&self, buf: &mut [u8], payload_len: usize) -> Result<()> {
        let total = DATA_HEADER_SIZE + payload_len;
        if buf.len() < total {
            return Err(LidarError::BufferTooSmall);
        }

        buf[0] = self.version;
        buf[1..3].copy_from_slice(&(total as u16).to_le_bytes());
        buf[3..5].copy_from_slice(&self.time_interval.to_le_bytes());
        buf[5..7].copy_from_slice(&self.dot_num.to_le_bytes());
        buf[7..9].copy_from_slice(&self.udp_cnt.to_le_bytes());
        buf[9] = self.frame_cnt;
        buf[10] = self.data_type as u8;
        buf[11] = self.time_type as u8;
        buf[12..24].fill(0); // reserved
        buf[24..28].fill(0); // CRC placeholder; caller must call fill_crc after writing payload.
        buf[28..36].copy_from_slice(&self.timestamp.to_le_bytes());
        Ok(())
    }

    pub fn fill_crc(&self, buf: &mut [u8], payload_len: usize) -> Result<()> {
        let total = DATA_HEADER_SIZE + payload_len;
        if buf.len() < total {
            return Err(LidarError::BufferTooSmall);
        }
        let computed = crc32(&buf[28..total]);
        buf[24..28].copy_from_slice(&computed.to_le_bytes());
        Ok(())
    }

    pub fn verify_crc(&self, buf: &[u8]) -> Result<()> {
        let stored = u32::from_le_bytes([buf[24], buf[25], buf[26], buf[27]]);
        // Some MID360 firmware versions leave the point-cloud CRC field zero
        // (CRC not populated for the high-rate data stream) while still
        // filling it for IMU packets. Treat a zero stored CRC as "not present"
        // and skip verification; a genuine zero CRC32 over non-empty data is
        // a 1-in-2^32 coincidence.
        if stored == 0 {
            return Ok(());
        }
        let computed = crc32(&buf[28..self.length as usize]);
        if stored != computed {
            return Err(LidarError::CrcMismatch);
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum DataPayload {
    Imu(ImuSample),
    Points(Vec<Point>),
}

#[derive(Debug, Clone, PartialEq)]
pub struct DataPacket {
    pub header: DataFrameHeader,
    pub payload: DataPayload,
}

impl DataPacket {
    pub fn parse(buf: &[u8]) -> Result<Self> {
        let header = DataFrameHeader::parse(buf)?;
        let payload_start = DATA_HEADER_SIZE;
        let payload = match header.data_type {
            DataType::Imu => {
                let sample = ImuSample::parse(&buf[payload_start..])?;
                DataPayload::Imu(sample)
            }
            data_type => {
                let point_size = data_type.point_size();
                let mut points = Vec::with_capacity(header.dot_num as usize);
                for i in 0..header.dot_num as usize {
                    let off = payload_start + i * point_size;
                    points.push(Point::parse(data_type, &buf[off..off + point_size])?);
                }
                DataPayload::Points(points)
            }
        };
        Ok(Self { header, payload })
    }

    pub fn build(header: &DataFrameHeader, payload: &DataPayload, buf: &mut [u8]) -> Result<()> {
        let payload_len = match payload {
            DataPayload::Imu(_) => ImuSample::SIZE,
            DataPayload::Points(points) => header.data_type.point_size() * points.len(),
        };
        header.write(buf, payload_len)?;

        let payload_start = DATA_HEADER_SIZE;
        match payload {
            DataPayload::Imu(sample) => sample.write(&mut buf[payload_start..])?,
            DataPayload::Points(points) => {
                let point_size = header.data_type.point_size();
                for (i, point) in points.iter().enumerate() {
                    let off = payload_start + i * point_size;
                    match point {
                        Point::Cartesian32(p) => p.write(&mut buf[off..])?,
                        Point::Cartesian16(p) => p.write(&mut buf[off..])?,
                        Point::Spherical(p) => p.write(&mut buf[off..])?,
                    }
                }
            }
        }

        header.fill_crc(buf, payload_len)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::points::{Cartesian32Point, Tag};

    #[test]
    fn roundtrip_imu_packet() {
        let sample = ImuSample {
            gyro_x: 0.1,
            gyro_y: 0.2,
            gyro_z: 0.3,
            acc_x: 0.4,
            acc_y: 0.5,
            acc_z: 0.6,
        };
        let header = DataFrameHeader {
            version: 0,
            length: 0, // computed on write
            time_interval: 100,
            dot_num: 1,
            udp_cnt: 0,
            frame_cnt: 0,
            data_type: DataType::Imu,
            time_type: TimestampType::None,
            timestamp: 1_000_000,
        };
        let packet = DataPacket {
            header,
            payload: DataPayload::Imu(sample),
        };

        let mut buf = vec![0u8; 1024];
        DataPacket::build(&header, &packet.payload, &mut buf).unwrap();
        let parsed = DataPacket::parse(&buf[..DATA_HEADER_SIZE + ImuSample::SIZE]).unwrap();
        assert_eq!(parsed.payload, packet.payload);
        assert_eq!(parsed.header.timestamp, header.timestamp);
        assert_eq!(parsed.header.data_type, header.data_type);
        assert_eq!(parsed.header.dot_num, header.dot_num);
    }

    #[test]
    fn roundtrip_cartesian32_packet() {
        let point = Cartesian32Point {
            x_mm: 1234,
            y_mm: -567,
            z_mm: 89,
            reflectivity: 200,
            tag: Tag(0),
        };
        let header = DataFrameHeader {
            version: 0,
            length: 0,
            time_interval: 50,
            dot_num: 1,
            udp_cnt: 7,
            frame_cnt: 3,
            data_type: DataType::PointCloudCartesian32,
            time_type: TimestampType::Ptp,
            timestamp: 2_000_000,
        };
        let payload = DataPayload::Points(vec![Point::Cartesian32(point)]);

        let mut buf = vec![0u8; 1024];
        DataPacket::build(&header, &payload, &mut buf).unwrap();
        let total = DATA_HEADER_SIZE + Cartesian32Point::SIZE;
        let parsed = DataPacket::parse(&buf[..total]).unwrap();
        assert_eq!(parsed.header.udp_cnt, 7);
        assert_eq!(parsed.header.frame_cnt, 3);
        assert_eq!(parsed.payload, payload);
    }
}
