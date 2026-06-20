use crate::error::{LidarError, Result};
use crate::protocol::DataType;

/// 8-bit tag carrying confidence / property flags for a single point.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Tag(pub u8);

impl Tag {
    pub fn raw(self) -> u8 {
        self.0
    }

    /// Confidence of the main detection point (bits 4-5).
    /// 0 = high, 1 = medium, 2 = low, 3 = reserved.
    pub fn detection_confidence(self) -> u8 {
        (self.0 >> 4) & 0x03
    }

    /// Confidence for rain/fog/dust particles (bits 2-3).
    pub fn particle_confidence(self) -> u8 {
        (self.0 >> 2) & 0x03
    }

    /// Confidence for adhesion/glue points between adjacent objects (bits 0-1).
    pub fn adhesion_confidence(self) -> u8 {
        self.0 & 0x03
    }

    pub fn is_high_confidence(self) -> bool {
        self.detection_confidence() == 0
            && self.particle_confidence() == 0
            && self.adhesion_confidence() == 0
    }
}

/// Single-return Cartesian point with 32-bit millimetre resolution.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cartesian32Point {
    pub x_mm: i32,
    pub y_mm: i32,
    pub z_mm: i32,
    pub reflectivity: u8,
    pub tag: Tag,
}

impl Cartesian32Point {
    pub const SIZE: usize = 14;

    pub fn parse(buf: &[u8]) -> Result<Self> {
        if buf.len() < Self::SIZE {
            return Err(LidarError::PacketTooShort {
                need: Self::SIZE,
                got: buf.len(),
            });
        }
        Ok(Self {
            x_mm: i32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
            y_mm: i32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
            z_mm: i32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
            reflectivity: buf[12],
            tag: Tag(buf[13]),
        })
    }

    pub fn write(&self, buf: &mut [u8]) -> Result<()> {
        if buf.len() < Self::SIZE {
            return Err(LidarError::BufferTooSmall);
        }
        buf[0..4].copy_from_slice(&self.x_mm.to_le_bytes());
        buf[4..8].copy_from_slice(&self.y_mm.to_le_bytes());
        buf[8..12].copy_from_slice(&self.z_mm.to_le_bytes());
        buf[12] = self.reflectivity;
        buf[13] = self.tag.0;
        Ok(())
    }

    pub fn coords_m(&self) -> (f32, f32, f32) {
        (
            self.x_mm as f32 / 1000.0,
            self.y_mm as f32 / 1000.0,
            self.z_mm as f32 / 1000.0,
        )
    }
}

/// Single-return Cartesian point with 16-bit 10 mm resolution.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cartesian16Point {
    pub x_10mm: i16,
    pub y_10mm: i16,
    pub z_10mm: i16,
    pub reflectivity: u8,
    pub tag: Tag,
}

impl Cartesian16Point {
    pub const SIZE: usize = 8;

    pub fn parse(buf: &[u8]) -> Result<Self> {
        if buf.len() < Self::SIZE {
            return Err(LidarError::PacketTooShort {
                need: Self::SIZE,
                got: buf.len(),
            });
        }
        Ok(Self {
            x_10mm: i16::from_le_bytes([buf[0], buf[1]]),
            y_10mm: i16::from_le_bytes([buf[2], buf[3]]),
            z_10mm: i16::from_le_bytes([buf[4], buf[5]]),
            reflectivity: buf[6],
            tag: Tag(buf[7]),
        })
    }

    pub fn write(&self, buf: &mut [u8]) -> Result<()> {
        if buf.len() < Self::SIZE {
            return Err(LidarError::BufferTooSmall);
        }
        buf[0..2].copy_from_slice(&self.x_10mm.to_le_bytes());
        buf[2..4].copy_from_slice(&self.y_10mm.to_le_bytes());
        buf[4..6].copy_from_slice(&self.z_10mm.to_le_bytes());
        buf[6] = self.reflectivity;
        buf[7] = self.tag.0;
        Ok(())
    }

    pub fn coords_m(&self) -> (f32, f32, f32) {
        (
            self.x_10mm as f32 * 0.01,
            self.y_10mm as f32 * 0.01,
            self.z_10mm as f32 * 0.01,
        )
    }
}

/// Single-return spherical coordinate point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SphericalPoint {
    pub depth_mm: u32,
    pub theta_0_01_deg: u16,
    pub phi_0_01_deg: u16,
    pub reflectivity: u8,
    pub tag: Tag,
}

impl SphericalPoint {
    pub const SIZE: usize = 10;

    pub fn parse(buf: &[u8]) -> Result<Self> {
        if buf.len() < Self::SIZE {
            return Err(LidarError::PacketTooShort {
                need: Self::SIZE,
                got: buf.len(),
            });
        }
        Ok(Self {
            depth_mm: u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
            theta_0_01_deg: u16::from_le_bytes([buf[4], buf[5]]),
            phi_0_01_deg: u16::from_le_bytes([buf[6], buf[7]]),
            reflectivity: buf[8],
            tag: Tag(buf[9]),
        })
    }

    pub fn write(&self, buf: &mut [u8]) -> Result<()> {
        if buf.len() < Self::SIZE {
            return Err(LidarError::BufferTooSmall);
        }
        buf[0..4].copy_from_slice(&self.depth_mm.to_le_bytes());
        buf[4..6].copy_from_slice(&self.theta_0_01_deg.to_le_bytes());
        buf[6..8].copy_from_slice(&self.phi_0_01_deg.to_le_bytes());
        buf[8] = self.reflectivity;
        buf[9] = self.tag.0;
        Ok(())
    }

    /// Convert spherical coordinates to a Cartesian point in metres.
    pub fn to_cartesian_m(&self) -> (f32, f32, f32) {
        let theta = (self.theta_0_01_deg as f32) * 0.01f32.to_radians();
        let phi = (self.phi_0_01_deg as f32) * 0.01f32.to_radians();
        let r = self.depth_mm as f32 / 1000.0;
        let sin_theta = theta.sin();
        (
            r * sin_theta * phi.cos(),
            r * sin_theta * phi.sin(),
            r * theta.cos(),
        )
    }
}

/// Inertial measurement sample from the LiDAR IMU.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    pub gyro_x: f32,
    pub gyro_y: f32,
    pub gyro_z: f32,
    pub acc_x: f32,
    pub acc_y: f32,
    pub acc_z: f32,
}

impl ImuSample {
    pub const SIZE: usize = 24;

    pub fn parse(buf: &[u8]) -> Result<Self> {
        if buf.len() < Self::SIZE {
            return Err(LidarError::PacketTooShort {
                need: Self::SIZE,
                got: buf.len(),
            });
        }
        Ok(Self {
            gyro_x: f32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]),
            gyro_y: f32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
            gyro_z: f32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]),
            acc_x: f32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]),
            acc_y: f32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]),
            acc_z: f32::from_le_bytes([buf[20], buf[21], buf[22], buf[23]]),
        })
    }

    pub fn write(&self, buf: &mut [u8]) -> Result<()> {
        if buf.len() < Self::SIZE {
            return Err(LidarError::BufferTooSmall);
        }
        buf[0..4].copy_from_slice(&self.gyro_x.to_le_bytes());
        buf[4..8].copy_from_slice(&self.gyro_y.to_le_bytes());
        buf[8..12].copy_from_slice(&self.gyro_z.to_le_bytes());
        buf[12..16].copy_from_slice(&self.acc_x.to_le_bytes());
        buf[16..20].copy_from_slice(&self.acc_y.to_le_bytes());
        buf[20..24].copy_from_slice(&self.acc_z.to_le_bytes());
        Ok(())
    }
}

/// Unified point representation returned by the parser.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Point {
    Cartesian32(Cartesian32Point),
    Cartesian16(Cartesian16Point),
    Spherical(SphericalPoint),
}

impl Point {
    pub fn coords_m(&self) -> (f32, f32, f32) {
        match self {
            Point::Cartesian32(p) => p.coords_m(),
            Point::Cartesian16(p) => p.coords_m(),
            Point::Spherical(p) => p.to_cartesian_m(),
        }
    }

    pub fn reflectivity(&self) -> u8 {
        match self {
            Point::Cartesian32(p) => p.reflectivity,
            Point::Cartesian16(p) => p.reflectivity,
            Point::Spherical(p) => p.reflectivity,
        }
    }

    pub fn tag(&self) -> Tag {
        match self {
            Point::Cartesian32(p) => p.tag,
            Point::Cartesian16(p) => p.tag,
            Point::Spherical(p) => p.tag,
        }
    }

    pub fn parse(data_type: DataType, buf: &[u8]) -> Result<Self> {
        match data_type {
            DataType::PointCloudCartesian32 => {
                Cartesian32Point::parse(buf).map(Point::Cartesian32)
            }
            DataType::PointCloudCartesian16 => {
                Cartesian16Point::parse(buf).map(Point::Cartesian16)
            }
            DataType::PointCloudSpherical => SphericalPoint::parse(buf).map(Point::Spherical),
            DataType::Imu => Err(LidarError::InvalidDataType(0)),
        }
    }
}
