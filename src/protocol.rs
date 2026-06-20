use crate::error::{LidarError, Result};

// Default MID360 ports.
pub const DISCOVERY_PORT: u16 = 56000;
pub const CMD_PORT: u16 = 56100;
pub const PUSH_PORT: u16 = 56200;
pub const DATA_PORT: u16 = 56300;
pub const IMU_PORT: u16 = 56400;
pub const LOG_PORT: u16 = 56500;

// Default host-side ports.
pub const HOST_CMD_PORT: u16 = 56101;
pub const HOST_PUSH_PORT: u16 = 56201;
pub const HOST_DATA_PORT: u16 = 56301;
pub const HOST_IMU_PORT: u16 = 56401;
pub const HOST_LOG_PORT: u16 = 56501;

pub const SOF: u8 = 0xAA;
pub const PROTOCOL_VERSION: u8 = 0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DataType {
    Imu = 0,
    PointCloudCartesian32 = 1,
    PointCloudCartesian16 = 2,
    PointCloudSpherical = 3,
}

impl DataType {
    pub fn points_per_packet(self) -> usize {
        match self {
            DataType::Imu => 1,
            DataType::PointCloudCartesian32 => 96,
            DataType::PointCloudCartesian16 => 96,
            DataType::PointCloudSpherical => 96,
        }
    }

    pub fn point_size(self) -> usize {
        match self {
            DataType::Imu => 24,
            DataType::PointCloudCartesian32 => 14,
            DataType::PointCloudCartesian16 => 8,
            DataType::PointCloudSpherical => 10,
        }
    }
}

impl TryFrom<u8> for DataType {
    type Error = LidarError;

    fn try_from(value: u8) -> Result<Self> {
        match value {
            0 => Ok(DataType::Imu),
            1 => Ok(DataType::PointCloudCartesian32),
            2 => Ok(DataType::PointCloudCartesian16),
            3 => Ok(DataType::PointCloudSpherical),
            other => Err(LidarError::InvalidDataType(other)),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TimestampType {
    None = 0,
    Ptp = 1,
    Gps = 2,
}

impl TryFrom<u8> for TimestampType {
    type Error = LidarError;

    fn try_from(value: u8) -> Result<Self> {
        match value {
            0 => Ok(TimestampType::None),
            1 => Ok(TimestampType::Ptp),
            2 => Ok(TimestampType::Gps),
            other => Err(LidarError::InvalidTimestampType(other)),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CmdType {
    Req = 0,
    Ack = 1,
}

impl TryFrom<u8> for CmdType {
    type Error = LidarError;

    fn try_from(value: u8) -> Result<Self> {
        match value {
            0 => Ok(CmdType::Req),
            1 => Ok(CmdType::Ack),
            other => Err(LidarError::InvalidCmdType(other)),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SenderType {
    Host = 0,
    Lidar = 1,
}

impl TryFrom<u8> for SenderType {
    type Error = LidarError;

    fn try_from(value: u8) -> Result<Self> {
        match value {
            0 => Ok(SenderType::Host),
            1 => Ok(SenderType::Lidar),
            other => Err(LidarError::InvalidSenderType(other)),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum LidarState {
    Sampling = 0x01,
    Idle = 0x02,
    Error = 0x04,
    SelfCheck = 0x05,
    MotorStartup = 0x06,
    Upgrade = 0x08,
    Ready = 0x09,
}

impl TryFrom<u8> for LidarState {
    type Error = LidarError;

    fn try_from(value: u8) -> Result<Self> {
        match value {
            0x01 => Ok(LidarState::Sampling),
            0x02 => Ok(LidarState::Idle),
            0x04 => Ok(LidarState::Error),
            0x05 => Ok(LidarState::SelfCheck),
            0x06 => Ok(LidarState::MotorStartup),
            0x08 => Ok(LidarState::Upgrade),
            0x09 => Ok(LidarState::Ready),
            other => Err(LidarError::InvalidLidarState(other)),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ReturnCode {
    Success = 0x00,
    Failure = 0x01,
    NotPermitNow = 0x02,
    OutOfRange = 0x03,
    ParamNotSupport = 0x20,
    ParamRebootEffect = 0x21,
    ParamReadOnly = 0x22,
    ParamInvalidLen = 0x23,
    ParamKeyNumErr = 0x24,
    UpgradePubKeyError = 0x30,
    UpgradeDigestError = 0x31,
    UpgradeFwTypeError = 0x32,
    UpgradeFwOutOfRange = 0x33,
    UpgradeFwErasing = 0x34,
}

impl TryFrom<u8> for ReturnCode {
    type Error = LidarError;

    fn try_from(value: u8) -> Result<Self> {
        match value {
            0x00 => Ok(ReturnCode::Success),
            0x01 => Ok(ReturnCode::Failure),
            0x02 => Ok(ReturnCode::NotPermitNow),
            0x03 => Ok(ReturnCode::OutOfRange),
            0x20 => Ok(ReturnCode::ParamNotSupport),
            0x21 => Ok(ReturnCode::ParamRebootEffect),
            0x22 => Ok(ReturnCode::ParamReadOnly),
            0x23 => Ok(ReturnCode::ParamInvalidLen),
            0x24 => Ok(ReturnCode::ParamKeyNumErr),
            0x30 => Ok(ReturnCode::UpgradePubKeyError),
            0x31 => Ok(ReturnCode::UpgradeDigestError),
            0x32 => Ok(ReturnCode::UpgradeFwTypeError),
            0x33 => Ok(ReturnCode::UpgradeFwOutOfRange),
            0x34 => Ok(ReturnCode::UpgradeFwErasing),
            other => Err(LidarError::InvalidReturnCode(other)),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum CmdId {
    Discovery = 0x0000,
    ParamConfig = 0x0100,
    ParamInquire = 0x0101,
    ParamPush = 0x0102,
    Reboot = 0x0200,
    FactoryReset = 0x0201,
    SetGpsTimestamp = 0x0202,
    LogFilePush = 0x0300,
    LogCollectionConfig = 0x0301,
    LogTimeSync = 0x0302,
    DebugRawDataConfig = 0x0303,
    UpgradeStart = 0x0400,
    UpgradeData = 0x0401,
    UpgradeComplete = 0x0402,
    UpgradeState = 0x0403,
}

impl CmdId {
    pub fn as_u16(self) -> u16 {
        self as u16
    }
}

/// Parameter keys used in key/value configuration and inquiry commands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum ParameterKey {
    PclDataType = 0x0000,
    PatternMode = 0x0001,
    LidarIpCfg = 0x0004,
    StateInfoHostIpCfg = 0x0005,
    PointCloudHostIpCfg = 0x0006,
    ImuHostIpCfg = 0x0007,
    InstallAttitude = 0x0012,
    FovCfg0 = 0x0015,
    FovCfg1 = 0x0016,
    FovCfgEn = 0x0017,
    DetectMode = 0x0018,
    FuncIoCfg = 0x0019,
    WorkTgtMode = 0x001A,
    ImuDataEn = 0x001C,
    SpeedMode = 0x0021,
    TimeFilter = 0x0026,
    Sn = 0x8000,
    ProductInfo = 0x8001,
    VersionApp = 0x8002,
    VersionLoader = 0x8003,
    VersionHardware = 0x8004,
    Mac = 0x8005,
    CurWorkState = 0x8006,
    CoreTemp = 0x8007,
    PowerupCnt = 0x8008,
    LocalTimeNow = 0x8009,
    LastSyncTime = 0x800A,
    TimeOffset = 0x800B,
    TimeSyncType = 0x800C,
    LidarDiagStatus = 0x800E,
    FwType = 0x8010,
    HmsCode = 0x8011,
}

impl ParameterKey {
    pub fn as_u16(self) -> u16 {
        self as u16
    }
}

/// Default recommended host-side ports for a single LiDAR session.
pub fn default_host_sockets(bind_ip: std::net::Ipv4Addr) -> [(std::net::SocketAddr, u16); 4] {
    [
        (std::net::SocketAddr::from((bind_ip, HOST_CMD_PORT)), CMD_PORT),
        (std::net::SocketAddr::from((bind_ip, HOST_PUSH_PORT)), PUSH_PORT),
        (std::net::SocketAddr::from((bind_ip, HOST_DATA_PORT)), DATA_PORT),
        (std::net::SocketAddr::from((bind_ip, HOST_IMU_PORT)), IMU_PORT),
    ]
}
