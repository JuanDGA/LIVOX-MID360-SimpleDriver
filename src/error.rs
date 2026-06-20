use std::net::SocketAddr;

#[derive(Debug, thiserror::Error)]
pub enum LidarError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("packet too short: need {need} bytes, got {got}")]
    PacketTooShort { need: usize, got: usize },

    #[error("invalid data type: {0}")]
    InvalidDataType(u8),

    #[error("invalid timestamp type: {0}")]
    InvalidTimestampType(u8),

    #[error("invalid command type: {0}")]
    InvalidCmdType(u8),

    #[error("invalid sender type: {0}")]
    InvalidSenderType(u8),

    #[error("invalid LiDAR state: {0}")]
    InvalidLidarState(u8),

    #[error("invalid return code: {0}")]
    InvalidReturnCode(u8),

    #[error("CRC mismatch")]
    CrcMismatch,

    #[error("command failed: {code:?}")]
    CommandFailed { code: crate::protocol::ReturnCode },

    #[error("unexpected response: expected cmd_id {expected}, got {got}")]
    UnexpectedResponse { expected: u16, got: u16 },

    #[error("no response from LiDAR at {addr}")]
    NoResponse { addr: SocketAddr },

    #[error("discovery timeout")]
    DiscoveryTimeout,

    #[error("parameter parse error: {0}")]
    ParameterParse(String),

    #[error("buffer too small")]
    BufferTooSmall,
}

pub type Result<T> = std::result::Result<T, LidarError>;
