use std::net::Ipv4Addr;

use crate::crc::{crc16_ccitt_false, crc32};
use crate::error::{LidarError, Result};
use crate::protocol::{CmdType, ParameterKey, ReturnCode, SenderType, SOF};

pub const CMD_HEADER_SIZE: usize = 24;

/// A complete control-command frame.
#[derive(Debug, Clone, PartialEq)]
pub struct CommandFrame {
    pub version: u8,
    /// Total length from `sof` to end of data.
    pub length: u16,
    pub seq_num: u32,
    pub cmd_id: u16,
    pub cmd_type: CmdType,
    pub sender_type: SenderType,
    pub data: Vec<u8>,
}

impl CommandFrame {
    pub fn new_req(seq_num: u32, cmd_id: u16, data: Vec<u8>) -> Self {
        Self {
            version: 0,
            length: (CMD_HEADER_SIZE + data.len()) as u16,
            seq_num,
            cmd_id,
            cmd_type: CmdType::Req,
            sender_type: SenderType::Host,
            data,
        }
    }

    pub fn parse(buf: &[u8]) -> Result<Self> {
        if buf.len() < CMD_HEADER_SIZE {
            return Err(LidarError::PacketTooShort {
                need: CMD_HEADER_SIZE,
                got: buf.len(),
            });
        }
        if buf[0] != SOF {
            return Err(LidarError::ParameterParse(format!(
                "invalid SOF: expected 0xAA, got 0x{:02X}",
                buf[0]
            )));
        }
        let length = u16::from_le_bytes([buf[2], buf[3]]);
        if buf.len() < length as usize {
            return Err(LidarError::PacketTooShort {
                need: length as usize,
                got: buf.len(),
            });
        }

        let stored_header_crc = u16::from_le_bytes([buf[18], buf[19]]);
        let computed_header_crc = crc16_ccitt_false(&buf[0..18]);
        if stored_header_crc != computed_header_crc {
            return Err(LidarError::CrcMismatch);
        }

        let data_len = length as usize - CMD_HEADER_SIZE;
        let stored_data_crc = u32::from_le_bytes([buf[20], buf[21], buf[22], buf[23]]);
        let computed_data_crc = if data_len == 0 {
            0
        } else {
            crc32(&buf[CMD_HEADER_SIZE..length as usize])
        };
        if stored_data_crc != computed_data_crc {
            return Err(LidarError::CrcMismatch);
        }

        Ok(Self {
            version: buf[1],
            length,
            seq_num: u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
            cmd_id: u16::from_le_bytes([buf[8], buf[9]]),
            cmd_type: CmdType::try_from(buf[10])?,
            sender_type: SenderType::try_from(buf[11])?,
            data: buf[CMD_HEADER_SIZE..length as usize].to_vec(),
        })
    }

    pub fn write(&self, buf: &mut [u8]) -> Result<()> {
        if buf.len() < self.length as usize {
            return Err(LidarError::BufferTooSmall);
        }

        buf[0] = SOF;
        buf[1] = self.version;
        buf[2..4].copy_from_slice(&self.length.to_le_bytes());
        buf[4..8].copy_from_slice(&self.seq_num.to_le_bytes());
        buf[8..10].copy_from_slice(&self.cmd_id.to_le_bytes());
        buf[10] = self.cmd_type as u8;
        buf[11] = self.sender_type as u8;
        buf[12..18].fill(0); // reserved
        buf[18..20].copy_from_slice(&0u16.to_le_bytes()); // crc16 placeholder
        let data_len = self.length as usize - CMD_HEADER_SIZE;
        let data_crc = if data_len == 0 {
            0
        } else {
            crc32(&self.data)
        };
        buf[20..24].copy_from_slice(&data_crc.to_le_bytes());
        buf[CMD_HEADER_SIZE..self.length as usize].copy_from_slice(&self.data);

        let header_crc = crc16_ccitt_false(&buf[0..18]);
        buf[18..20].copy_from_slice(&header_crc.to_le_bytes());
        Ok(())
    }

    pub fn is_ack_for(&self, req: &CommandFrame) -> bool {
        self.cmd_type == CmdType::Ack
            && self.cmd_id == req.cmd_id
            && self.seq_num == req.seq_num
            && self.sender_type == SenderType::Lidar
    }
}

/// Encodes a list of `(key, value)` pairs for parameter configuration.
pub fn encode_key_value_list(items: &[(u16, &[u8])]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(4 + items.len() * 8);
    buf.extend_from_slice(&(items.len() as u16).to_le_bytes());
    buf.extend_from_slice(&0u16.to_le_bytes()); // reserved
    for (key, value) in items {
        buf.extend_from_slice(&key.to_le_bytes());
        buf.extend_from_slice(&(value.len() as u16).to_le_bytes());
        buf.extend_from_slice(value);
    }
    buf
}

/// Decodes a `key_value_list` payload into `(key, value)` pairs.
pub fn decode_key_value_list(data: &[u8]) -> Result<KeyValueList> {
    if data.len() < 4 {
        return Err(LidarError::PacketTooShort {
            need: 4,
            got: data.len(),
        });
    }
    let key_num = u16::from_le_bytes([data[0], data[1]]) as usize;
    let mut out = Vec::with_capacity(key_num);
    let mut off = 4;
    for _ in 0..key_num {
        if off + 4 > data.len() {
            return Err(LidarError::PacketTooShort {
                need: off + 4,
                got: data.len(),
            });
        }
        let key = u16::from_le_bytes([data[off], data[off + 1]]);
        let len = u16::from_le_bytes([data[off + 2], data[off + 3]]) as usize;
        off += 4;
        if off + len > data.len() {
            return Err(LidarError::PacketTooShort {
                need: off + len,
                got: data.len(),
            });
        }
        out.push((key, data[off..off + len].to_vec()));
        off += len;
    }
    Ok(out)
}

/// Encodes a simple key list for parameter inquiry.
pub fn encode_key_list(keys: &[u16]) -> Vec<u8> {
    let mut buf = Vec::with_capacity(4 + keys.len() * 2);
    buf.extend_from_slice(&(keys.len() as u16).to_le_bytes());
    buf.extend_from_slice(&0u16.to_le_bytes());
    for key in keys {
        buf.extend_from_slice(&key.to_le_bytes());
    }
    buf
}

/// Discovery broadcast response contents.
#[derive(Debug, Clone, PartialEq)]
pub struct DiscoveryAck {
    pub ret_code: ReturnCode,
    pub dev_type: u8,
    pub serial_number: [u8; 16],
    pub lidar_ip: Ipv4Addr,
    pub cmd_port: u16,
}

impl DiscoveryAck {
    pub fn parse(data: &[u8]) -> Result<Self> {
        if data.len() < 24 {
            return Err(LidarError::PacketTooShort {
                need: 24,
                got: data.len(),
            });
        }
        Ok(Self {
            ret_code: ReturnCode::try_from(data[0])?,
            dev_type: data[1],
            serial_number: data[2..18].try_into().unwrap(),
            lidar_ip: Ipv4Addr::new(data[18], data[19], data[20], data[21]),
            cmd_port: u16::from_le_bytes([data[22], data[23]]),
        })
    }
}

/// Helpers for building common parameter values.
pub fn host_ip_cfg_value(ip: Ipv4Addr, dest_port: u16, source_port: u16) -> Vec<u8> {
    let mut v = Vec::with_capacity(8);
    v.extend_from_slice(&ip.octets());
    v.extend_from_slice(&dest_port.to_le_bytes());
    v.extend_from_slice(&source_port.to_le_bytes());
    v
}

pub fn lidar_ip_cfg_value(ip: Ipv4Addr, mask: Ipv4Addr, gateway: Ipv4Addr) -> Vec<u8> {
    let mut v = Vec::with_capacity(12);
    v.extend_from_slice(&ip.octets());
    v.extend_from_slice(&mask.octets());
    v.extend_from_slice(&gateway.octets());
    v
}

/// Build a parameter configuration command payload.
pub fn build_param_config(keys: &[(ParameterKey, Vec<u8>)]) -> Vec<u8> {
    let items: Vec<(u16, &[u8])> = keys
        .iter()
        .map(|(k, v)| (k.as_u16(), v.as_slice()))
        .collect();
    encode_key_value_list(&items)
}

/// Build a parameter inquiry command payload.
pub fn build_param_inquire(keys: &[ParameterKey]) -> Vec<u8> {
    encode_key_list(&keys.iter().map(|k| k.as_u16()).collect::<Vec<_>>())
}

pub type KeyValueList = Vec<(u16, Vec<u8>)>;

/// Parse a parameter configuration ACK payload (cmd_id 0x0100).
pub fn parse_param_config_ack(data: &[u8]) -> Result<(ReturnCode, Option<u16>)> {
    if data.is_empty() {
        return Err(LidarError::PacketTooShort {
            need: 1,
            got: 0,
        });
    }
    let ret_code = ReturnCode::try_from(data[0])?;
    let error_key = if data.len() >= 3 {
        Some(u16::from_le_bytes([data[1], data[2]]))
    } else {
        None
    };
    Ok((ret_code, error_key))
}

/// Parse a parameter inquiry ACK payload (cmd_id 0x0101).
pub fn parse_param_inquire_ack(data: &[u8]) -> Result<(ReturnCode, KeyValueList)> {
    if data.is_empty() {
        return Err(LidarError::PacketTooShort {
            need: 1,
            got: 0,
        });
    }
    let ret_code = ReturnCode::try_from(data[0])?;
    let pairs = decode_key_value_list(&data[1..])?;
    Ok((ret_code, pairs))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::CmdId;

    #[test]
    fn command_frame_roundtrip() {
        let data = build_param_config(&[(ParameterKey::PclDataType, vec![1])]);
        let req = CommandFrame::new_req(42, CmdId::ParamConfig.as_u16(), data);
        let mut buf = vec![0u8; 1024];
        req.write(&mut buf).unwrap();
        let parsed = CommandFrame::parse(&buf[..req.length as usize]).unwrap();
        assert_eq!(parsed, req);
    }

    #[test]
    fn empty_data_command_frame() {
        let req = CommandFrame::new_req(1, CmdId::Discovery.as_u16(), Vec::new());
        let mut buf = vec![0u8; 1024];
        req.write(&mut buf).unwrap();
        let parsed = CommandFrame::parse(&buf[..req.length as usize]).unwrap();
        assert_eq!(parsed.data.len(), 0);
        assert_eq!(parsed.cmd_id, CmdId::Discovery.as_u16());
    }

    #[test]
    fn key_value_roundtrip() {
        let items = vec![(0x0006u16, &[192u8, 168, 1, 100, 0x63, 0x01, 0x63, 0x00][..])];
        let encoded = encode_key_value_list(&items);
        let decoded = decode_key_value_list(&encoded).unwrap();
        assert_eq!(items, decoded.iter().map(|(k, v)| (*k, v.as_slice())).collect::<Vec<_>>());
    }

    #[test]
    fn discovery_ack_parse() {
        let mut data = vec![0u8; 24];
        data[0] = 0; // success
        data[1] = 9; // Mid360
        data[2..18].copy_from_slice(b"MID360-123456789");
        data[18..22].copy_from_slice(&[192, 168, 1, 100]);
        data[22..24].copy_from_slice(&56100u16.to_le_bytes());
        let ack = DiscoveryAck::parse(&data).unwrap();
        assert_eq!(ack.ret_code, ReturnCode::Success);
        assert_eq!(ack.lidar_ip, Ipv4Addr::new(192, 168, 1, 100));
        assert_eq!(ack.cmd_port, 56100);
    }
}
