/// CRC-16/CCITT-FALSE as used for the MID360 command header.
/// Polynomial 0x1021, init 0xFFFF, no reflection, xorout 0x0000.
pub fn crc16_ccitt_false(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// Standard CRC-32 (Ethernet) as used for the MID360 data payload and command data.
/// Polynomial 0x04C11DB7, init 0xFFFFFFFF, reflected input/output, xorout 0xFFFFFFFF.
pub fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFFFFFF;
    for &byte in data {
        crc ^= byte as u32;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    crc ^ 0xFFFFFFFF
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc16_known_value() {
        // "123456789" is the standard CRC test string.
        assert_eq!(crc16_ccitt_false(b"123456789"), 0x29B1);
    }

    #[test]
    fn crc32_known_value() {
        assert_eq!(crc32(b"123456789"), 0xCBF43926);
    }
}
