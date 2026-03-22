//! CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no xorout).

const POLY: u16 = 0x1021;

/// Lookup table generated at compile time.
const TABLE: [u16; 256] = {
    let mut table = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        let mut crc = (i as u16) << 8;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ POLY } else { crc << 1 };
            bit += 1;
        }
        table[i] = crc;
        i += 1;
    }
    table
};

/// Compute CRC-16/CCITT-FALSE over `data`.
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in data {
        crc = (crc << 8) ^ TABLE[(((crc >> 8) as u8) ^ b) as usize];
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn check_value() {
        // Standard check value for CRC-16/CCITT-FALSE.
        assert_eq!(crc16(b"123456789"), 0x29B1);
    }

    #[test]
    fn empty() {
        assert_eq!(crc16(&[]), 0xFFFF);
    }
}
