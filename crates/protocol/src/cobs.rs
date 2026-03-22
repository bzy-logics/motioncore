//! Consistent Overhead Byte Stuffing.

/// Worst-case encoded length (without the trailing `0x00` delimiter).
pub const fn max_encoded_len(len: usize) -> usize {
    len + len / 254 + 1
}

/// Errors produced by COBS encode/decode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CobsError {
    /// The output buffer is too small.
    BufferTooSmall,
    /// The input is not valid COBS (zero byte or code pointing past the end).
    Malformed,
}

/// Encode `input` into `out`. Returns the encoded length. Does **not** append
/// the frame delimiter.
pub fn encode(input: &[u8], out: &mut [u8]) -> Result<usize, CobsError> {
    if out.len() < max_encoded_len(input.len()) {
        return Err(CobsError::BufferTooSmall);
    }
    let mut code_idx = 0;
    let mut code: u8 = 1;
    let mut w = 1;
    for &b in input {
        if b == 0 {
            out[code_idx] = code;
            code_idx = w;
            w += 1;
            code = 1;
        } else {
            out[w] = b;
            w += 1;
            code += 1;
            if code == 0xFF {
                out[code_idx] = code;
                code_idx = w;
                w += 1;
                code = 1;
            }
        }
    }
    out[code_idx] = code;
    Ok(w)
}

/// Decode `input` (without delimiter) into `out`. Returns the decoded length.
pub fn decode(input: &[u8], out: &mut [u8]) -> Result<usize, CobsError> {
    let mut r = 0;
    let mut w = 0;
    while r < input.len() {
        let code = input[r];
        if code == 0 {
            return Err(CobsError::Malformed);
        }
        r += 1;
        let run = (code - 1) as usize;
        if r + run > input.len() {
            return Err(CobsError::Malformed);
        }
        if w + run > out.len() {
            return Err(CobsError::BufferTooSmall);
        }
        for i in 0..run {
            let b = input[r + i];
            if b == 0 {
                return Err(CobsError::Malformed);
            }
            out[w + i] = b;
        }
        w += run;
        r += run;
        if code != 0xFF && r < input.len() {
            if w >= out.len() {
                return Err(CobsError::BufferTooSmall);
            }
            out[w] = 0;
            w += 1;
        }
    }
    Ok(w)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn roundtrip(data: &[u8]) {
        let mut enc = [0u8; 600];
        let mut dec = [0u8; 600];
        let n = encode(data, &mut enc).unwrap();
        assert!(!enc[..n].contains(&0), "encoded output must not contain zero");
        let m = decode(&enc[..n], &mut dec).unwrap();
        assert_eq!(&dec[..m], data);
    }

    #[test]
    fn known_vectors() {
        let mut out = [0u8; 16];
        let n = encode(&[0x11, 0x22, 0x00, 0x33], &mut out).unwrap();
        assert_eq!(&out[..n], &[0x03, 0x11, 0x22, 0x02, 0x33]);
        let n = encode(&[0x00], &mut out).unwrap();
        assert_eq!(&out[..n], &[0x01, 0x01]);
        let n = encode(&[], &mut out).unwrap();
        assert_eq!(&out[..n], &[0x01]);
    }

    #[test]
    fn roundtrips() {
        roundtrip(&[]);
        roundtrip(&[0]);
        roundtrip(&[0, 0, 0]);
        roundtrip(&[1, 2, 3, 0, 4, 0]);
        let long: [u8; 300] = core::array::from_fn(|i| (i % 255 + 1) as u8);
        roundtrip(&long);
        let mixed: [u8; 520] = core::array::from_fn(|i| (i % 7) as u8);
        roundtrip(&mixed);
    }

    #[test]
    fn rejects_malformed() {
        let mut out = [0u8; 16];
        assert_eq!(decode(&[0x05, 0x11], &mut out), Err(CobsError::Malformed));
        assert_eq!(decode(&[0x02, 0x00], &mut out), Err(CobsError::Malformed));
    }
}
