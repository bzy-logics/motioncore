//! ELF -> UF2 conversion and UF2 validation (https://github.com/microsoft/uf2).
//!
//! Only what the firmware images need: 32-bit little-endian ELF executables,
//! loadable segments placed in a given flash window, 256-byte payloads with
//! the family-ID flag (the RP2040 bootrom's requirements).

use anyhow::{bail, ensure, Result};

pub const MAGIC_START0: u32 = 0x0A32_4655;
pub const MAGIC_START1: u32 = 0x9E5D_5157;
pub const MAGIC_END: u32 = 0x0AB1_6F30;
pub const FLAG_FAMILY_ID: u32 = 0x0000_2000;
pub const BLOCK: usize = 512;
pub const PAYLOAD: u32 = 256;

/// Address window the image must lie in.
#[derive(Debug, Clone, Copy)]
pub struct FlashRegion {
    pub start: u32,
    pub size: u32,
}

/// RP2040 external QSPI flash (Raspberry Pi Pico: 2 MiB).
pub const RP2040_FLASH: FlashRegion = FlashRegion { start: 0x1000_0000, size: 2 * 1024 * 1024 };

fn u16_at(b: &[u8], o: usize) -> Result<u16> {
    Ok(u16::from_le_bytes(b.get(o..o + 2).ok_or_else(|| anyhow::anyhow!("ELF truncated"))?.try_into()?))
}
fn u32_at(b: &[u8], o: usize) -> Result<u32> {
    Ok(u32::from_le_bytes(b.get(o..o + 4).ok_or_else(|| anyhow::anyhow!("ELF truncated"))?.try_into()?))
}

/// Loadable bytes of an ELF image, as (load address, data) pairs.
pub fn elf_segments(elf: &[u8]) -> Result<Vec<(u32, Vec<u8>)>> {
    ensure!(elf.len() >= 52 && &elf[..4] == b"\x7fELF", "not an ELF file");
    ensure!(elf[4] == 1, "not a 32-bit ELF");
    ensure!(elf[5] == 1, "not a little-endian ELF");
    ensure!(u16_at(elf, 16)? == 2, "not an executable ELF");
    ensure!(u16_at(elf, 18)? == 40, "not an ARM ELF");
    let phoff = u32_at(elf, 28)? as usize;
    let phentsize = u16_at(elf, 42)? as usize;
    let phnum = u16_at(elf, 44)? as usize;
    ensure!(phentsize >= 32, "bad program header size");
    let mut out = Vec::new();
    for i in 0..phnum {
        let ph = phoff + i * phentsize;
        let p_type = u32_at(elf, ph)?;
        let offset = u32_at(elf, ph + 4)? as usize;
        let paddr = u32_at(elf, ph + 12)?;
        let filesz = u32_at(elf, ph + 16)? as usize;
        if p_type != 1 || filesz == 0 {
            continue; // not PT_LOAD, or nothing to store (e.g. .bss)
        }
        let data = elf.get(offset..offset + filesz).ok_or_else(|| anyhow::anyhow!("ELF segment {i} out of file bounds"))?;
        out.push((paddr, data.to_vec()));
    }
    ensure!(!out.is_empty(), "ELF has no loadable data");
    Ok(out)
}

/// Convert an ELF image to UF2 for `family_id`. Every loadable byte must lie
/// in `region`; the image must start at the region start (boot code).
pub fn elf_to_uf2(elf: &[u8], family_id: u32, region: FlashRegion) -> Result<Vec<u8>> {
    let segs = elf_segments(elf)?;
    let end = region.start as u64 + region.size as u64;
    for (addr, data) in &segs {
        let seg_end = *addr as u64 + data.len() as u64;
        if (*addr as u64) < region.start as u64 || seg_end > end {
            bail!("segment {addr:#010x}..{seg_end:#010x} lies outside flash {:#010x}..{end:#010x}", region.start);
        }
    }
    ensure!(segs.iter().any(|(a, _)| *a == region.start), "image has no code at the flash start {:#010x}", region.start);

    // Gather into 256-byte pages (sparse, ordered); unused bytes are zero.
    let mut pages: std::collections::BTreeMap<u32, [u8; PAYLOAD as usize]> = Default::default();
    for (addr, data) in &segs {
        for (k, &b) in data.iter().enumerate() {
            let a = addr + k as u32;
            let page = a & !(PAYLOAD - 1);
            pages.entry(page).or_insert([0; PAYLOAD as usize])[(a - page) as usize] = b;
        }
    }
    let n = pages.len() as u32;
    let mut out = Vec::with_capacity(pages.len() * BLOCK);
    for (i, (addr, payload)) in pages.iter().enumerate() {
        let mut blk = [0u8; BLOCK];
        let words = [MAGIC_START0, MAGIC_START1, FLAG_FAMILY_ID, *addr, PAYLOAD, i as u32, n, family_id];
        for (k, w) in words.iter().enumerate() {
            blk[k * 4..k * 4 + 4].copy_from_slice(&w.to_le_bytes());
        }
        blk[32..32 + PAYLOAD as usize].copy_from_slice(payload);
        blk[BLOCK - 4..].copy_from_slice(&MAGIC_END.to_le_bytes());
        out.extend_from_slice(&blk);
    }
    Ok(out)
}

/// Check a UF2 file: magics, block numbering, family, payload size and that
/// every block targets `region`. Returns the (address, payload) pairs.
// `is_multiple_of` needs Rust 1.87; the workspace MSRV is 1.85.
#[allow(clippy::manual_is_multiple_of)]
pub fn validate(uf2: &[u8], family_id: u32, region: FlashRegion) -> Result<Vec<(u32, Vec<u8>)>> {
    ensure!(!uf2.is_empty() && uf2.len() % BLOCK == 0, "UF2 size {} is not a multiple of {BLOCK}", uf2.len());
    let n = uf2.len() / BLOCK;
    let mut out = Vec::with_capacity(n);
    for (i, blk) in (0..n).map(|i| &uf2[i * BLOCK..(i + 1) * BLOCK]).enumerate() {
        let w = |k: usize| u32::from_le_bytes(blk[k * 4..k * 4 + 4].try_into().unwrap());
        ensure!(w(0) == MAGIC_START0 && w(1) == MAGIC_START1, "block {i}: bad start magic");
        ensure!(u32::from_le_bytes(blk[BLOCK - 4..].try_into().unwrap()) == MAGIC_END, "block {i}: bad end magic");
        ensure!(w(2) & FLAG_FAMILY_ID != 0 && w(7) == family_id, "block {i}: family {:#010x}, expected {family_id:#010x}", w(7));
        ensure!(w(4) == PAYLOAD, "block {i}: payload size {}", w(4));
        ensure!(w(5) == i as u32 && w(6) == n as u32, "block {i}: numbering {}/{}", w(5), w(6));
        let addr = w(3);
        ensure!(
            addr >= region.start && addr as u64 + PAYLOAD as u64 <= region.start as u64 + region.size as u64,
            "block {i}: address {addr:#010x} outside flash"
        );
        out.push((addr, blk[32..32 + PAYLOAD as usize].to_vec()));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Minimal ELF32 ARM executable with the given PT_LOAD segments.
    fn elf(segs: &[(u32, &[u8])]) -> Vec<u8> {
        let phoff = 52usize;
        let data_off = phoff + 32 * segs.len();
        let mut v = vec![0u8; data_off];
        v[..4].copy_from_slice(b"\x7fELF");
        v[4] = 1; // 32-bit
        v[5] = 1; // little endian
        v[6] = 1;
        v[16..18].copy_from_slice(&2u16.to_le_bytes()); // ET_EXEC
        v[18..20].copy_from_slice(&40u16.to_le_bytes()); // EM_ARM
        v[28..32].copy_from_slice(&(phoff as u32).to_le_bytes());
        v[42..44].copy_from_slice(&32u16.to_le_bytes());
        v[44..46].copy_from_slice(&(segs.len() as u16).to_le_bytes());
        let mut off = data_off;
        for (i, (addr, data)) in segs.iter().enumerate() {
            let ph = phoff + 32 * i;
            v[ph..ph + 4].copy_from_slice(&1u32.to_le_bytes());
            v[ph + 4..ph + 8].copy_from_slice(&(off as u32).to_le_bytes());
            v[ph + 8..ph + 12].copy_from_slice(&addr.to_le_bytes()); // vaddr
            v[ph + 12..ph + 16].copy_from_slice(&addr.to_le_bytes()); // paddr
            v[ph + 16..ph + 20].copy_from_slice(&(data.len() as u32).to_le_bytes());
            v[ph + 20..ph + 24].copy_from_slice(&(data.len() as u32).to_le_bytes());
            off += data.len();
        }
        for (_, data) in segs {
            v.extend_from_slice(data);
        }
        v
    }

    const FAM: u32 = 0xE48B_FF56;

    #[test]
    fn converts_and_round_trips() {
        let boot2 = [0xAAu8; 256];
        let code: Vec<u8> = (0..600u32).map(|i| i as u8).collect();
        let e = elf(&[(0x1000_0000, &boot2), (0x1000_0100, &code)]);
        let uf2 = elf_to_uf2(&e, FAM, RP2040_FLASH).unwrap();
        // 256 + 600 bytes -> pages 0x000, 0x100, 0x200, 0x300.
        assert_eq!(uf2.len(), 4 * BLOCK);
        let blocks = validate(&uf2, FAM, RP2040_FLASH).unwrap();
        let addrs: Vec<u32> = blocks.iter().map(|b| b.0).collect();
        assert_eq!(addrs, [0x1000_0000, 0x1000_0100, 0x1000_0200, 0x1000_0300]);
        assert_eq!(blocks[0].1, boot2);
        let mut image: Vec<u8> = blocks[1..].iter().flat_map(|b| b.1.clone()).collect();
        image.truncate(600);
        assert_eq!(image, code);
    }

    #[test]
    fn rejects_images_outside_flash_or_without_boot_code() {
        let ram = elf(&[(0x1000_0000, &[1; 4]), (0x2000_0000, &[1; 16])]);
        assert!(elf_to_uf2(&ram, FAM, RP2040_FLASH).unwrap_err().to_string().contains("outside flash"));
        let no_boot = elf(&[(0x1000_0100, &[1; 16])]);
        assert!(elf_to_uf2(&no_boot, FAM, RP2040_FLASH).unwrap_err().to_string().contains("flash start"));
        assert!(elf_segments(b"not an elf at all, definitely not").is_err());
    }

    #[test]
    fn validate_detects_corruption() {
        let e = elf(&[(0x1000_0000, &[7; 300])]);
        let good = elf_to_uf2(&e, FAM, RP2040_FLASH).unwrap();
        assert!(validate(&good, FAM, RP2040_FLASH).is_ok());
        assert!(validate(&good, 0x1234_5678, RP2040_FLASH).unwrap_err().to_string().contains("family"));
        let mut bad = good.clone();
        bad[BLOCK - 1] ^= 0xFF;
        assert!(validate(&bad, FAM, RP2040_FLASH).unwrap_err().to_string().contains("end magic"));
        let mut renum = good.clone();
        renum[BLOCK + 20] = 9; // second block's block number
        assert!(validate(&renum, FAM, RP2040_FLASH).unwrap_err().to_string().contains("numbering"));
        assert!(validate(&good[..BLOCK + 10], FAM, RP2040_FLASH).is_err());
    }
}
