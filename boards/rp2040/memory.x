/* RP2040 with 2 MiB QSPI flash (Raspberry Pi Pico): 256-byte second-stage
   bootloader at the start of flash, 264 KiB SRAM (striped banks used as one). */
MEMORY
{
  BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
  FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
  RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}

EXTERN(BOOT2_FIRMWARE)

SECTIONS
{
  .boot2 ORIGIN(BOOT2) :
  {
    KEEP(*(.boot2));
  } > BOOT2
} INSERT BEFORE .text;
