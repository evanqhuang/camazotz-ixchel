# Camazotz-Ixchel

Firmware for a handheld underwater cave mapping device that generates live stick maps as divers navigate submerged cave systems.

## Overview

Camazotz-Ixchel automates traditional cave surveying by continuously measuring distance (via line reel rotation), heading/orientation (IMU), and depth (pressure sensor). The device computes real-time 3D position and renders a stick map on an AMOLED display while logging survey data to SD card for post-dive analysis.

Named after Mayan deities: Camazotz (bat god of caves and darkness) and Ixchel (jaguar goddess of water).

## Hardware

**Platform**
- MCU: RP2350 (Raspberry Pi Pico 2), dual Cortex-M33 @ 150MHz
- Display: Waveshare RP2350-Touch-AMOLED-1.64 (touch AMOLED)
- SDK: Pico SDK 2.2.0
- Languages: C11 / C++17

**Sensors**
- **AS5600** — Magnetic rotary encoder for line reel distance measurement
  - I2C0, GP16(SDA)/GP17(SCL), 400kHz
- **BNO085** — 9-DOF IMU (gyro/accel/mag fusion → quaternion orientation)
  - I2C1, GP2(SDA)/GP3(SCL), RST=GP4, 100kHz
- **MS5837-30BA** — Pressure/temperature sensor for depth measurement
  - PIO I2C (software), GP0(SDA)/GP1(SCL), 100kHz
- **SD Card** — SDIO 4-bit interface for data logging
  - GP18-23

## Architecture

**Dual-Core Design**
- **Core 0**: Display rendering, SD logging, USB serial output
- **Core 1**: 100Hz navigation loop (sensor fusion: encoder + IMU + depth → position)

**Key Characteristics**
- Inter-core communication via spinlock-protected shared memory
- Non-blocking sensor state machines to prevent I2C bus starvation
- Double precision for accumulated position (prevents error accumulation)
- Single precision for per-tick sensor data (optimized for Cortex-M33 FPU)
- Chunked I2C timeouts (100ms max) to avoid USB CDC starvation on RP2350
- PIO-based I2C master for depth sensor (frees hardware I2C buses)

## Building

**Prerequisites**
- Pico SDK 2.2.0 or later
- CMake 3.13+
- ARM GCC toolchain (`arm-none-eabi-gcc`)

**Build Steps**
```bash
mkdir build && cd build
cmake ..
cmake --build . -j$(sysctl -n hw.ncpu)  # macOS
# or
cmake --build . -j$(nproc)              # Linux
```

Output: `build/src/mapper.uf2`

**Flashing**
```bash
# Flash and reboot
picotool load build/src/mapper.uf2
picotool reboot -f

# Force BOOTSEL mode
picotool reboot -f -u
```

## Project Structure

```
├── config.h                     # Pin assignments, frequencies, calibration constants
├── types.h                      # Navigation state (56B), Quat, Vec3, calibration types
├── src/
│   ├── main.cpp                # Entry point, hardware init, boot calibration
│   └── CMakeLists.txt
├── drivers/                    # Hardware abstraction layer (root level)
│   ├── encoder_wrapper.{hpp,cpp}   # AS5600 with magnet detection, zero offset
│   ├── imu_wrapper.{hpp,cpp}       # BNO085 with hardware reset, tare, accuracy
│   ├── depth_wrapper.{hpp,cpp}     # MS5837-30BA non-blocking state machine
│   ├── pio_i2c.{pio,hpp,cpp}       # Custom PIO I2C master with timeouts
├── logic/
│   └── calibration_manager.{hpp,cpp}  # Boot-time calibration orchestrator
├── utils/
│   ├── display_interface.hpp      # Abstract display interface
│   └── stdio_display.{hpp,cpp}    # USB serial display implementation
└── CMakeLists.txt
```

**Include Paths** (via `project_common` INTERFACE target)
- Root directory
- `drivers/`, `logic/`, `utils/`, `libs/`

## Dependencies

External libraries fetched via CMake `FetchContent`:

- [BNO08x_Pico_Library](https://github.com/robotcopper/BNO08x_Pico_Library) — BNO085 driver
  - Patched: I2C error handler disabled (breaks i2c1), timeout extended to 100ms
- [dwm_pico_as5600](https://github.com/dancesWithMachines/dwm_pico_as5600) — AS5600 encoder driver (C library)
- [no-OS-FatFS-SD-SDIO-SPI-RPi-Pico](https://github.com/carlk3/no-OS-FatFS-SD-SDIO-SPI-RPi-Pico) — FatFS for SD card

**Hardware Notes**
- GY-BNO085 board (Teyleten Robot): SD0 pulled HIGH → I2C address **0x4B** (not 0x4A)
- PS0/PS1 must be grounded for I2C mode (no internal pull-downs)
- BNO085 aggressively clock-stretches after ACK — use chunked timeouts with retries

## Compiler Flags

Strict compilation enabled:
- `-Wall -Wextra -Werror -Wshadow -Wcast-align -Wfloat-equal -Wnull-dereference`
- `-fstack-protector-strong` (stack canary protection)

## Development Notes

**I2C Best Practices**
- Always use `i2c_*_timeout_us()` variants (never `*_blocking()`)
- Chunk long operations into 100ms segments (prevents USB CDC starvation)
- PIO I2C timeout implementation required — upstream pico-examples version blocks indefinitely on NAK

**USB Serial**
- Poll `stdio_usb_connected()` at startup to avoid losing early output if host connects late
- Long I2C timeouts (>200ms) starve TinyUSB on RP2350 — serial output halts, picotool hangs

**Calibration**
- BNO085 tare API: `tareNow()`, `saveTare()`, `clearTare()`, `getQuatAccuracy()` (0-3)
- AS5600 magnet status: check MD|ML|MH bits before trusting angle readings
- MS5837 CRC4: validate PROM on init (per AN520), virtual PROM[7]=0 for checksum

## License

All rights reserved.
