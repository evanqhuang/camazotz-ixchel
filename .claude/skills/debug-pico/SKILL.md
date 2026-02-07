---
name: debug-pico
description: Debug a connected Raspberry Pi Pico 2 (RP2350) device. Use when the user mentions debugging, flashing, serial output, crash analysis, device info, memory usage, I2C scanning, SWD, OpenOCD, GDB, breakpoints, registers, watchpoints, or any hardware debugging task for the Pico 2.
argument-hint: [command]
disable-model-invocation: false
allowed-tools: Bash, Read, Grep, Glob, Write, Edit
---

# Pico 2 (RP2350) Debug Skill

You are debugging an RP2350-based embedded device. Two connection methods are available:

1. **USB direct** — USB CDC serial + picotool (BOOTSEL mode flashing)
2. **SWD debug probe** — A second Pico running debugprobe firmware, connected via CMSIS-DAP to OpenOCD for live debugging (halt, step, breakpoints, memory/register read-write, flash-while-running)

Always try SWD first for operations that support it. Fall back to USB/picotool when SWD is unavailable.

## Debug Probe Wiring Reference

The debugprobe Pico connects to the target Pico 2 via 3 wires:
- **SWCLK**: Debugprobe GP2 → Target SWCLK (test pad on bottom of Pico 2)
- **SWDIO**: Debugprobe GP3 → Target SWDIO (test pad on bottom of Pico 2)
- **GND**: Debugprobe GND → Target GND

## Project Context

- **Board**: RP2350 (pico2), Pico SDK 2.2.0
- **Firmware ELF**: `build/src/mapper.elf`
- **Firmware UF2**: `build/src/mapper.uf2`
- **Linker map**: `build/src/mapper.elf.map`
- **Disassembly**: `build/src/mapper.dis`
- **USB CDC**: stdio over USB (UART disabled)

## Tool Paths

- **picotool**: `/opt/homebrew/bin/picotool`
- **arm-none-eabi-objdump**: `/opt/homebrew/bin/arm-none-eabi-objdump`
- **arm-none-eabi-gdb**: `/opt/homebrew/bin/arm-none-eabi-gdb`
- **arm-none-eabi-nm**: `/opt/homebrew/bin/arm-none-eabi-nm`
- **arm-none-eabi-size**: `/opt/homebrew/bin/arm-none-eabi-size`
- **arm-none-eabi-readelf**: `/opt/homebrew/bin/arm-none-eabi-readelf`
- **screen**: `/usr/bin/screen` (for serial terminal)
- **OpenOCD (Pico SDK)**: `~/.pico-sdk/openocd/0.12.0+dev/openocd` (with RP2350 target config)

## OpenOCD Configuration

The Pico SDK bundles OpenOCD 0.12.0+dev with RP2350 support. The standard invocation:

```bash
OPENOCD=~/.pico-sdk/openocd/0.12.0+dev/openocd
OPENOCD_SCRIPTS=~/.pico-sdk/openocd/0.12.0+dev/scripts

# Start OpenOCD server (background)
$OPENOCD -s $OPENOCD_SCRIPTS -f interface/cmsis-dap.cfg -f target/rp2350.cfg
```

This starts:
- **GDB server** on `localhost:3333`
- **Telnet server** on `localhost:4444` (for raw OpenOCD commands)

For one-shot OpenOCD commands (no persistent server):

```bash
$OPENOCD -s $OPENOCD_SCRIPTS -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "init" -c "<command>" -c "shutdown"
```

## Available Commands

Parse `$ARGUMENTS` to determine the action. If no argument is given, run `status`.

---

## USB / Offline Commands

### `status` (default) — Device Status Overview

Run all of these and present a summary:

```bash
# Check for Pico in BOOTSEL mode
picotool info -a 2>&1

# Check for USB CDC serial device
ls /dev/tty.usbmodem* /dev/cu.usbmodem* 2>/dev/null

# Check for debug probe (CMSIS-DAP)
system_profiler SPUSBDataType 2>/dev/null | grep -A5 -i "CMSIS-DAP\|Debugprobe\|Picoprobe"

# Try OpenOCD probe detection (quick, 2s timeout)
timeout 3 ~/.pico-sdk/openocd/0.12.0+dev/openocd \
  -s ~/.pico-sdk/openocd/0.12.0+dev/scripts \
  -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "init" -c "targets" -c "shutdown" 2>&1

# Check if firmware is built
ls -la build/src/mapper.elf build/src/mapper.uf2 2>/dev/null

# Memory usage summary
arm-none-eabi-size build/src/mapper.elf 2>/dev/null
```

Present results as a clear status dashboard:
- Device mode: BOOTSEL / Running / Not detected
- Debug probe: Connected / Not detected
- SWD target state: Halted / Running / Unknown
- Serial port: path or "not found"
- Firmware: last modified time, size
- Memory: text/data/bss breakdown with % of RP2350 limits (520KB SRAM, up to 16MB flash)

### `serial` — Monitor Serial Output

Use Python serial to capture output non-interactively (preferred — `screen`/`stty` methods get stuck):

```bash
python3 -c "
import serial, serial.tools.list_ports, time, sys
ports = [p.device for p in serial.tools.list_ports.comports() if 'usbmodem' in p.device]
if not ports:
    print('No serial device found. Is the Pico connected and running (not in BOOTSEL)?')
    sys.exit(1)
port = ports[0]
print(f'Connecting to {port}...')
ser = serial.Serial(port, 115200, timeout=0.5)
try:
    while True:
        data = ser.read(ser.in_waiting or 1)
        if data:
            sys.stdout.write(data.decode('utf-8', errors='replace'))
            sys.stdout.flush()
except KeyboardInterrupt:
    pass
finally:
    ser.close()
"
```

Run with a timeout for non-interactive capture (e.g. 10 seconds):

```bash
timeout 10 python3 -c "
import serial, serial.tools.list_ports, time, sys
ports = [p.device for p in serial.tools.list_ports.comports() if 'usbmodem' in p.device]
if not ports:
    print('No serial device found.')
    sys.exit(1)
ser = serial.Serial(ports[0], 115200, timeout=0.5)
try:
    while True:
        data = ser.read(ser.in_waiting or 1)
        if data:
            sys.stdout.write(data.decode('utf-8', errors='replace'))
            sys.stdout.flush()
except KeyboardInterrupt:
    pass
finally:
    ser.close()
"
```

### `flash` — Build and Flash Firmware

**If debug probe is connected**, prefer SWD flashing (see `flash-swd` below). It's faster and requires no button presses.

**Via picotool (USB):**

IMPORTANT: Do NOT use `picotool load -x` — it gets stuck. Use the reboot-based method below.

```bash
# 1. Build firmware
cd /Users/evanhuang/camazotz-ixchel/build && cmake .. && cmake --build . -j$(sysctl -n hw.ncpu)

# 2. Force device into BOOTSEL mode
picotool reboot -f -u 2>&1
sleep 2

# 3. Flash the firmware (no -x flag!)
picotool load build/src/mapper.uf2 -f 2>&1

# 4. Reboot into app and capture serial output
picotool reboot -f 2>&1
sleep 1

# 5. Read serial logs with python
python3 -c "
import serial, serial.tools.list_ports, time, sys
# Wait for serial device to enumerate after reboot
for i in range(10):
    ports = [p.device for p in serial.tools.list_ports.comports() if 'usbmodem' in p.device]
    if ports:
        break
    time.sleep(0.5)
if not ports:
    print('No serial device found after reboot.')
    sys.exit(1)
ser = serial.Serial(ports[0], 115200, timeout=0.5)
end = time.time() + 10
try:
    while time.time() < end:
        data = ser.read(ser.in_waiting or 1)
        if data:
            sys.stdout.write(data.decode('utf-8', errors='replace'))
            sys.stdout.flush()
except KeyboardInterrupt:
    pass
finally:
    ser.close()
"
```

If picotool fails with "No accessible RP-series devices":
- Tell the user: "Hold BOOTSEL, press RESET (or replug USB), then release BOOTSEL"
- Or try: `picotool reboot -f -u` to force into BOOTSEL mode from a running device
- Or use `flash-swd` if the debug probe is connected

### `reboot` — Reboot Device

```bash
# Reboot into application mode and capture early serial output
picotool reboot -f 2>&1 && sleep 1 && python3 -c "
import serial, serial.tools.list_ports, time, sys
for i in range(10):
    ports = [p.device for p in serial.tools.list_ports.comports() if 'usbmodem' in p.device]
    if ports:
        break
    time.sleep(0.5)
if not ports:
    print('No serial device found after reboot.')
    sys.exit(1)
ser = serial.Serial(ports[0], 115200, timeout=0.5)
end = time.time() + 10
try:
    while time.time() < end:
        data = ser.read(ser.in_waiting or 1)
        if data:
            sys.stdout.write(data.decode('utf-8', errors='replace'))
            sys.stdout.flush()
except KeyboardInterrupt:
    pass
finally:
    ser.close()
"

# Or force into BOOTSEL mode
picotool reboot -f -u 2>&1
```

### `info` — Detailed Device & Binary Info

```bash
# Device info (if in BOOTSEL mode)
picotool info -a 2>&1

# Binary info from ELF
picotool info -a build/src/mapper.elf 2>&1

# Section sizes
arm-none-eabi-size -A build/src/mapper.elf

# ELF header
arm-none-eabi-readelf -h build/src/mapper.elf
```

### `memory` — Memory Usage Analysis

```bash
# Overall size
arm-none-eabi-size build/src/mapper.elf

# Detailed section breakdown
arm-none-eabi-size -A build/src/mapper.elf

# Top symbols by size (largest first)
arm-none-eabi-nm --size-sort --reverse-sort -S build/src/mapper.elf | head -30

# RAM usage breakdown from map file
grep -A2 "^\.data\|^\.bss\|^\.heap\|^\.stack" build/src/mapper.elf.map | head -20
```

Present memory analysis with:
- Flash usage: .text + .rodata + .data(init) vs available flash
- RAM usage: .data + .bss + heap + stack vs 520KB SRAM
- Top 10 largest symbols with their sizes and sections
- Any concerning allocations (large buffers, oversized stacks)

### `symbols [pattern]` — Search Symbols

Search for symbols in the firmware binary. Use the pattern from the argument.

```bash
# List all symbols matching pattern
arm-none-eabi-nm -C build/src/mapper.elf | grep -i "<pattern>"

# Or with demangled C++ names and sizes
arm-none-eabi-nm -C --size-sort -S build/src/mapper.elf | grep -i "<pattern>"
```

### `disasm [function]` — Disassemble a Function

```bash
# Disassemble specific function
arm-none-eabi-objdump -d -C --no-show-raw-insn build/src/mapper.elf | grep -A 50 "<function_name>:"

# Or use the pre-generated disassembly
grep -A 50 "<function_name>:" build/src/mapper.dis
```

### `crash [address]` — Analyze Crash / HardFault

When the user provides a crash address or HardFault dump from serial output:

1. **Resolve address to source location**:
```bash
arm-none-eabi-addr2line -e build/src/mapper.elf -f -C <address>
```

2. **Show surrounding disassembly**:
```bash
arm-none-eabi-objdump -d -C --start-address=<addr-0x20> --stop-address=<addr+0x20> build/src/mapper.elf
```

3. **Check if address is in flash, RAM, or peripheral space**:
   - `0x10000000-0x1FFFFFFF`: XIP flash (code/rodata)
   - `0x20000000-0x20081FFF`: SRAM (520KB)
   - `0x40000000-0x40FFFFFF`: APB peripherals
   - `0x50000000-0x50FFFFFF`: AHB peripherals
   - `0xD0000000-0xD0FFFFFF`: SIO (core-local)
   - `0xE0000000-0xE00FFFFF`: Cortex-M33 PPB (NVIC, SysTick, etc.)

4. **For HardFault analysis**, guide the user to add this to their firmware if not present:
   - Read SCB->CFSR (0xE000ED28) for fault status flags
   - Read SCB->HFSR (0xE000ED2C) for HardFault status
   - Read SCB->MMFAR (0xE000ED34) for MemManage fault address
   - Read SCB->BFAR (0xE000ED38) for BusFault address
   - Decode the stacked registers (R0-R3, R12, LR, PC, xPSR)

### `i2c` — I2C Bus Diagnostics

Interpret I2C diagnostics from serial output. If the debug probe is connected, also use `swd-scan` for live peripheral register reads. Provide reference info:

**Expected I2C devices on this project:**
| Bus | Address | Device | Pins |
|------|---------|--------|------|
| I2C0 | 0x36 | AS5600 encoder | SDA=GP16, SCL=GP17 |
| I2C1 | 0x4A | BNO08x IMU | SDA=GP2, SCL=GP3, RST=GP4 |
| PIO1 | 0x76 | MS5837-30BA depth | SDA=GP0, SCL=GP1 |

If the user has I2C scan output, help interpret:
- Missing device → check wiring, pull-ups (2.2k-4.7k to 3.3V), power
- Multiple unexpected addresses → bus contention or short
- Device present but communication fails → check clock speed, try lower freq

### `map [section]` — Analyze Linker Map

```bash
# Read the map file for specific sections or symbols
# The map file is at build/src/mapper.elf.map
```

Read the map file and present:
- Section placement and sizes
- Memory region usage (flash vs SRAM)
- Cross-reference tables for specific symbols if requested

### `build` — Build Only (No Flash)

```bash
cd build && cmake .. && cmake --build . -j$(sysctl -n hw.ncpu) 2>&1
```

Report build success/failure. On failure, analyze the error messages and suggest fixes.

### `clean` — Clean Build

```bash
cd build && cmake --build . --target clean 2>&1
```

### `log [duration]` — Capture Serial Log

Non-interactive serial capture for a specified duration (default 10 seconds). Uses python serial (stty/cat methods get stuck):

```bash
python3 -c "
import serial, serial.tools.list_ports, time, sys
duration = int(sys.argv[1]) if len(sys.argv) > 1 else 10
ports = [p.device for p in serial.tools.list_ports.comports() if 'usbmodem' in p.device]
if not ports:
    print('No serial device found. Is the Pico connected and running (not in BOOTSEL)?')
    sys.exit(1)
port = ports[0]
print(f'Capturing from {port} for {duration}s...')
ser = serial.Serial(port, 115200, timeout=0.5)
end = time.time() + duration
try:
    while time.time() < end:
        data = ser.read(ser.in_waiting or 1)
        if data:
            sys.stdout.write(data.decode('utf-8', errors='replace'))
            sys.stdout.flush()
except KeyboardInterrupt:
    pass
finally:
    ser.close()
" ${DURATION:-10}
```

After capture, analyze the output for:
- Error messages or fault indicators
- I2C communication failures
- Sensor initialization status
- Calibration progress/results
- Timing anomalies

---

## SWD / OpenOCD / GDB Commands

These commands require the debugprobe to be connected. If OpenOCD fails to connect, tell the user to check the wiring and that the debugprobe is powered.

### `openocd` — Start OpenOCD Server

Start the OpenOCD server in the background:

```bash
~/.pico-sdk/openocd/0.12.0+dev/openocd \
  -s ~/.pico-sdk/openocd/0.12.0+dev/scripts \
  -f interface/cmsis-dap.cfg \
  -f target/rp2350.cfg
```

Run this in background. It must stay running for GDB and other SWD commands. Tell the user:
- GDB server available on `localhost:3333`
- Telnet interface on `localhost:4444`
- Stop with Ctrl-C or kill the background process

If it fails with "no device found", the debugprobe is not connected or not recognized.
If it fails with "Error connecting DP", the SWD wires may be loose or incorrect.

### `gdb [function]` — Start GDB Debug Session

Provide the GDB command to run. This is interactive so the user runs it themselves:

```bash
arm-none-eabi-gdb build/src/mapper.elf \
  -ex "target extended-remote :3333" \
  -ex "monitor reset init" \
  -ex "load" \
  -ex "break main" \
  -ex "continue"
```

If a function name is given as argument, set a breakpoint there instead of main:

```bash
arm-none-eabi-gdb build/src/mapper.elf \
  -ex "target extended-remote :3333" \
  -ex "monitor reset init" \
  -ex "load" \
  -ex "break <function>" \
  -ex "continue"
```

**Essential GDB commands for the user:**
| Command | Action |
|---------|--------|
| `n` / `next` | Step over |
| `s` / `step` | Step into |
| `c` / `continue` | Resume execution |
| `bt` / `backtrace` | Stack backtrace |
| `info reg` | Show all registers |
| `p <expr>` | Print expression/variable |
| `x/16xw <addr>` | Examine 16 words at address |
| `monitor halt` | Halt the target |
| `monitor resume` | Resume the target |
| `monitor reset init` | Reset and halt at boot |
| `break <loc>` | Set breakpoint |
| `watch <expr>` | Set watchpoint (hardware) |
| `info break` | List breakpoints |
| `delete <n>` | Delete breakpoint |
| `layout src` | TUI source view |
| `layout split` | TUI source + asm view |
| `thread info` | Show cores (SMP) |
| `thread 1` / `thread 2` | Switch cores |

### `halt` — Halt Target via SWD

```bash
~/.pico-sdk/openocd/0.12.0+dev/openocd \
  -s ~/.pico-sdk/openocd/0.12.0+dev/scripts \
  -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "init" -c "halt" -c "shutdown"
```

### `resume` — Resume Target via SWD

```bash
~/.pico-sdk/openocd/0.12.0+dev/openocd \
  -s ~/.pico-sdk/openocd/0.12.0+dev/scripts \
  -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "init" -c "resume" -c "shutdown"
```

### `reset` — Reset Target via SWD

```bash
# Reset and run
~/.pico-sdk/openocd/0.12.0+dev/openocd \
  -s ~/.pico-sdk/openocd/0.12.0+dev/scripts \
  -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "init" -c "reset run" -c "shutdown"
```

```bash
# Reset and halt (for debugging from boot)
~/.pico-sdk/openocd/0.12.0+dev/openocd \
  -s ~/.pico-sdk/openocd/0.12.0+dev/scripts \
  -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "init" -c "reset halt" -c "shutdown"
```

### `reg [name]` — Read Registers via SWD

Read all general-purpose registers or a specific one:

```bash
# Read all registers (target must be halted)
~/.pico-sdk/openocd/0.12.0+dev/openocd \
  -s ~/.pico-sdk/openocd/0.12.0+dev/scripts \
  -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "init" -c "halt" -c "reg" -c "shutdown"
```

```bash
# Read specific register
~/.pico-sdk/openocd/0.12.0+dev/openocd \
  -s ~/.pico-sdk/openocd/0.12.0+dev/scripts \
  -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "init" -c "halt" -c "reg <name>" -c "shutdown"
```

Key Cortex-M33 registers: `pc`, `sp`, `lr`, `r0`-`r12`, `xpsr`, `msp`, `psp`, `primask`, `basepri`, `faultmask`, `control`.

After reading registers, cross-reference `pc` with `arm-none-eabi-addr2line` to determine where the CPU was executing.

### `read-mem <address> [count]` — Read Memory via SWD

```bash
# Read 'count' 32-bit words starting at 'address' (default: 16 words)
~/.pico-sdk/openocd/0.12.0+dev/openocd \
  -s ~/.pico-sdk/openocd/0.12.0+dev/scripts \
  -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "init" -c "halt" -c "mdw <address> <count>" -c "shutdown"
```

For byte reads use `mdb`, for halfword use `mdh`.

Present the memory dump with address labels and, where possible, cross-reference with known peripheral registers or symbols from the ELF.

### `write-mem <address> <value>` — Write Memory via SWD

```bash
~/.pico-sdk/openocd/0.12.0+dev/openocd \
  -s ~/.pico-sdk/openocd/0.12.0+dev/scripts \
  -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "init" -c "halt" -c "mww <address> <value>" -c "resume" -c "shutdown"
```

CAUTION: Writing to wrong addresses can brick the device or corrupt flash. Always confirm the address with the user before writing.

### `flash-swd` — Flash via SWD (No BOOTSEL Needed)

This is the preferred flashing method when the debug probe is connected. No need for BOOTSEL mode.

```bash
# Build first
cd build && cmake .. && cmake --build . -j$(sysctl -n hw.ncpu)

# Flash via SWD and reset
~/.pico-sdk/openocd/0.12.0+dev/openocd \
  -s ~/.pico-sdk/openocd/0.12.0+dev/scripts \
  -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "adapter speed 5000" \
  -c "program build/src/mapper.elf verify reset exit"
```

The `program` command erases, writes, verifies, resets, and exits in one step.

### `flash-swd-halt` — Flash via SWD and Halt at Reset

Useful for debugging from the very first instruction:

```bash
cd build && cmake .. && cmake --build . -j$(sysctl -n hw.ncpu)

~/.pico-sdk/openocd/0.12.0+dev/openocd \
  -s ~/.pico-sdk/openocd/0.12.0+dev/scripts \
  -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "adapter speed 5000" \
  -c "program build/src/mapper.elf verify" \
  -c "reset halt" -c "shutdown"
```

Then connect GDB to continue from halted state.

### `backtrace` — Get Stack Backtrace via SWD

Uses GDB in batch mode to get a backtrace without an interactive session:

```bash
arm-none-eabi-gdb build/src/mapper.elf --batch \
  -ex "target extended-remote :3333" \
  -ex "monitor halt" \
  -ex "bt full" \
  -ex "info threads" \
  -ex "thread 2" \
  -ex "bt full" \
  -ex "detach" \
  -ex "quit"
```

Requires OpenOCD to be running (`/debug-pico openocd` first). Reports backtraces for both cores.

After getting the backtrace, read the relevant source files to provide context about what each frame is doing.

### `breakpoint <location>` — Set Breakpoint via GDB Batch

```bash
arm-none-eabi-gdb build/src/mapper.elf --batch \
  -ex "target extended-remote :3333" \
  -ex "monitor reset halt" \
  -ex "break <location>" \
  -ex "continue" \
  -ex "bt" \
  -ex "info reg" \
  -ex "detach" \
  -ex "quit"
```

Location can be: function name, `file.cpp:line`, or raw address `*0x10001234`.

Note: RP2350 Cortex-M33 supports 8 hardware breakpoints. Exceeding this requires software breakpoints (flash patching), which OpenOCD handles automatically.

### `watchpoint <expression>` — Set Data Watchpoint

Hardware watchpoints trigger when a memory location is read or written:

```bash
arm-none-eabi-gdb build/src/mapper.elf --batch \
  -ex "target extended-remote :3333" \
  -ex "monitor halt" \
  -ex "watch <expression>" \
  -ex "continue" \
  -ex "bt" \
  -ex "info reg" \
  -ex "detach" \
  -ex "quit"
```

Cortex-M33 supports 4 hardware watchpoints. Useful for catching:
- Buffer overflows (watch end of array)
- Unexpected variable modifications
- Peripheral register access patterns

### `fault` — Read Fault Registers Live via SWD

Read all Cortex-M33 fault status registers directly from hardware:

```bash
~/.pico-sdk/openocd/0.12.0+dev/openocd \
  -s ~/.pico-sdk/openocd/0.12.0+dev/scripts \
  -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "init" -c "halt" \
  -c "echo '=== Fault Status Registers ==='" \
  -c "mdw 0xE000ED28 1" \
  -c "mdw 0xE000ED2C 1" \
  -c "mdw 0xE000ED30 1" \
  -c "mdw 0xE000ED34 1" \
  -c "mdw 0xE000ED38 1" \
  -c "echo '=== Core Registers ==='" \
  -c "reg pc" -c "reg sp" -c "reg lr" -c "reg xpsr" \
  -c "shutdown"
```

Decode the results:

**CFSR (0xE000ED28)** — Configurable Fault Status Register:
- Bits [7:0] = MMFSR (MemManage): MMARVALID, MSTKERR, MUNSTKERR, DACCVIOL, IACCVIOL
- Bits [15:8] = BFSR (BusFault): BFARVALID, LSPERR, STKERR, UNSTKERR, IMPRECISERR, PRECISERR, IBUSERR
- Bits [31:16] = UFSR (UsageFault): DIVBYZERO, UNALIGNED, NOCP, INVPC, INVSTATE, UNDEFINSTR

**HFSR (0xE000ED2C)** — HardFault Status:
- Bit 1 = VECTTBL (vector table read fault)
- Bit 30 = FORCED (escalated from configurable fault)
- Bit 31 = DEBUGEVT (debug event)

**DFSR (0xE000ED30)** — Debug Fault Status:
- EXTERNAL, VCATCH, DWTTRAP, BKPT, HALTED

**MMFAR (0xE000ED34)** — MemManage Fault Address (valid if MMARVALID set)
**BFAR (0xE000ED38)** — BusFault Address (valid if BFARVALID set)

### `peripheral <name>` — Read Peripheral Registers

Read key peripheral registers relevant to this project. Supported names:

**`i2c0`** — I2C0 registers (AS5600 encoder bus):
```bash
~/.pico-sdk/openocd/0.12.0+dev/openocd \
  -s ~/.pico-sdk/openocd/0.12.0+dev/scripts \
  -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "init" -c "halt" \
  -c "echo '=== I2C0 Registers ==='" \
  -c "mdw 0x40090000 16" \
  -c "shutdown"
```

**`i2c1`** — I2C1 registers (BNO08x IMU bus):
```bash
# Base: 0x40098000
-c "mdw 0x40098000 16"
```

**`pio1`** — PIO1 registers (depth sensor I2C):
```bash
# Base: 0x50300000
-c "mdw 0x50300000 16"
```

**`gpio`** — GPIO status for project pins:
```bash
# Read IO_BANK0 GPIO status registers for relevant pins
# Each GPIO has a status reg at 0x40028000 + pin*8
~/.pico-sdk/openocd/0.12.0+dev/openocd \
  -s ~/.pico-sdk/openocd/0.12.0+dev/scripts \
  -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "init" -c "halt" \
  -c "echo '=== GPIO Pin States ==='" \
  -c "echo 'GP0 (Depth SDA):'" -c "mdw 0x40028000 2" \
  -c "echo 'GP1 (Depth SCL):'" -c "mdw 0x40028008 2" \
  -c "echo 'GP2 (IMU SDA):'"   -c "mdw 0x40028010 2" \
  -c "echo 'GP3 (IMU SCL):'"   -c "mdw 0x40028018 2" \
  -c "echo 'GP4 (IMU RST):'"   -c "mdw 0x40028020 2" \
  -c "echo 'GP5 (Enc SCL):'"   -c "mdw 0x40028028 2" \
  -c "echo 'GP16 (Enc SDA):'"  -c "mdw 0x40028080 2" \
  -c "shutdown"
```

**`nvic`** — NVIC interrupt status:
```bash
# NVIC_ISER (enabled interrupts): 0xE000E100
# NVIC_ISPR (pending interrupts): 0xE000E200
# NVIC_IABR (active interrupts): 0xE000E300
~/.pico-sdk/openocd/0.12.0+dev/openocd \
  -s ~/.pico-sdk/openocd/0.12.0+dev/scripts \
  -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "init" -c "halt" \
  -c "echo '=== NVIC Status ==='" \
  -c "echo 'Enabled:'" -c "mdw 0xE000E100 2" \
  -c "echo 'Pending:'" -c "mdw 0xE000E200 2" \
  -c "echo 'Active:'"  -c "mdw 0xE000E300 2" \
  -c "shutdown"
```

**`systick`** — SysTick timer:
```bash
# SysTick: 0xE000E010
-c "mdw 0xE000E010 4"
```

### `swd-scan` — I2C Bus Scan via SWD

Scan I2C buses by reading their status registers and probing addresses. More reliable than serial-based scanning because it doesn't require firmware cooperation:

```bash
# This requires the firmware to NOT be actively using the I2C bus,
# so halt first, read the I2C peripheral enable/status registers
~/.pico-sdk/openocd/0.12.0+dev/openocd \
  -s ~/.pico-sdk/openocd/0.12.0+dev/scripts \
  -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "init" -c "halt" \
  -c "echo '=== I2C0 Status (Encoder) ==='" \
  -c "mdw 0x40090000 16" \
  -c "echo '=== I2C1 Status (IMU) ==='" \
  -c "mdw 0x40098000 16" \
  -c "echo '=== PIO1 Status (Depth) ==='" \
  -c "mdw 0x50300000 8" \
  -c "resume" -c "shutdown"
```

Interpret I2C_STATUS register bits and present results as a table.

### `core [0|1]` — Inspect Specific Core

The RP2350 has dual Cortex-M33 cores. Inspect a specific core:

```bash
arm-none-eabi-gdb build/src/mapper.elf --batch \
  -ex "target extended-remote :3333" \
  -ex "monitor halt" \
  -ex "info threads" \
  -ex "thread <core+1>" \
  -ex "bt full" \
  -ex "info reg" \
  -ex "detach" \
  -ex "quit"
```

In GDB, core 0 = thread 1, core 1 = thread 2 (1-indexed).

### `erase` — Erase Flash via SWD

```bash
~/.pico-sdk/openocd/0.12.0+dev/openocd \
  -s ~/.pico-sdk/openocd/0.12.0+dev/scripts \
  -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "init" -c "halt" \
  -c "flash erase_sector 0 0 last" \
  -c "shutdown"
```

CAUTION: This erases all firmware from the target. Confirm with user before executing.

### `verify` — Verify Flashed Firmware Matches ELF

```bash
~/.pico-sdk/openocd/0.12.0+dev/openocd \
  -s ~/.pico-sdk/openocd/0.12.0+dev/scripts \
  -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "init" -c "halt" \
  -c "verify_image build/src/mapper.elf" \
  -c "shutdown"
```

### `rescue` — Rescue a Bricked Device via SWD

If the device is stuck (bad firmware, infinite loop in boot, locked up):

```bash
~/.pico-sdk/openocd/0.12.0+dev/openocd \
  -s ~/.pico-sdk/openocd/0.12.0+dev/scripts \
  -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
  -c "init" -c "halt" \
  -c "flash erase_sector 0 0 last" \
  -c "reset run" -c "shutdown"
```

This erases the flash entirely, allowing the device to boot into BOOTSEL-like state. The debug probe can always reach a bricked device via SWD even when USB is unresponsive.

---

## General Guidelines

1. **Always check device state first** before attempting operations. A device in BOOTSEL mode has no serial, and a running device can't be flashed via picotool without rebooting. SWD can flash anytime.

2. **Prefer SWD flashing** (`flash-swd`) over BOOTSEL flashing when the debug probe is connected. It's faster and doesn't require physical button presses.

3. **Be explicit about physical actions** the user needs to take (pressing BOOTSEL, replugging USB, checking SWD wiring).

4. **Cross-reference addresses** with the ELF when analyzing crashes or unexpected behavior.

5. **Use the map file** (`build/src/mapper.elf.map`) for detailed memory layout analysis -- it has the most complete information about symbol placement.

6. **For serial parsing**, look for patterns from this firmware's output format (calibration manager messages, sensor status prints, error codes).

7. **Build before flash** -- always rebuild to ensure the latest code is flashed.

8. When analyzing binary output or crash dumps, always use `arm-none-eabi-addr2line` and `arm-none-eabi-objdump` with the `-C` flag to demangle C++ symbol names.

9. **OpenOCD must be running** for GDB-based commands (`backtrace`, `breakpoint`, `watchpoint`, `core`). For one-shot commands (`halt`, `reset`, `reg`, `read-mem`, `fault`, `peripheral`), OpenOCD is invoked inline and exits automatically.

10. **Halt before reading** -- most SWD memory/register reads require the target to be halted first. The one-shot commands include `halt` automatically. Remember to `resume` if the user wants the target to continue running.

11. **RP2350 hardware debug limits**: 8 hardware breakpoints, 4 hardware watchpoints. OpenOCD will use flash patching for software breakpoints beyond this limit.
