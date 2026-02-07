---
name: flash-pico
description: Build, flash, and monitor a Raspberry Pi Pico 2 (RP2350). Use when the user wants to flash firmware, view serial logs, build and deploy, or monitor device output. Keywords: flash, deploy, upload, serial, logs, monitor, output, build and run.
argument-hint: [build|flash|log|reboot|watch]
disable-model-invocation: false
allowed-tools: Bash, Read, Grep, Glob
---

# Pico 2 Flash & Log Skill

Fast build-flash-log workflow for the RP2350. Uses `picotool reboot` + python serial (avoids stuck `screen`/`stty`/`picotool load -x` methods).

## Project Context

- **Board**: RP2350 (pico2), Pico SDK 2.2.0
- **Firmware UF2**: `build/src/mapper.uf2`
- **Firmware ELF**: `build/src/mapper.elf`
- **USB CDC**: stdio over USB at 115200 baud
- **picotool**: `/opt/homebrew/bin/picotool`

## Python Serial Helper

All serial reads use python `serial` (pyserial). This is reliable and non-blocking unlike `screen`/`stty`/`cat` which get stuck on macOS with Pico USB CDC.

## Available Commands

Parse `$ARGUMENTS` to determine the action. If no argument is given, run `flash` (build + flash + log).

---

### `flash` (default) — Build, Flash, and Show Logs

Full pipeline: build firmware, flash to device, reboot, and capture boot serial output.

```bash
# Step 1: Build
cd /Users/evanhuang/camazotz-ixchel/build && cmake .. && cmake --build . -j$(sysctl -n hw.ncpu) 2>&1
```

If build fails, report the error and stop. Do NOT flash stale firmware.

```bash
# Step 2: Force into BOOTSEL mode
picotool reboot -f -u 2>&1
sleep 2

# Step 3: Flash (no -x flag — it gets stuck!)
picotool load /Users/evanhuang/camazotz-ixchel/build/src/mapper.uf2 -f 2>&1

# Step 4: Reboot into app + capture serial
picotool reboot -f 2>&1
sleep 1

python3 -c "
import serial, serial.tools.list_ports, time, sys
for i in range(15):
    ports = [p.device for p in serial.tools.list_ports.comports() if 'usbmodem' in p.device]
    if ports:
        break
    time.sleep(0.5)
if not ports:
    print('No serial device found after reboot. Device may still be booting.')
    sys.exit(1)
port = ports[0]
print(f'Connected to {port}')
print('--- Serial Output ---')
ser = serial.Serial(port, 115200, timeout=0.5)
end = time.time() + 15
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
    print('\n--- End Serial Output ---')
"
```

If `picotool reboot -f -u` fails with "No accessible RP-series devices":
- The device may already be in BOOTSEL — try `picotool load` directly
- Or tell the user: "Hold BOOTSEL, press RESET (or replug USB), then release BOOTSEL"

After capturing serial output, analyze it for:
- Sensor initialization success/failure
- I2C communication errors
- Calibration progress
- Any crash or fault indicators

### `build` — Build Only (No Flash)

```bash
cd /Users/evanhuang/camazotz-ixchel/build && cmake .. && cmake --build . -j$(sysctl -n hw.ncpu) 2>&1
```

Report build success/failure with firmware size:
```bash
arm-none-eabi-size /Users/evanhuang/camazotz-ixchel/build/src/mapper.elf 2>/dev/null
```

### `log [duration]` — Capture Serial Logs

Capture serial output for a specified duration (default: 15 seconds). Device must already be running (not in BOOTSEL).

```bash
python3 -c "
import serial, serial.tools.list_ports, time, sys
duration = int(sys.argv[1]) if len(sys.argv) > 1 else 15
ports = [p.device for p in serial.tools.list_ports.comports() if 'usbmodem' in p.device]
if not ports:
    print('No serial device found. Is the Pico connected and running?')
    sys.exit(1)
port = ports[0]
print(f'Capturing from {port} for {duration}s...')
print('--- Serial Output ---')
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
    print('\n--- End Serial Output ---')
" ${DURATION:-15}
```

After capture, analyze the output for errors, sensor status, calibration progress, and timing issues.

### `reboot` — Reboot and Watch

Reboot the running device and immediately capture serial output from boot:

```bash
picotool reboot -f 2>&1
sleep 1

python3 -c "
import serial, serial.tools.list_ports, time, sys
for i in range(15):
    ports = [p.device for p in serial.tools.list_ports.comports() if 'usbmodem' in p.device]
    if ports:
        break
    time.sleep(0.5)
if not ports:
    print('No serial device found after reboot.')
    sys.exit(1)
port = ports[0]
print(f'Connected to {port}')
print('--- Serial Output (boot) ---')
ser = serial.Serial(port, 115200, timeout=0.5)
end = time.time() + 15
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
    print('\n--- End Serial Output ---')
"
```

### `watch` — Continuous Serial Monitor

Watch serial output indefinitely (until Ctrl-C / timeout). Use for long-running monitoring:

```bash
timeout 60 python3 -c "
import serial, serial.tools.list_ports, time, sys
ports = [p.device for p in serial.tools.list_ports.comports() if 'usbmodem' in p.device]
if not ports:
    print('No serial device found.')
    sys.exit(1)
port = ports[0]
print(f'Watching {port} (Ctrl-C to stop)...')
print('--- Serial Output ---')
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
    print('\n--- End Serial Output ---')
"
```

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "No accessible RP-series devices" | Hold BOOTSEL + press RESET, or replug USB while holding BOOTSEL |
| Serial device not found after flash | Wait longer (increase sleep), or check `ls /dev/tty.usbmodem*` |
| `picotool load` fails | Device may not be in BOOTSEL — run `picotool reboot -f -u` first |
| Serial output garbled | Check baud rate is 115200, check USB cable supports data (not charge-only) |
| Device stuck / no USB | Use debug probe SWD to rescue: `/debug-pico rescue` |
