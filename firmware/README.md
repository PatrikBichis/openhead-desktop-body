# Firmware for the Uno R4 WiFi

| Sketch | For |
|---|---|
| `servo_setup/` | **First:** find the servos, give them their IDs, configure them and drive them to the middle (2048) before the horns go on. A text menu in a serial terminal |
| `body_uno/` | The body's firmware: a binary packet protocol over USB (460 800 baud) to `openhead-body`, 50 Hz control, watchdog, step limit, emergency stop, the BNO085 |

Both use the same wiring: Uno `Serial1` (pin 0 RX, pin 1 TX) → Waveshare Bus Servo Adapter (A) in mode B, TX/RX crossed, common ground, 7.4 V servo power on the adapter. Check first that the adapter's UART tolerates the Uno's 5 V logic. The BNO085 goes on the Qwiic connector (`Wire1`, address 0x4A; change `IMU_WIRE` / `IMU_ADDR` in `config.h` otherwise).

`feetech.h/.cpp` (the servo bus) and `protocol.h` are in both folders, since Arduino cannot share files between sketches. Change them in `body_uno/` and copy; `protocol.h` is generated into both with `cd python && uv run python -m openhead_body.protocol --header`, and a test checks that the copies match.

The sketches come from the Open Duck Mini bench work, so they still speak the full robot's 14-joint protocol (the desk body uses the neck: IDs 30–33) and their comments and the setup menu are in Swedish.

## Build and upload

Arduino IDE: open `servo_setup/servo_setup.ino` (or `body_uno/body_uno.ino`), pick the board *Arduino UNO R4 WiFi* and the port, upload. Or with arduino-cli:

```bash
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi -u -p COM5 firmware/servo_setup
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi -u -p COM5 firmware/body_uno
```

## Servo setup (`servo_setup`)

Terminal at **115 200 baud, 8N1** (PuTTY, or the Arduino IDE's serial monitor with "Carriage return"). Close it before uploading new firmware.

| Command | Does |
|---|---|
| `scan` | Finds every servo: ID, model, position, voltage, temperature |
| `tabell` | All joint places with their IDs, found and missing |
| `guide` | Label the servos one at a time: pick the place, the servo gets its ID and configuration and drives to the middle |
| `id <old> <new>` | Change an ID (number or joint name, e.g. `id 1 head_yaw`) |
| `konfig <id\|alla>` | Position mode, P 32 / I 0 / D 0, angle limits 0..4095, offset 0 |
| `noll <id\|alla>` | Drive slowly to the middle (2048) and hold |
| `av [id\|alla]` | Torque off |
| `visa` | Positions continuously (Enter stops) |

For the desk body: connect **one servo at a time**, run `guide`, and pick neck pitch (ID 30), head pitch (31), head yaw (32) and head roll (33). New servos all have ID 1, and SO-101 servos often keep LeRobot's limits and offset, which `guide` resets. Then `noll alla` before the horns and the printed parts go on.

The LED matrix on the Uno shows the state: three dots = starting, ring = waiting (torque off), frame = torque on, cross = watchdog or emergency stop, exclamation mark = a servo does not answer.
