# Hexapod USB-HS — Python Codebase Overview

## What This Is

6-DOF hexapod robot control system. Python GUI sends commands over USB serial to a Teensy 4.1 microcontroller. Telemetry streams back and gets logged to binary files for post-hoc analysis.

---

## Architecture

```
main.py  (process orchestrator)
├── AppProcess
│   └── ui.py          (CustomTkinter GUI + USB hotplug detection)
│       └── Pipe ──────────────────────────────────┐
└── SerialServerProcess                            │
    └── serial_server.py  (IPC request handler)  ◄─┘
        └── hexapod.py    (hardware abstraction layer)
            ├── hexlink.py     (packet encoder)
            └── constants.py   (protocol definitions)

decode_log.py  (standalone post-hoc analysis tool — separate entry point)
```

**Two processes, one pipe.** The GUI and serial I/O live in separate OS processes so hardware blocking never freezes the UI. All communication between them is a bidirectional `multiprocessing.Pipe`.

---

## File-by-File

### [constants.py](constants.py)
Protocol constants. No logic.

- `START_BYTE = 0xFE`, `END_BYTE = 0xFF` — frame delimiters
- `PACKET_SIZE = 64` — all packets are exactly 64 bytes
- `MASTER_ID = 0xFE` (device), `PC_ID = 0xFF` (PC)
- Message IDs `0x01`–`0x11` and `0xFF`:

| ID | Name | Purpose |
|----|------|---------|
| 0x01 | PING | Heartbeat request |
| 0x02 | SYNC | TIMESYNC |
| 0x03 | ENABLE | Arm motors |
| 0x04 | DISABLE | Disarm motors |
| 0x05 | UPLOAD | Trajectory point |
| 0x06 | PLAY | Start playback |
| 0x07 | PAUSE | Pause playback |
| 0x08 | STOP | Stop playback |
| 0x09 | ESTOP | Emergency stop |
| 0x0A | RESET | Full reset |
| 0x0B | JOG | Manual move (6 floats) |
| 0x0C | VALIDATE | Verify uploaded trajectory |
| 0x0E | CALIBRATE | Run calibration routine |
| 0x0F | STAGE | Move to stage position |
| 0x10 | PARK | Move to park position |
| 0x11 | INFO | Device info message |
| 0xFF | STATUS | Telemetry packet from device |

---

### [hexlink.py](hexlink.py)
Packet serializer. Builds 64-byte binary frames.

**Packet layout:**
```
Byte  0      START_BYTE  (0xFE)
Byte  1      FROM_ID     (PC_ID = 0xFF)
Byte  2      TO_ID       (MASTER_ID = 0xFE)
Byte  3      SEQ         (0–255, auto-increments per instance)
Byte  4      MSGID
Bytes 5–60   PAYLOAD     (56 bytes, usage varies by MSGID)
Bytes 61–62  CRC16-XMODEM (big-endian, over bytes 0–60)
Byte  63     END_BYTE    (0xFF)
```

**CRC:** CRC16-XMODEM (polynomial `0x1021`, init `0x0000`) via `fastcrc.crc16.xmodem`. Validation uses the residue trick: CRC over the full 63 bytes (including stored CRC) must equal 0.

**Key methods on `Packet`:**

| Method | Payload content |
|--------|----------------|
| `ping()`, `pong()`, `enable()`, `disable()`, `calibrate()`, `stage()`, `park()` | Empty (zeros) |
| `jog(pose)` | 6 × float32 at bytes 5–28 |
| `upload(trajectory)` | Returns list of N packets; one per row of the Nx6 float32 array |
| `validate_trajectory(crc32, length)` | `length` at bytes 5–8, `crc32` at bytes 9–12 |

---

### [hexapod.py](hexapod.py)
Hardware abstraction layer. Owns the serial port and listener thread.

**`TelemetryRecorder`** — writes raw 64-byte packets to `output/telem_YYYYMMDD_HHMMSS.bin` during playback.

**Telemetry parsing** — STATUS packets (`MSGID_STATUS = 0xFF`) carry 51-byte payload:
```
Bytes 5–8    timestamp (uint32, microseconds)
Byte  9      globalFlags
Bytes 10–32  left axis (23 bytes)
Bytes 33–55  right axis (23 bytes)
```

Each 23-byte axis block (`<BBffffbHBB`):
- `mode` (byte), `flags` (byte)
- `setpoint`, `theta`, `omega`, `tau` (float32 each)
- `temp` (int8, °C), `rtt` (uint16, µs), `txErrors`, `timeouts` (bytes)

Flag bits in the `flags` byte:
```
bit 0  armed
bit 1  calibrated
bit 2  calibrating
bit 3  hasError
bit 4  awaitingResponse
bits 5-7  calibPhase (0=IDLE, 1=SEEK_SENSOR, 2=FIND_EDGE_1,
                       3=REVERSE_TO_SENSOR, 4=FIND_EDGE_2,
                       5=MOVE_TO_CENTER, 6=COMPLETE)
```

**`Hexapod` class** — main driver:

- `connect()` — opens `serial.Serial`, spawns listener daemon thread
- `run()` — listener thread: reads bytes → `byteBuffer` → `_consume_packets()`
- `_consume_packets()` — scans buffer for valid 64-byte frames (START/END + CRC check)
- `process_packet(pkt)` — dispatches by MSGID; logs telemetry; writes to recorder if active
- `sendData(data)` — sends bytes in 512-byte chunks
- `upload(filename)` — loads CSV (Nx6 float32), calls `Packet.upload()`, sends all packets, then sends a `validate_trajectory` packet with CRC32 of the raw payload

---

### [serial_server.py](serial_server.py)
IPC bridge. Runs in its own process. Receives dict-based requests from the GUI via pipe, calls the corresponding `Hexapod` method.

**Request format:** `{"KEY": value}` — value is `None` for simple commands or a string/array for parameterized ones.

| Key | Action |
|-----|--------|
| `PORT` | `hexapod.set_port(value)` |
| `CONNECT` / `DISCONNECT` | `hexapod.connect()` / `disconnect()` |
| `ENABLE` / `DISABLE` | `hexapod.enable()` / `disable()` |
| `CALIBRATE` / `STAGE` / `PARK` | device state commands |
| `UPLOAD` | `hexapod.upload(path)` |
| `VALIDATE` | `hexapod.validate_trajectory()` |
| `PLAY` / `PAUSE` / `STOP` / `ESTOP` / `RESET` | playback & safety commands |
| `SEND` | `hexapod.move(value)` — 6-element float array for JOG |
| `None` | graceful shutdown |

Exceptions per-request are caught and logged; server keeps running.

---

### [ui.py](ui.py)
CustomTkinter GUI, 800×480 px.

**Layout (rows):**
```
Row 0   Port dropdown | CONNECT | DISCONNECT
Row 1   ENABLE (2 cols) | DISABLE (2 cols)
Row 2   UPLOAD (2 cols) | CALIBRATE (2 cols)
Row 3   STAGE  (2 cols) | PARK     (2 cols)
Row 4   PLAY / PAUSE / STOP  (segmented button, full width)
Row 5-6 ESTOP (3 cols)  | RESET (1 col)
Row 7   Angle input (degrees) + SEND
```

**USB hotplug:** `usbx` monitors for Teensy 4.1 (VID `0x16C0`, PID `0x0483`). Connect/disconnect callbacks update the port dropdown in real time via `after()` (thread-safe).

**SEND flow:** Input string → parse float → clamp to ±60° → convert to radians → broadcast same value to all 6 axes → send `{"SEND": [rad]*6}` to server.

**UPLOAD flow:** File dialog restricted to `Trajectories/` folder → send `{"UPLOAD": path}` to server.

**Response listener:** Daemon thread blocks on `conn.recv()`. Receives `None` on shutdown. Fires `<<ReceivedResponse>>` virtual event for future UI updates.

---

### [main.py](main.py)
Entry point. Wires everything together.

```python
parent_conn, child_conn = Pipe(duplex=True)

AppProcess(target=run_app, args=(parent_conn,))           # GUI
SerialServerProcess(target=run_serial_server, args=(child_conn,))  # hardware
```

Both spawned as daemon processes. Main thread joins `AppProcess` (blocks until GUI closes), then sends `None` to the server for graceful shutdown (3-second timeout, force-terminates if stuck).

**Logging:** `RotatingFileHandler` (2 MB, 5 backups) per process:
- `logs/main.log`
- `logs/app.log`
- `logs/serial_server_process.log`

---

### [decode_log.py](decode_log.py)
Standalone analysis tool. Launched separately (e.g., `decoderPy.bat`).

**Input:** `output/*.bin` binary telemetry files.

**Parsing:** Scans file for 64-byte frames. Extracts only STATUS (`0xFF`) and JOG (`0x0B`) packets. From-ID determines axis grouping: each slave (IDs 1–3) owns 2 axes (1+2, 3+4, 5+6).

**Output:**
- CustomTkinter tabbed GUI — one tab per axis, plots `theta` and JOG setpoint vs time
- Export to `.xlsx` — one sheet per axis + one JOG sheet

---

## Data Flow Summary

### Connect & Command
```
User → GUI button → {command dict} via Pipe → SerialServer → Hexapod.method() → serial port → Teensy
```

### Telemetry
```
Teensy → serial port → Hexapod.run() (thread) → _consume_packets() → process_packet()
    ├── logs telemetry fields
    └── TelemetryRecorder.write() (during playback)
```

### Trajectory Playback
```
1. User clicks UPLOAD → CSV loaded → N packets sent → CRC32 validation sent
2. User clicks PLAY   → recorder starts, PLAY packet sent → device executes trajectory
3. Device streams STATUS/JOG packets → recorder writes raw bytes
4. User clicks STOP   → recorder closes → .bin file ready
5. Open decode_log.py → load .bin → plots + optional Excel export
```

---

## Hardware Notes

| Item | Value |
|------|-------|
| Controller | Teensy 4.1 |
| USB VID/PID | `0x16C0` / `0x0483` |
| Protocol | UART over USB-CDC |
| Packet size | 64 bytes fixed |
| Max slaves | 3 (each with 2 axes = 6 total) |
| Trajectory DOF | 6 (float32 per point) |
| Angle input range | ±60° (UI clamp) |

---

## Dependencies

| Package | Used for |
|---------|---------|
| `customtkinter` | GUI |
| `pyserial` | Serial port I/O |
| `fastcrc` | CRC16-XMODEM |
| `numpy` | Array ops, deg→rad, float32 packing |
| `usbx` | USB hotplug monitoring |
| `more_itertools` | `chunked()` for batch sends |
| `pandas` | DataFrames in decode_log |
| `matplotlib` | Plots in decode_log |
