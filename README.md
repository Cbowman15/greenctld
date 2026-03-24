# RT-21 Rotor Driver

Python driver and rotctld-compatible TCP server for the **Green Heron Engineering RT-21 Digital Rotor Controller**, used in the TeraLink-1 sub-THz ground station to control the **Alfa Spid RAS** az/el antenna mount.

---

## Overview

Two RT-21 units are deployed — one per axis (azimuth and elevation). Each connects to the host PC over RS-232 (or USB-serial). The driver opens both serial ports, translates rotctld network commands from tracking software (e.g., GPredict) into RT-21 serial commands, and returns position responses.

```
GPredict / tracking SW
        │  TCP (rotctld protocol, port 4533)
        ▼
   TCPServer (this script)
        │
        ├── GreenHeronRotor.az_serial ──► RT-21 #1 ──► Alfa Spid RAS (azimuth)
        └── GreenHeronRotor.el_serial ──► RT-21 #2 ──► Alfa Spid RAS (elevation)
```

---

### Serial Port Settings

Fixed by the RT-21 DCU-1 protocol: **4800 baud, 8N1**. Do not change the `--speed` argument unless you have a specific reason.

---

## Software Requirements

```
Python >= 3.6
pyserial
```

---

## Usage

### Basic

```bash
python greenctld.py \
    --az-device /dev/ttyUSB0 \
    --el-device /dev/ttyUSB1
```

The server listens on TCP port 4533 (rotctld default). Point GPredict's rotor interface at `localhost:4533`.

### All options

```
--az-device / -a   Serial device for azimuth RT-21      (e.g. /dev/ttyUSB0)
--el-device / -e   Serial device for elevation RT-21    (e.g. /dev/ttyUSB1)
--speed     / -s   Baud rate, default 4800               (do not change for RT-21)
--timeout   / -t   Serial read timeout in seconds        (default 1.5)
--port      / -p   TCP listen port                       (default 4533)
--get-pos   / -g   Query position once and exit          (serial comms test)
--dummy            Use software stub, no hardware needed
```

### Serial comms test

```bash
python greenctld.py --az-device /dev/ttyUSB0 --el-device /dev/ttyUSB1 --get-pos
```

Queries both RT-21 units once and prints the result. Useful for confirming wiring and baud rate before starting the server.

### Dummy mode (no hardware)

```bash
python greenctld.py --dummy
```

Runs a software-only rotor that converges toward commanded positions in 4-degree steps. Useful for testing GPredict integration without the physical mount.

---

## Supported rotctld Commands

This server implements the subset of the rotctld protocol that GPredict uses:

| Command | Description | Response |
|---|---|---|
| `p` | Get current position | `<az>\n<el>\n` (6 decimal places) |
| `P <az> <el>` | Set target position | `RPRT 0` on success |
| `S` | Stop rotation | `RPRT 0` |
| `q` | Close connection | — |

**Coordinate limits enforced by this driver:**
- Azimuth: 0–359° (360 is remapped to 359)
- Elevation: 0–90°

---

## RT-21 Serial Protocol (reference)

Commands follow the DCU-1 superset protocol (RT-21 User Guide, Appendix G). All uppercase, semicolon-terminated.

| Command | Function |
|---|---|
| `AP1<xxx.y>\r;` | Set target heading to xxx.y degrees (e.g., `AP1045.5\r;`) |
| `BI1;` | Query current heading; responds with `xxx.y;` (6 bytes) |
| `;` | Stop rotation immediately |

Position response format: `xxx.y;` — 3 digits, decimal point, 1 digit, semicolon. Values under 100° are space-padded on the left.

---

## Calibration

Calibration is performed via the RT-21 front panel (or SETUP UTILITY software), not via this driver. For the Alfa Spid / SPID option:

1. Physically align the antenna to a known heading.
2. Enter `SETUP → CALIBRATE` on the RT-21 and set the displayed value to match the actual heading.
3. Set soft limits to prevent over-travel beyond coax loop limits (`SETUP → CCW LIMIT` / `CW LIMIT`).

See RT-21 User Guide §4.2 and Appendix A.8 for the full procedure.

---

## Known Limitations

- Position commands are truncated to integer degrees (`int(float(az))`). The RT-21 supports 0.1° resolution via the AP1 command; if sub-degree commanding is needed, this line should be changed to pass the float directly.
- The `BI1` response read is exactly 6 bytes. If the RT-21 is slow to respond (long cable, high timeout), the read may return fewer bytes and trigger a parse failure. Increase `--timeout` if this occurs.
- The Alfa Spid SPID option uses 1 pulse/degree (360 pulses/revolution). If a 2-pulse/degree variant is used, the RT-21's `DIVIDE HI/LO` must be set to 720 and this driver requires no changes.
- USB functionality on the RT-21 is for computer control only; it cannot be used for Master/Slave or Counter-rotate inter-unit communication (RT-21 User Guide §5.2.1).

---

## References

- Green Heron Engineering, [*RT-21 Digital Rotor Controller User Guide*](https://www.greenheronengineering.com/wp-content/uploads/2019/08/RT-21_Manual_1_3.pdf), Rev 1.3a
