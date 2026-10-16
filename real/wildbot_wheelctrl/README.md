# wildbot_wheelctrl

ESP32 firmware for controlling DDSM115 hub motors over RS-485 via serial JSON commands.

## Hardware

- **MCU**: ESP32
- **Motor**: DDSM115 (RS-485, CRC-8/MAXIM)
- **USB serial to the host**: 460800 baud, 8N1 (`SERIAL_BAUDRATE`; must match `wheel_baudrate` in the host's `hardware.yaml`)
- **RS-485 to the motors**: 115200 baud, 8N1
- **Pins**: WHEEL_RX = GPIO 18, WHEEL_TX = GPIO 19

## Communication

Send newline-terminated JSON strings over the ESP32 USB serial. Each command is identified by the `"T"` field.

### Wheel Commands

| Command | T | Example | Description |
|---|---|---|---|
| Stop | 10000 | `{"T":10000,"id":1}` | Stop a single wheel |
| Control | 10010 | `{"T":10010,"id":1,"cmd":50,"act":3}` | Control wheel speed/position/current |
| Control All | 10011 | `{"T":10011,"cmd":[50,50,-50,-50],"act":1}` | Set the target of wheel 1-4 and start polling mode |
| Change Mode | 10012 | `{"T":10012,"id":1,"mode":2}` | Change control mode |
| Polling | 10013 | `{"T":10013,"en":0}` | Leave (`0`) or enter (`1`) polling mode; leaving stops the wheels |
| Get Info | 10032 | `{"T":10032,"id":1}` | Get wheel info (winding temperature, 8-bit position, error). Not available in polling mode |
| Heartbeat | 11001 | `{"T":11001,"time":500}` | Set heartbeat timeout in ms (-1 to disable) |

### Control Modes (`"mode"`)

| Mode | Description |
|---|---|
| 1 | Current loop (cmd: -32767 ~ 32767 -> -8 ~ 8 A) |
| 2 | Speed loop (cmd: -200 ~ 200 rpm) |
| 3 | Position loop (cmd: 0 ~ 32767 -> 0 ~ 360 deg) |

The motors power up in speed loop mode.

### System Commands

| Command | T | Example | Description |
|---|---|---|---|
| Reboot | 600 | `{"T":600}` | Reboot ESP32 |

## Polling mode

The RS-485 bus is half duplex and every motor answers each command, so two commands sent too close together collide with a reply. In polling mode the ESP32 owns the bus timing: the host only updates the targets, and every 20 ms (50 Hz) the ESP32 sends the latest target to wheel 1-4, one wheel per 4 ms, reads each reply, then reports all four wheels in one line.

- Starts with the first `T:10011`, ends with `{"T":10013,"en":0}`.
- While it runs, `T:10000` and `T:10010` only change that wheel's target, and `T:10012` is sent between two cycles.
- The host should use this mode. The single-wheel commands outside polling mode are for bench testing with a serial terminal.

## Feedback

### All wheels (T: 20012), polling mode

One line per cycle:

```json
{"T":20012,"ms":123456,"ok":15,"mode":[2,2,2,2],"spd":[50,50,-50,-50],"pos":[1234,22222,333,4444],"tor":[120,118,-121,-119],"err":[0,0,0,0]}
```

| Field | Meaning |
|---|---|
| `ms` | ESP32 `millis()` when the line was sent |
| `ok` | Bit mask, bit `i` = wheel `i+1` answered in this cycle. A wheel whose bit is 0 keeps its previous values |
| `mode` | Control mode of each wheel |
| `spd` | Speed, rpm |
| `pos` | Position, 0 ~ 32767 -> 0 ~ 360 deg (single turn) |
| `tor` | Torque current, -32767 ~ 32767 -> -8 ~ 8 A |
| `err` | DDSM115 error code (bit 0 sensor, 1 overcurrent, 2 phase overcurrent, 3 stall, 4 troubleshooting) |

Arrays are indexed by wheel id - 1. Wheel `i` is sampled about `(4 - i) * 4 ms` before `ms`.

### Single wheel (T: 20010), outside polling mode

Normal response:

```json
{"T":20010,"id":1,"mode":2,"tor":0,"spd":50,"pos":1234,"err":0}
```

Info response (after `CMD_WHEEL_INFO`). `u8` is the position as 0 ~ 255 -> 0 ~ 360 deg:

```json
{"T":20010,"id":1,"mode":2,"tor":0,"spd":50,"temp":35,"u8":0,"err":0}
```

CRC error response:

```json
{"T":20010,"crc":0}
```

### Heartbeat notice (T: 20011)

Sent once when the heartbeat stops the wheels:

```json
{"T":20011,"hb":0}
```

## Heartbeat

The heartbeat is on by default with a 500 ms timeout: if no command is received within the timeout, all wheels (ID 1-4) are stopped. Any new command resets the timer.

- Change: `{"T":11001,"time":2000}` (2 second timeout)
- Disable: `{"T":11001,"time":-1}`

In polling mode the targets are set to zero and the poll cycle keeps running, so the host still receives feedback.

## Debugging

Uncomment `DEBUG_PRINT_PACKET` in `wildbot_wheelctrl.ino` to dump every raw motor packet as HEX. Those lines are not JSON and roughly double the USB traffic, so leave it off when the host is connected.
