# mousetrap HID protocol

Wire format between the device firmware and the setup page. Protocol version **1**.

## Transport

The device presents a single USB HID interface with two top-level collections:

| Report ID | Collection | Purpose |
| --- | --- | --- |
| 1 | Generic Desktop / Mouse | ordinary mouse — the OS drives this, browsers cannot touch it |
| 2 | Vendor-defined, usage page `0xFF00` | this protocol |

The split matters: browsers are forbidden by spec from opening protected collections such as mice and keyboards. Putting the control channel on a vendor usage page is what lets a web page talk to the device while it continues to work as a mouse.

From a browser:

```js
const [device] = await navigator.hid.requestDevice({
  filters: [{ usagePage: 0xFF00 }]
});
await device.open();
device.addEventListener("inputreport", e => { /* e.reportId === 2 */ });
```

### Framing

Every report in both directions is **64 bytes**, report ID 2, zero-padded. Byte 0 is the opcode. Floats are **IEEE-754 little-endian**, 4 bytes each, packed with no alignment padding.

In WebHID the report ID is passed separately and is *not* part of the data buffer, so byte 0 of `event.data` is the opcode.

```js
const buf = new Uint8Array(64);
buf[0] = 0x01;
await device.sendReport(2, buf);
```

The firmware handles one command at a time and ignores a new one while a reply is pending, so wait for each response before sending the next.

---

## Host → device

| Op | Name | Payload | Reply |
| --- | --- | --- | --- |
| `0x01` | PING | — | `0x81` |
| `0x10` | STREAM | `u8` 1 = on, 0 = off | none |
| `0x11` | GET_SETTINGS | — | `0x91` |
| `0x12` | SET_SETTINGS | 8 × f32 | `0x91` |
| `0x13` | GET_CAL | — | `0x93` |
| `0x20` | CAL_BEGIN | — | `0xA3` |
| `0x21` | CAL_REST | — | `0xA1` |
| `0x22` | CAL_DIR | `u8` direction 0–3 | `0xA2` |
| `0x23` | CAL_COMPUTE | — | `0xA3` |
| `0x24` | CAL_ABORT | — | `0xA3` |
| `0x30` | SAVE | — | `0xA3` |
| `0x31` | DEFAULTS | — | `0xA3` |

Direction indices are **0 = up, 1 = down, 2 = left, 3 = right**.

`SET_SETTINGS` payload order, all f32, starting at byte 1:

```
deadzone, speed, trimUp, trimDown, trimLeft, trimRight, scrollStep, buttonQuietMs
```

`scrollStep` and `buttonQuietMs` are integers sent as floats; the firmware truncates them.

Settings changes apply immediately but live in RAM. `SAVE` writes them to flash. `DEFAULTS` restores factory values **and clears the calibration**, so a full calibration is needed afterwards.

---

## Device → host

### `0x81` PONG

| Offset | Type | Field |
| --- | --- | --- |
| 1 | u8 | protocol version |
| 2 | u8 | firmware version |
| 3 | u8 | calibration valid (0/1) |
| 4 | f32 | tilt change since calibration, degrees |

Tilt change is the angle between the current resting orientation and the one recorded at calibration. A large value means the device has been physically reoriented since it was set up.

### `0x91` SETTINGS

Byte 1 onward: the same 8 floats as `SET_SETTINGS`, in the same order.

### `0x93` CAL

| Offset | Type | Field |
| --- | --- | --- |
| 1 | u8 | valid (0/1) |
| 2 | 4 × f32 | `XX XY YX YY` — rotation matrix, sensor tilt → screen axes |
| 18 | 4 × f32 | `gUp gDown gLeft gRight` — per-direction gain |
| 34 | 2 × f32 | `magOn`, `magOff` — gate thresholds, µT |
| 42 | f32 | tilt change, degrees |

### `0xA1` REST

| Offset | Type | Field |
| --- | --- | --- |
| 1 | u8 | status (0 = ok) |
| 2 | f32 | resting magnet noise (peak over the sample window) |
| 6 | 3 × f32 | resting accelerometer `x y z` |

### `0xA2` DIR

| Offset | Type | Field |
| --- | --- | --- |
| 1 | u8 | direction index 0–3 |
| 2 | f32 | mean tilt `dX` |
| 6 | f32 | mean tilt `dY` |
| 10 | f32 | magnet minimum over the window |
| 14 | f32 | magnet maximum over the window |

### `0xA3` RESULT

| Offset | Type | Field |
| --- | --- | --- |
| 1 | u8 | error code (0 = success) |

On a successful `CAL_COMPUTE` only, five more floats follow:

| Offset | Type | Field |
| --- | --- | --- |
| 2 | f32 | resting magnet noise |
| 6 | f32 | weakest push reading |
| 10 | f32 | computed `magOn` |
| 14 | f32 | computed `magOff` |
| 18 | f32 | margin ratio (weakest ÷ rest) |

Error codes:

| Code | Meaning | What to tell the user |
| --- | --- | --- |
| 1 | a step wasn't recorded | run calibration again |
| 2 | push vectors too small or too collinear (determinant < 0.2) | push further and hold steady |
| 3 | directions didn't resolve to distinct screen axes | each push must go the way the step says |
| 4 | weakest push < 1.2 × resting noise | the magnet can't separate a push from rest — it may need to sit closer to the sensor |

A failed compute leaves the previous calibration untouched.

### `0x50` STATE

Streamed at ~20 Hz while streaming is enabled. Not a reply — it arrives unsolicited, so dispatch on the opcode rather than assuming reports are responses.

| Offset | Type | Field |
| --- | --- | --- |
| 1 | u8 | flags |
| 2 | f32 | tilt `dX` |
| 6 | f32 | tilt `dY` |
| 10 | f32 | magnet magnitude |
| 14 | f32 | screen `sx` after rotation, gain and deadzone |
| 18 | f32 | screen `sy` |
| 22 | f32 | cursor `moveX` (pixels this tick) |
| 26 | f32 | cursor `moveY` |
| 30 | f32 | encoder count |
| 34 | 3 × f32 | raw accelerometer `x y z` |

Flags bitfield:

| Bit | Meaning |
| --- | --- |
| 0 | gripped — magnet gate open |
| 1 | left button down |
| 2 | right button down |
| 3 | calibration mode — cursor movement suspended |

Cursor values are computed and streamed even in calibration mode, so the page can show what *would* happen while the mouse itself stays still.

---

## Calibration sequence

```
CAL_BEGIN            → movement suspended
CAL_REST             → re-zero, measure resting noise      (~2 s, hands off)
CAL_DIR 0,1,2,3      → one per direction, each held         (~2 s each)
CAL_COMPUTE          → solve, validate, save, resume
```

`CAL_ABORT` at any point restores movement and leaves the stored calibration alone.

Each record step blocks for roughly two seconds of sampling. The host should prompt the user into position *before* sending the command, since the firmware begins sampling immediately on receipt.

## Compatibility

Check `PONG`'s protocol version before issuing anything else. Within a major version, fields may be appended to the end of a report but existing offsets will not move.
