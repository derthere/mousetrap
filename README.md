# mousetrap

A magnetic space mouse. Push the handle and the cursor moves; the magnet decides *when* it's allowed to, the accelerometer decides *where*.

The handle sits on a sprung mount over a fixed magnet. Pushing it tilts an IMU, and that tilt becomes cursor motion. A separate magnetometer reading tells the firmware the handle has actually left centre — that gate is what keeps the cursor still when you bump the desk or pick the whole thing up, which a tilt sensor alone can't distinguish from a real push.

---

## Hardware

| Part | Notes |
| --- | --- |
| SparkFun Pro Micro RP2350 | 3.3 V, Pro Micro footprint, USB-C |
| Adafruit ICM-20948 | 9-DoF IMU over Qwiic / STEMMA QT — accelerometer and magnetometer both used |
| Rotary encoder (EC11 or similar) | scroll wheel; common pin to GND |
| 2 × momentary switches | left and right click |
| Magnet | fixed in the base, under the handle's travel |

### Pins

Edit these at the top of the sketch if your wiring differs. They are **GPIO numbers**, which do not match the silkscreen-to-GPIO mapping of the ATmega32U4 Pro Micro — if you're porting from one, re-check them.

```
LEFT_CLICK_PIN   4
RIGHT_CLICK_PIN  3
ENC_A            6
ENC_B            5
```

`PRESSED` is `HIGH` for switches wired to 3.3 V. For switches wired COM→GND, NO→pin, set it to `LOW`.

The IMU connects over Qwiic — no soldering, no level shifting, since the RP2350 is natively 3.3 V.

---

## Firmware

Two sketches live in `/firmware`:

- **`mousetrap.ino`** — the full build. Presents as a USB mouse *and* a raw-HID device the setup page talks to.
- **`mousetrap-serial.ino`** — same behaviour, calibration driven from the Arduino Serial Monitor instead. Useful on the bench.

### Building

1. Install the **Raspberry Pi Pico/RP2040/RP2350** core by Earle F. Philhower, III. Add this to *Settings → Additional boards manager URLs*, then install from Boards Manager:
   ```
   https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
   ```
   The official "Arduino Mbed OS RP2040" package will not work — it lacks the board definition and handles HID differently.
2. Libraries, all from Library Manager: `Adafruit ICM20X`, `Adafruit Unified Sensor`, `Adafruit BusIO`, `Adafruit TinyUSB`.
3. *Tools → Board* → **SparkFun Pro Micro RP2350**.
4. *Tools → USB Stack* → **Adafruit TinyUSB**. Required — the mouse won't enumerate on the Pico SDK stack.
5. First upload: hold **BOOT** while plugging in the USB cable. A drive appears; the IDE flashes to it. Afterwards a normal serial port shows up and you can upload to that instead.

The IDE often reports `Failed uploading: exit status 1` right after a successful BOOTSEL flash. The board reboots and unmounts the drive before the IDE finishes tidying up. If you saw `Wrote … bytes`, it worked.

---

## How it works

Every cycle (~50 Hz) the firmware:

1. Reads the IMU and subtracts the resting baseline, giving tilt deltas `dX, dY`.
2. Rotates those into screen axes. The sensor rarely sits square to the handle's push directions — in the current enclosure it's about 45° off — so a 2×2 matrix maps sensor tilt onto screen X/Y.
3. Applies a per-direction gain, because handle travel is rarely symmetric. Calibration normalises each of the four directions so a full push reaches 1.0.
4. Applies a deadzone, then scales to cursor pixels.
5. Checks the magnet. The reading is the magnitude of the field's change from rest across all three axes — direction-free, so it can only gate, never steer. It uses hysteresis (on at `magOn`, off at the lower `magOff`) so a light push doesn't flicker in and out.

Three values are stored per unit, in flash:

- **rotation** — `XX XY YX YY`, sensor tilt → screen axes
- **gains** — `gUp gDown gLeft gRight`, evens out uneven handle travel
- **magnet thresholds** — `magOn`, `magOff`

### Two kinds of calibration

| | When | What it does |
| --- | --- | --- |
| **Zeroing** | automatically, every power-up | Records the resting tilt and magnetic field. Takes ~3 s — keep hands off the handle. Needed because the Earth's field shifts when you move or rotate the base. |
| **Full calibration** | once per unit, or after moving the magnet or rebuilding the enclosure | Measures all four push directions and computes rotation, gains and thresholds. Run from the setup page. |

Full calibration is **not** a startup routine. Once a unit is set up, the stored values hold until the hardware changes.

---

## Setup page

`/web/index.html` — a single static page, no build step, no dependencies. Serve it anywhere (GitHub Pages works) and open it in **Chrome or Edge**. WebHID is unsupported in Safari and Firefox.

It connects over raw HID alongside the mouse interface, so the device keeps working as a mouse the whole time. It offers:

- live magnet, tilt and cursor-vector readouts, with the gate thresholds drawn on the magnet bar
- a step-by-step calibration wizard that waits for you rather than running on timers
- live sliders for speed, deadzone, scroll step, click pause and the four trims
- a copyable diagnostics dump — the fastest way for a tester to tell you what's wrong

Calibration failures report a cause rather than just failing: pushes too small or too alike, directions not distinct, or the magnet unable to separate a push from rest.

See [`docs/protocol.md`](docs/protocol.md) for the wire format.

---

## Tuning

Most of this is adjustable from the setup page; the constants are at the top of the sketch if you'd rather bake them in.

| Symptom | Try |
| --- | --- |
| Cursor drifts while holding still | raise the deadzone |
| One direction feels stiff or slow | raise that direction's trim; also check for mechanical binding |
| Movement hesitates on one side | magnet margin is thin there — recentre the magnet over the sensor |
| Cursor moves on its own after moving the base | re-zero (unplug and replug, hands off) |
| Clicking nudges the cursor | raise the click pause |
| Scrolling too slow | raise the scroll step (values above 127 per report are chunked automatically) |

A healthy magnet margin is the weakest push reading at **2× or more** the resting noise. Below about 1.2× calibration refuses to save, because the gate can't reliably tell a push from rest.

The single most useful hardware fix is centring the magnet under the sensor. An off-centre magnet swings much closer on one side than the other, which shows up as one weak direction and forces the threshold uncomfortably low.

---

## Repository layout

```
firmware/   Arduino sketches
web/        setup page (static, WebHID)
hardware/   enclosure files, wiring notes
docs/       protocol spec, tester instructions
```

## Licence

Firmware and web app: **GPL-3.0** (see `LICENSE`).
Hardware files under `hardware/`: **CERN-OHL-S v2**.

Copyright © 2026 derthere
