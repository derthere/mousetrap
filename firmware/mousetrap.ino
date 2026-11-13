// mousetrap firmware - RP2350 Pro Micro
// HID mouse + vendor raw HID for the calibration web app
//
// Build: Philhower RP2040/RP2350 core, board = SparkFun Pro Micro RP2350,
//        Tools > USB Stack > Adafruit TinyUSB.
// Libraries: Adafruit ICM20X, Adafruit Unified Sensor, Adafruit BusIO, Adafruit TinyUSB.

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_TinyUSB.h>

#define FW_VERSION    2
#define PROTO_VERSION 1

// ---- Pins (GPIO numbers) ----
#define LEFT_CLICK_PIN  3   // swapped Nov 2026: physical left switch is on GPIO 3
#define RIGHT_CLICK_PIN 4
#define ENC_A 6
#define ENC_B 5
#define PRESSED HIGH        // LOW if switches are wired COM->GND, NO->pin

#define DEBUG_USB 0         // 1 logs every raw-HID command to Serial

// Cursor stays still this long after the scroll wheel moves, so turning the
// knob doesn't drag the pointer with it.
#define SCROLL_QUIET_MS 250
// Signal changes per physical click of the scroll knob (EC11-type: usually 4,
// some are 2). Watch "enc" on the setup page while turning one click to check.
#define ENC_COUNTS_PER_DETENT 1   // this build's knob: one count per click

// ---- Motion feel ----
// All tunable from here; the web page's "speed" slider still scales the top speed.
#define LOOP_US          5000   // fixed 200 Hz loop; motion is computed per second, not per loop
#define PX_PER_S_PER_SPEED 80.0 // top speed = set.speed * this (speed 25 -> 2000 px/s at full push)
#define CURVE_EXP        2.2    // response curve: 1 = linear, higher = finer near centre
#define MAX_PUSH         1.15   // calibrated full push = 1.0; allow a little past it
// One Euro filter: smooth when nearly still, near-zero lag when moving fast.
#define EURO_MIN_CUTOFF  2.5    // Hz. Lower = steadier hold, more lag on slow moves (1.5 rounded off small circles)
#define EURO_BETA        0.4    // Higher = less lag on fast moves
#define EURO_D_CUTOFF    1.0
#define GATE_RAMP_MS     60     // cursor eases in over this long after the magnet gate opens
// Plateau curve (TrackPoint-style, Selker & Rutledge): soft ramp, then a wide
// band of steady speed for normal pushes, then a rise to top speed at full push.
// Its shape (ramp end, boost start, crispness) is set from the web page.
#define NEG_INERTIA_TAU  0.10   // seconds
#define REBOUND_MS       120    // a direction reversal must sit in the deadzone this long first; 0 = off
#define REBOUND_MIN_PEAK 0.5    // ...but only after a push that went past this fraction of full
#define RECENTRE_RATE    0.002  // per loop, while hands-off: slowly re-zero drifting rest values

void onDeviceCalibration();   // defined at the bottom

// Declared up here because the IDE auto-generates prototypes at the top of
// the file; a type used in a return position must already be known.
struct Sample { float dX, dY, mag, rawAX, rawAY, rawAZ, mX, mY, mZ; };

// One Euro filter (Casiez et al. 2012): a low-pass whose cutoff rises with speed.
struct Euro {
  float x = 0, dx = 0; bool init = false;
  static float alpha(float cutoff, float dt) { float tau = 1.0f / (2 * PI * cutoff); return 1.0f / (1.0f + tau / dt); }
  float step(float v, float dt) {
    if (!init) { x = v; dx = 0; init = true; return v; }
    float d = (v - x) / dt;
    dx += alpha(EURO_D_CUTOFF, dt) * (d - dx);
    float cutoff = EURO_MIN_CUTOFF + EURO_BETA * fabs(dx);
    x += alpha(cutoff, dt) * (v - x);
    return x;
  }
};

// ================= USB HID =================

#define RID_MOUSE 1
#define RID_RAW   2
// 63, not 64: the report ID byte rides along on the wire, and the endpoint
// is 64 bytes total. At 64 the host's reports arrive split in two and ours
// are refused outright.
#define RAW_SIZE  63

// Two SEPARATE HID interfaces, not two collections on one interface.
// macOS seizes any HID interface containing a mouse, so the vendor channel
// needs its own interface or the browser cannot open the device.

uint8_t const desc_mouse_report[] = {
  TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(RID_MOUSE))
};

uint8_t const desc_raw_report[] = {
  0x06, 0x00, 0xFF,              // Usage Page (Vendor 0xFF00)
  0x09, 0x01,                    // Usage 1
  0xA1, 0x01,                    // Collection (Application)
  0x85, RID_RAW,                 //   Report ID
  0x09, 0x02,                    //   Usage 2 (device -> host)
  0x15, 0x00, 0x26, 0xFF, 0x00,
  0x75, 0x08, 0x95, RAW_SIZE,
  0x81, 0x02,                    //   Input
  0x09, 0x03,                    //   Usage 3 (host -> device)
  0x15, 0x00, 0x26, 0xFF, 0x00,
  0x75, 0x08, 0x95, RAW_SIZE,
  0x91, 0x02,                    //   Output
  0xC0
};

Adafruit_USBD_HID usb_mouse;
Adafruit_USBD_HID usb_raw;
uint8_t mouseButtons = 0;

void mouseMove(int8_t x, int8_t y, int8_t wheel) {
  if (usb_mouse.ready()) usb_mouse.mouseReport(RID_MOUSE, mouseButtons, x, y, wheel, 0);
}
// Button changes are sent by the loop in one combined report with movement and
// scroll. Sending them separately meant whichever came second often found the
// USB endpoint busy and was silently dropped.
bool buttonsDirty = false;
long pendingWheel = 0;            // scroll waiting to be sent
void mouseSetButton(uint8_t mask, bool down) {
  uint8_t before = mouseButtons;
  if (down) mouseButtons |= mask; else mouseButtons &= ~mask;
  if (before != mouseButtons) buttonsDirty = true;
}
#define BTN_LEFT  1
#define BTN_RIGHT 2

// ================= STORED DATA =================

struct Cal {
  float XX, XY, YX, YY;
  float gUp, gDown, gLeft, gRight;
  float magOn, magOff;
  float restAX, restAY, restAZ;
};
struct Settings {
  float deadzone, speed;
  float trimUp, trimDown, trimLeft, trimRight;
  int16_t scrollStep;
  uint16_t buttonQuietMs;
  float curve;      // 0 = power curve, 1 = plateau curve
  float plateau;    // plateau speed as a fraction of top speed
  float rampEnd;    // push fraction where the soft ramp reaches the plateau
  float boostStart; // push fraction where speed starts rising above the plateau
  float crisp;      // negative inertia: exaggerate speed changes; 0 = off
  float axisSnap;   // degrees: pushes this close to horizontal/vertical lock onto the axis
};
struct Store { uint16_t magic; Cal cal; Settings set; };

const uint16_t STORE_MAGIC = 0xCA1E;
const Cal      DEF_CAL = { -0.252, 0.571, 0.945, 0.270, 0.65, 2.1, 0.97, 1.03, 4.0, 3.0, 0, 0, 0 };
const Settings DEF_SET = { 0.35, 25.0, 1.0, 1.0, 1.0, 1.0, 50, 150, 0, 0.25, 0.35, 0.85, 0.0, 10.0 };


Cal cal = DEF_CAL;
Settings set = DEF_SET;
bool calValid = false;

// Settings saved by older firmware end before these fields, so whatever sits
// in flash there is garbage. Pull anything out of range back to defaults.
void sanitizeSettings() {
  if (!(set.curve == 0 || set.curve == 1)) set.curve = 0;
  if (!(set.plateau >= 0.1f && set.plateau <= 0.8f)) set.plateau = DEF_SET.plateau;
  if (!(set.rampEnd >= 0.05f && set.rampEnd <= 0.6f)) set.rampEnd = DEF_SET.rampEnd;
  if (!(set.boostStart >= 0.4f && set.boostStart <= 0.98f)) set.boostStart = DEF_SET.boostStart;
  if (set.boostStart < set.rampEnd + 0.05f) set.boostStart = set.rampEnd + 0.05f;
  if (!(set.crisp >= 0 && set.crisp <= 1)) set.crisp = DEF_SET.crisp;
  if (!(set.axisSnap >= 0 && set.axisSnap <= 25)) set.axisSnap = DEF_SET.axisSnap;
}

float smoothstep01(float x) { x = constrain(x, 0.0f, 1.0f); return x * x * (3 - 2 * x); }

// Push fraction t (0 at deadzone edge, 1 at calibrated full push) -> fraction of top speed.
float plateauCurve(float t) {
  float P = set.plateau, a = set.rampEnd, b = set.boostStart;
  if (t < a)     return P * smoothstep01(t / a);
  if (t < b)     return P;
  if (t <= 1.0f) return P + (1 - P) * smoothstep01((t - b) / (1 - b));
  return t;                                    // past full push: keep rising gently
}

void saveStore() {
  Store s = { STORE_MAGIC, cal, set };
  EEPROM.put(0, s);
  EEPROM.commit();
}

// ================= ENCODER =================

volatile long encPos = 0;
volatile uint8_t encState = 0;
const int8_t ENC_TABLE[16] = { 0, -1, 1, 0,  1, 0, 0, -1,  -1, 0, 0, 1,  0, 1, -1, 0 };

void encISR() {
  encState = ((encState << 2) | (digitalRead(ENC_A) << 1) | digitalRead(ENC_B)) & 0x0F;
  encPos += ENC_TABLE[encState];
}
long encRead() { noInterrupts(); long p = encPos; interrupts(); return p; }
long encEmitPos = 0;              // encoder count at the last click we scrolled for
uint8_t encRest = 0;              // A/B state when the knob sits in a detent
bool encAtRest() {
  uint8_t ab = encState & 3;
  if (ENC_COUNTS_PER_DETENT == 2) return ab == 0 || ab == 3;
  if (ENC_COUNTS_PER_DETENT == 4) return ab == encRest;
  return false;
}

// ================= SENSOR =================

Adafruit_ICM20948 icm;
float restAccX, restAccY, restAccZ, restMagX, restMagY, restMagZ;

Sample readSensor() {
  sensors_event_t a, g, t, m;
  icm.getEvent(&a, &g, &t, &m);
  Sample s;
  s.rawAX = a.acceleration.x;  s.rawAY = a.acceleration.y;  s.rawAZ = a.acceleration.z;
  s.dX = s.rawAX - restAccX;
  s.dY = s.rawAY - restAccY;
  float mx = m.magnetic.x - restMagX, my = m.magnetic.y - restMagY, mz = m.magnetic.z - restMagZ;
  s.mag = sqrt(mx * mx + my * my + mz * mz);
  s.mX = mx;  s.mY = my;  s.mZ = mz;   // field components relative to rest, uT
  return s;
}

void baseline() {   // resting zero, blocking, ~1 s
  float sAX = 0, sAY = 0, sAZ = 0, sMX = 0, sMY = 0, sMZ = 0;
  restMagX = restMagY = restMagZ = 0;
  for (int i = 0; i < 20; i++) {
    sensors_event_t a, g, t, m;
    icm.getEvent(&a, &g, &t, &m);
    sAX += a.acceleration.x;  sAY += a.acceleration.y;  sAZ += a.acceleration.z;
    sMX += m.magnetic.x;  sMY += m.magnetic.y;  sMZ += m.magnetic.z;
    delay(50);
  }
  restAccX = sAX / 20;  restAccY = sAY / 20;  restAccZ = sAZ / 20;
  restMagX = sMX / 20;  restMagY = sMY / 20;  restMagZ = sMZ / 20;
}

float tiltChangeDeg() {
  float n1 = sqrt(sq(restAccX) + sq(restAccY) + sq(restAccZ));
  float n2 = sqrt(sq(cal.restAX) + sq(cal.restAY) + sq(cal.restAZ));
  if (n1 < 1 || n2 < 1) return 180;
  float dot = restAccX * cal.restAX + restAccY * cal.restAY + restAccZ * cal.restAZ;
  return degrees(acos(constrain(dot / (n1 * n2), -1.0f, 1.0f)));
}

// ================= CALIBRATION STATE =================

bool calMode = false;            // movement disabled while true
uint8_t recMode = 0;             // 0 idle, 1 rest-noise, 2 direction
uint8_t recDir = 0;
unsigned long recStart = 0;
int recN = 0;
float recSX, recSY, recMagLo, recMagHi;

float dirX[4], dirY[4], dirLo[4], dirHi[4];   // up, down, left, right
bool  dirDone[4] = {false, false, false, false};
float restNoise = 0;
bool  restDone = false;

const unsigned long REC_MS = 1800;

void startRecord(uint8_t mode, uint8_t dir) {
  recMode = mode;  recDir = dir;  recStart = millis();
  recN = 0;  recSX = recSY = 0;  recMagLo = 1e6;  recMagHi = 0;
}

// ================= PROTOCOL =================

// host -> device
#define C_PING        0x01
#define C_STREAM      0x10
#define C_GET_SETTING 0x11
#define C_SET_SETTING 0x12
#define C_GET_CAL     0x13
#define C_CAL_BEGIN   0x20
#define C_CAL_REST    0x21
#define C_CAL_DIR     0x22
#define C_CAL_COMPUTE 0x23
#define C_CAL_ABORT   0x24
#define C_SAVE        0x30
#define C_DEFAULTS    0x31
// device -> host
#define R_PONG        0x81
#define R_SETTINGS    0x91
#define R_CAL         0x93
#define R_REST        0xA1
#define R_DIR         0xA2
#define R_RESULT      0xA3
#define R_STATE       0x50

bool streaming = false;
volatile bool cmdPending = false;
uint8_t cmdBuf[RAW_SIZE];

void set_report_cb(uint8_t id, hid_report_type_t type, uint8_t const* buf, uint16_t len) {
  if (len == 0) return;
  // Two delivery paths. Over the control pipe, `id` is the report ID and `buf`
  // starts at the payload. Over an interrupt OUT endpoint, `id` is 0 and the
  // report ID is the first byte of `buf`. Normalise both.
  if (id == 0 && buf[0] == RID_RAW && len > 1) { buf++; len--; }
  if (DEBUG_USB) { Serial.print("cmd 0x"); Serial.println(buf[0], HEX); }
  if (cmdPending) return;                       // one at a time; app waits for the reply
  memcpy(cmdBuf, buf, min((int)len, RAW_SIZE));
  cmdPending = true;
}

void pf(uint8_t* b, int &i, float v) { memcpy(b + i, &v, 4); i += 4; }
float uf(const uint8_t* b, int &i) { float v; memcpy(&v, b + i, 4); i += 4; return v; }

void send(uint8_t* b) {
  if (!usb_raw.ready()) { if (DEBUG_USB) Serial.println("  raw not ready"); return; }
  bool ok = usb_raw.sendReport(RID_RAW, b, RAW_SIZE);
  if (DEBUG_USB && !ok) Serial.println("  sendReport failed");
}

void sendSettings() {
  uint8_t b[RAW_SIZE] = {0}; int i = 0;
  b[i++] = R_SETTINGS;
  pf(b, i, set.deadzone); pf(b, i, set.speed);
  pf(b, i, set.trimUp); pf(b, i, set.trimDown);
  pf(b, i, set.trimLeft); pf(b, i, set.trimRight);
  pf(b, i, (float)set.scrollStep); pf(b, i, (float)set.buttonQuietMs);
  pf(b, i, set.curve); pf(b, i, set.plateau);
  pf(b, i, set.rampEnd); pf(b, i, set.boostStart); pf(b, i, set.crisp);
  pf(b, i, set.axisSnap);
  send(b);
}

void sendCal() {
  uint8_t b[RAW_SIZE] = {0}; int i = 0;
  b[i++] = R_CAL;  b[i++] = calValid ? 1 : 0;
  pf(b, i, cal.XX); pf(b, i, cal.XY); pf(b, i, cal.YX); pf(b, i, cal.YY);
  pf(b, i, cal.gUp); pf(b, i, cal.gDown); pf(b, i, cal.gLeft); pf(b, i, cal.gRight);
  pf(b, i, cal.magOn); pf(b, i, cal.magOff);
  pf(b, i, tiltChangeDeg());
  send(b);
}

void handleCommand() {
  uint8_t* c = cmdBuf;
  uint8_t op = c[0];
  int i = 1;
  uint8_t b[RAW_SIZE] = {0};

  switch (op) {
    case C_PING: {
      int j = 0;
      b[j++] = R_PONG;  b[j++] = PROTO_VERSION;  b[j++] = FW_VERSION;
      b[j++] = calValid ? 1 : 0;
      pf(b, j, tiltChangeDeg());
      send(b);
      break;
    }
    case C_STREAM:    streaming = c[1] != 0; break;
    case C_GET_SETTING: sendSettings(); break;
    case C_SET_SETTING: {
      set.deadzone = uf(c, i); set.speed = uf(c, i);
      set.trimUp = uf(c, i);   set.trimDown = uf(c, i);
      set.trimLeft = uf(c, i); set.trimRight = uf(c, i);
      set.scrollStep = (int16_t)uf(c, i);
      set.buttonQuietMs = (uint16_t)uf(c, i);
      set.curve = uf(c, i);  set.plateau = uf(c, i);   // 0s from an older page
      set.rampEnd = uf(c, i);  set.boostStart = uf(c, i);  set.crisp = uf(c, i);
      set.axisSnap = uf(c, i);
      sanitizeSettings();
      sendSettings();
      break;
    }
    case C_GET_CAL: sendCal(); break;

    case C_CAL_BEGIN:
      calMode = true;  restDone = false;
      for (int d = 0; d < 4; d++) dirDone[d] = false;
      b[0] = R_RESULT; b[1] = 0; send(b);
      break;

    case C_CAL_REST:
      baseline();                 // zero to the current resting position
      startRecord(1, 0);          // then measure how noisy that rest is
      break;

    case C_CAL_DIR:
      if (c[1] < 4) startRecord(2, c[1]);
      break;

    case C_CAL_COMPUTE: {
      bool have = restDone;
      for (int d = 0; d < 4; d++) if (!dirDone[d]) have = false;
      uint8_t err = 0;
      Cal n = cal;
      if (!have) err = 1;                        // missing a step
      else {
        float vX = (dirX[1] - dirX[0]) / 2, vY = (dirY[1] - dirY[0]) / 2;
        float hX = (dirX[3] - dirX[2]) / 2, hY = (dirY[3] - dirY[2]) / 2;
        float det = hX * vY - vX * hY;
        if (fabs(det) < 0.2) err = 2;            // pushes too small or too alike
        else {
          n.XX = vY / det;   n.XY = -vX / det;
          n.YX = -hY / det;  n.YY = hX / det;
          float upY    = n.YX * dirX[0] + n.YY * dirY[0];
          float downY  = n.YX * dirX[1] + n.YY * dirY[1];
          float leftX  = n.XX * dirX[2] + n.XY * dirY[2];
          float rightX = n.XX * dirX[3] + n.XY * dirY[3];
          if (!(upY < -0.25 && downY > 0.25 && leftX < -0.25 && rightX > 0.25)) err = 3;
          else {
            n.gUp = -1 / upY;     n.gDown = 1 / downY;
            n.gLeft = -1 / leftX; n.gRight = 1 / rightX;
            float weakest = dirLo[0];
            for (int d = 1; d < 4; d++) if (dirLo[d] < weakest) weakest = dirLo[d];
            if (weakest < restNoise * 1.2) err = 4;    // magnet can't separate push from rest
            else {
              // Gate opens a quarter of the way from rest to a FULL push, not
              // halfway. Halfway meant gentle pushes never opened the gate, so
              // the cursor sat dead and then lurched once you pushed harder.
              // Floor at 1.5x rest noise so desk bumps still don't open it.
              n.magOn  = restNoise + 0.25f * (weakest - restNoise);
              n.magOn  = max(n.magOn, restNoise * 1.5f);
              n.magOn  = min(n.magOn, (restNoise + weakest) / 2);
              n.magOff = (restNoise + n.magOn) / 2;
              n.restAX = restAccX; n.restAY = restAccY; n.restAZ = restAccZ;
              cal = n;  calValid = true;  saveStore();
              int j = 0;
              b[j++] = R_RESULT; b[j++] = 0;
              pf(b, j, restNoise); pf(b, j, weakest);
              pf(b, j, cal.magOn); pf(b, j, cal.magOff);
              pf(b, j, weakest / max(restNoise, 0.01f));   // margin ratio
              send(b);
              calMode = false;
              encEmitPos = encRead();
              break;
            }
          }
        }
      }
      b[0] = R_RESULT; b[1] = err; send(b);
      break;
    }

    case C_CAL_ABORT:
      calMode = false; recMode = 0;
      encEmitPos = encRead();
      b[0] = R_RESULT; b[1] = 0; send(b);
      break;

    case C_SAVE:     saveStore(); b[0] = R_RESULT; b[1] = 0; send(b); break;
    case C_DEFAULTS: cal = DEF_CAL; set = DEF_SET; calValid = false; saveStore();
                     b[0] = R_RESULT; b[1] = 0; send(b); break;
  }
  cmdPending = false;
}

// ================= SETUP =================

void setup() {
  pinMode(LEFT_CLICK_PIN, INPUT_PULLUP);
  pinMode(RIGHT_CLICK_PIN, INPUT_PULLUP);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  encState = (digitalRead(ENC_A) << 1) | digitalRead(ENC_B);
  encRest = encState & 3;         // the knob rests in a detent at power-up
  attachInterrupt(digitalPinToInterrupt(ENC_A), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), encISR, CHANGE);

  TinyUSBDevice.setManufacturerDescriptor("derthere");
  TinyUSBDevice.setProductDescriptor("mousetrap");

  usb_mouse.setPollInterval(2);
  usb_mouse.setReportDescriptor(desc_mouse_report, sizeof(desc_mouse_report));
  usb_mouse.setStringDescriptor("mousetrap mouse");
  usb_mouse.begin();

  usb_raw.setPollInterval(2);
  usb_raw.setReportDescriptor(desc_raw_report, sizeof(desc_raw_report));
  usb_raw.setReportCallback(NULL, set_report_cb);
  usb_raw.setStringDescriptor("mousetrap control");
  usb_raw.enableOutEndpoint(true);   // host -> device reports need their own endpoint
  usb_raw.begin();

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);              // one getEvent() burst: ~2.5 ms at 100 kHz, ~0.6 ms at 400 kHz
  if (!icm.begin_I2C()) { while (1) delay(10); }

  // Library defaults are wrong for this job: +/-16 g range (8x coarser than
  // needed for a sub-1 g tilt) and a 53.6 Hz output rate, slower than the
  // loop, so the cursor got the same sample several times in a row and then
  // a jump. Values stay in m/s^2, so stored calibration remains valid.
  icm.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
  icm.setAccelRateDivisor(4);         // 1125 / (1+4) = 225 Hz, above the 200 Hz loop
  icm.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_50_4_HZ);

  EEPROM.begin(256);
  Store s;
  EEPROM.get(0, s);
  if (s.magic == STORE_MAGIC) { cal = s.cal; set = s.set; calValid = true; }
  sanitizeSettings();

  delay(1000);
  baseline();                 // hands off the handle for ~3 s after plug-in
  encEmitPos = encRead();
  while (!TinyUSBDevice.mounted()) delay(10);
  if (DEBUG_USB) {
    delay(500);
    Serial.println("mousetrap ready - raw HID listening");
  }
}

// ================= LOOP =================

void loop() {
  if (cmdPending) handleCommand();

  Sample s = readSensor();

  // ---- recording for calibration ----
  if (recMode) {
    recSX += s.dX;  recSY += s.dY;  recN++;
    if (s.mag < recMagLo) recMagLo = s.mag;
    if (s.mag > recMagHi) recMagHi = s.mag;
    if (millis() - recStart > REC_MS && recN > 5) {
      uint8_t b[RAW_SIZE] = {0}; int j = 0;
      if (recMode == 1) {
        restNoise = recMagHi;  restDone = true;
        b[j++] = R_REST;  b[j++] = 0;
        pf(b, j, restNoise);
        pf(b, j, restAccX); pf(b, j, restAccY); pf(b, j, restAccZ);
      } else {
        dirX[recDir] = recSX / recN;  dirY[recDir] = recSY / recN;
        dirLo[recDir] = recMagLo;     dirHi[recDir] = recMagHi;
        dirDone[recDir] = true;
        b[j++] = R_DIR;  b[j++] = recDir;
        pf(b, j, dirX[recDir]); pf(b, j, dirY[recDir]);
        pf(b, j, recMagLo);     pf(b, j, recMagHi);
      }
      send(b);
      recMode = 0;
    }
  }

  // ---- timing ----
  static unsigned long lastUs = micros();
  unsigned long nowUs = micros();
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  if (dt <= 0 || dt > 0.05f) dt = LOOP_US * 1e-6f;   // first pass, or after a blocking command

  // ---- magnet gate ----
  static bool gripped = false;
  static unsigned long gripSince = 0;
  bool wasGripped = gripped;
  if (s.mag > cal.magOn)       gripped = true;
  else if (s.mag < cal.magOff) gripped = false;
  if (gripped && !wasGripped) gripSince = millis();

  // ---- hands-off drift correction ----
  // While nobody is touching it, nudge the zero toward the current reading so
  // slow drift (temperature, the sprung mount settling) never becomes a
  // constant bias that makes one direction feel heavier than the other.
  if (!gripped && !calMode && !recMode && fabs(s.dX) < 0.6f && fabs(s.dY) < 0.6f) {
    restAccX += RECENTRE_RATE * s.dX;
    restAccY += RECENTRE_RATE * s.dY;
  }

  // ---- cursor math (always computed so the app can show it live) ----
  static Euro fX, fY;
  float fdX = fX.step(s.dX, dt);
  float fdY = fY.step(s.dY, dt);
  float sx = cal.XX * fdX + cal.XY * fdY;
  float sy = cal.YX * fdX + cal.YY * fdY;
  sx *= (sx > 0) ? cal.gRight * set.trimRight : cal.gLeft * set.trimLeft;
  sy *= (sy > 0) ? cal.gDown  * set.trimDown  : cal.gUp   * set.trimUp;

  // Axis snap: a push within axisSnap degrees of horizontal/vertical is
  // turned onto the axis, so straight lines stay straight. Angles beyond it
  // are stretched to fill the rest of each 45-degree half-quadrant, so
  // direction stays continuous and diagonals still work.
  if (set.axisSnap > 0 && (sx != 0 || sy != 0)) {
    float mag = sqrt(sx * sx + sy * sy);
    float ang = atan2(sy, sx);
    float axis = round(ang / HALF_PI) * HALF_PI;          // nearest of 0, +-90, 180
    float d = ang - axis;                                 // -45..45 deg
    float snap = radians(set.axisSnap), quarter = HALF_PI / 2;
    float ad = fabs(d);
    ad = (ad <= snap) ? 0 : (ad - snap) * quarter / (quarter - snap);
    ang = axis + (d < 0 ? -ad : ad);
    sx = mag * cos(ang);  sy = mag * sin(ang);
  }

  // Radial deadzone and curve. The old per-axis deadzone was a square: any
  // push within 0.35 of an axis got that component zeroed, so shallow
  // diagonals snapped to horizontal/vertical and then jumped when they
  // crossed the threshold. Here the curve acts on the push's length and the
  // direction is kept exactly.
  float r = sqrt(sx * sx + sy * sy);
  float vx = 0, vy = 0;                       // px/s
  float dz = constrain(set.deadzone, 0.0f, 0.9f);
  float t = (r > dz) ? constrain((r - dz) / (1.0f - dz), 0.0f, MAX_PUSH) : 0;
  float frac;                                 // fraction of top speed
  if (set.curve == 1) {
    frac = plateauCurve(t);
    // Negative inertia: add a bit of the difference between the speed now and
    // its recent average, so the cursor gets going (and stops) promptly.
    static float lagged = 0;
    lagged += (frac - lagged) * constrain(dt / NEG_INERTIA_TAU, 0.0f, 1.0f);
    if (frac > 0) frac = constrain(frac + set.crisp * (frac - lagged), 0.0f, MAX_PUSH);
  } else {
    frac = pow(t, CURVE_EXP);
  }
  if (r > dz) {
    float pxs = set.speed * PX_PER_S_PER_SPEED * frac;
    vx = sx / r * pxs;
    vy = sy / r * pxs;
  }

  // Spring rebound: on release the handle swings back through centre and
  // briefly tilts the other way, which flicked the cursor backwards. A
  // reversal (>90 deg from the last direction) is only accepted once the
  // push has rested inside the deadzone for REBOUND_MS.
  // Only armed after a push that went past REBOUND_MIN_PEAK: small pushes
  // can't build a real swing-back, and tight circles at the deadzone edge
  // were being chopped up by the guard.
  static float lastDirX = 0, lastDirY = 0, excursionPeak = 0;
  static bool reboundArmed = false;
  static unsigned long inDeadzoneSince = 0, blockedSince = 0;
  if (r <= dz) {
    if (inDeadzoneSince == 0) {
      inDeadzoneSince = millis();
      reboundArmed = excursionPeak >= REBOUND_MIN_PEAK;
      excursionPeak = 0;
    }
    blockedSince = 0;
  } else {
    bool settled = inDeadzoneSince && millis() - inDeadzoneSince >= REBOUND_MS;
    bool reversal = reboundArmed && (sx * lastDirX + sy * lastDirY) < 0;
    if (reversal && !settled && blockedSince == 0) blockedSince = millis();
    bool timedOut = blockedSince && millis() - blockedSince >= REBOUND_MS;  // deliberate reversal held
    if (REBOUND_MS > 0 && reversal && !settled && !timedOut) {
      vx = vy = 0;                            // ignore the swing-back
    } else {
      lastDirX = sx / r;  lastDirY = sy / r;
      inDeadzoneSince = 0;  blockedSince = 0;
      reboundArmed = false;
      if (t > excursionPeak) excursionPeak = t;
    }
  }
  if (inDeadzoneSince && millis() - inDeadzoneSince > 400) lastDirX = lastDirY = 0;  // long rest: forget direction

  // Ease in after the gate opens so grabbing the handle doesn't lurch.
  if (gripped) {
    unsigned long held = millis() - gripSince;
    if (held < GATE_RAMP_MS) { float k = (float)held / GATE_RAMP_MS; vx *= k; vy *= k; }
  }

  // Sub-pixel accumulator: slow pushes still move, and the speed is the same
  // whatever the loop rate happens to be.
  static float accX = 0, accY = 0;
  accX += vx * dt;  accY += vy * dt;
  int moveX = constrain((int)accX, -127, 127);
  int moveY = constrain((int)accY, -127, 127);

  bool leftPressed  = digitalRead(LEFT_CLICK_PIN)  == PRESSED;
  bool rightPressed = digitalRead(RIGHT_CLICK_PIN) == PRESSED;

  static unsigned long lastScroll = 0;

  if (!calMode) {
    // ---- scroll ----
    // One scroll per physical click, not per signal change: scrolling on every
    // quarter-click made the trigger point drift with contact bounce and
    // where the knob happened to rest. Each time the knob settles in a detent
    // we re-sync, so a missed or bounced count can't shift later clicks.
    static long lastEncRaw = 0;
    long pos = encRead();
    if (pos != lastEncRaw) { lastScroll = millis(); lastEncRaw = pos; }   // pause the cursor from the first twitch
    const long CPD = ENC_COUNTS_PER_DETENT;
    long d = pos - encEmitPos;
    long clicks = 0;
    while (d >=  CPD) { clicks++; encEmitPos += CPD; d -= CPD; }
    while (d <= -CPD) { clicks--; encEmitPos -= CPD; d += CPD; }
    if (CPD > 1 && encAtRest()) {
      if (d * 2 >=  CPD) clicks++;              // most of a click, with a count missed
      else if (d * 2 <= -CPD) clicks--;
      encEmitPos = pos;                         // bounce is absorbed here
    }
    pendingWheel += clicks * CPD * set.scrollStep;   // same distance per click as before

    // ---- both buttons held = on-device calibration fallback ----
    static bool armed = false;
    static unsigned long bothSince = 0;
    if (!leftPressed && !rightPressed) armed = true;
    if (leftPressed && rightPressed && armed) {
      if (bothSince == 0) bothSince = millis();
      if (millis() - bothSince > 2000) { onDeviceCalibration(); armed = false; bothSince = 0; }
      delay(10);
      return;
    }
    bothSince = 0;

    mouseSetButton(BTN_LEFT, leftPressed);
    mouseSetButton(BTN_RIGHT, rightPressed);

    // Settle on button EDGES only (press and release jolt the handle), so
    // click-and-drag still moves during the hold.
    static bool prevL = false, prevR = false;
    static unsigned long lastButtonEdge = 0;
    if (leftPressed != prevL || rightPressed != prevR) lastButtonEdge = millis();
    prevL = leftPressed;  prevR = rightPressed;

    bool allowed = gripped
                && millis() - lastButtonEdge > set.buttonQuietMs
                && millis() - lastScroll > SCROLL_QUIET_MS;
    int mx = 0, my = 0;
    if (allowed) { mx = moveX; my = moveY; }
    else accX = accY = 0;                     // don't bank motion to release later

    // One report carries buttons, movement and scroll. If the endpoint is still
    // busy, nothing is lost: scroll and button changes wait for the next loop.
    int w = constrain(pendingWheel, -127, 127);
    if ((mx || my || w || buttonsDirty) && usb_mouse.ready()) {
      usb_mouse.mouseReport(RID_MOUSE, mouseButtons, mx, my, w, 0);
      accX -= mx;  accY -= my;  pendingWheel -= w;  buttonsDirty = false;
    }
  } else {
    accX = accY = 0;
  }

  // ---- live state to the app ----
  static unsigned long lastStream = 0;
  if (streaming && millis() - lastStream > 50) {
    lastStream = millis();
    uint8_t b[RAW_SIZE] = {0}; int j = 0;
    b[j++] = R_STATE;
    b[j++] = (gripped ? 1 : 0) | (leftPressed ? 2 : 0) | (rightPressed ? 4 : 0) | (calMode ? 8 : 0);
    pf(b, j, s.dX); pf(b, j, s.dY); pf(b, j, s.mag);
    pf(b, j, sx);   pf(b, j, sy);
    pf(b, j, (float)moveX); pf(b, j, (float)moveY);
    pf(b, j, (float)encRead());
    pf(b, j, s.rawAX); pf(b, j, s.rawAY); pf(b, j, s.rawAZ);
    pf(b, j, s.mX);    pf(b, j, s.mY);    pf(b, j, s.mZ);      // bytes 46-57
    send(b);
  }

  // Fixed-rate loop: wait out the rest of the 5 ms period.
  static unsigned long nextUs = micros();
  nextUs += LOOP_US;
  long wait = (long)(nextUs - micros());
  if (wait > 0) delayMicroseconds(wait);
  else nextUs = micros();                     // fell behind (calibration, USB): resync
}

// ================= ON-DEVICE FALLBACK =================
// Cursor-gesture calibration, for when no app is open.
// Hold both buttons for 2 s to start.

void glide(int x, int y, int dist) {
  for (int i = 0; i < 10; i++) { mouseMove(x * dist / 10, y * dist / 10, 0); delay(25); }
}
void shakeCursor(int n) {
  for (int i = 0; i < n; i++) { mouseMove(30, 0, 0); delay(70); mouseMove(-30, 0, 0); delay(70); }
}
void holdAverage(float &aX, float &aY, float &lo, float &hi) {
  float sx = 0, sy = 0; lo = 1e6; hi = 0;
  for (int i = 0; i < 36; i++) {
    Sample s = readSensor();
    sx += s.dX; sy += s.dY;
    if (s.mag < lo) lo = s.mag;
    if (s.mag > hi) hi = s.mag;
    delay(50);
  }
  aX = sx / 36; aY = sy / 36;
}

void onDeviceCalibration() {
  calMode = true;
  unsigned long t0 = millis();
  while ((digitalRead(LEFT_CLICK_PIN) == PRESSED || digitalRead(RIGHT_CLICK_PIN) == PRESSED)
         && millis() - t0 < 5000) delay(10);

  shakeCursor(3);  delay(2000);
  baseline();
  float rx, ry, rlo;
  holdAverage(rx, ry, rlo, restNoise);
  restDone = true;

  const int DIRS[4][2] = { {0, -1}, {0, 1}, {-1, 0}, {1, 0} };
  for (int d = 0; d < 4; d++) {
    for (int k = 0; k < 2; k++) { glide(DIRS[d][0], DIRS[d][1], 150); delay(400);
                                  glide(-DIRS[d][0], -DIRS[d][1], 150); delay(300); }
    delay(1500);
    holdAverage(dirX[d], dirY[d], dirLo[d], dirHi[d]);
    dirDone[d] = true;
    shakeCursor(3);  delay(1500);
  }

  cmdBuf[0] = C_CAL_COMPUTE;      // reuse the same math and save path
  handleCommand();
  if (calValid) { glide(1,0,100); glide(0,1,100); glide(-1,0,100); glide(0,-1,100); }
  else shakeCursor(12);
  calMode = false;
  encEmitPos = encRead();
}
