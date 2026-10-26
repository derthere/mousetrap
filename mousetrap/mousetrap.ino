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

#define FW_VERSION    1
#define PROTO_VERSION 1

// ---- Pins (GPIO numbers) ----
#define LEFT_CLICK_PIN  4
#define RIGHT_CLICK_PIN 3
#define ENC_A 6
#define ENC_B 5
#define PRESSED HIGH        // LOW if switches are wired COM->GND, NO->pin

#define DEBUG_USB 1         // 1 logs every raw-HID command to Serial

void onDeviceCalibration();   // defined at the bottom

// Declared up here because the IDE auto-generates prototypes at the top of
// the file; a type used in a return position must already be known.
struct Sample { float dX, dY, mag, rawAX, rawAY, rawAZ; };

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
void mouseSetButton(uint8_t mask, bool down) {
  uint8_t before = mouseButtons;
  if (down) mouseButtons |= mask; else mouseButtons &= ~mask;
  if (before != mouseButtons && usb_mouse.ready())
    usb_mouse.mouseReport(RID_MOUSE, mouseButtons, 0, 0, 0, 0);
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
};
struct Store { uint16_t magic; Cal cal; Settings set; };

const uint16_t STORE_MAGIC = 0xCA1E;
const Cal      DEF_CAL = { -0.252, 0.571, 0.945, 0.270, 0.65, 2.1, 0.97, 1.03, 4.0, 3.0, 0, 0, 0 };
const Settings DEF_SET = { 0.35, 25.0, 1.0, 1.0, 1.0, 1.0, 50, 150 };

Cal cal = DEF_CAL;
Settings set = DEF_SET;
bool calValid = false;

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
long oldPosition = 0;

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
              n.magOn  = (restNoise + weakest) / 2;
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
              oldPosition = encRead();
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
      oldPosition = encRead();
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
  if (!icm.begin_I2C()) { while (1) delay(10); }

  EEPROM.begin(256);
  Store s;
  EEPROM.get(0, s);
  if (s.magic == STORE_MAGIC) { cal = s.cal; set = s.set; calValid = true; }

  delay(1000);
  baseline();                 // hands off the handle for ~3 s after plug-in
  oldPosition = encRead();
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

  // ---- cursor math (always computed so the app can show it live) ----
  float sx = cal.XX * s.dX + cal.XY * s.dY;
  float sy = cal.YX * s.dX + cal.YY * s.dY;
  sx *= (sx > 0) ? cal.gRight * set.trimRight : cal.gLeft * set.trimLeft;
  sy *= (sy > 0) ? cal.gDown  * set.trimDown  : cal.gUp   * set.trimUp;
  if (fabs(sx) < set.deadzone) sx = 0;
  if (fabs(sy) < set.deadzone) sy = 0;
  int moveX = constrain((int)(sx * set.speed), -20, 20);
  int moveY = constrain((int)(sy * set.speed), -20, 20);

  static bool gripped = false;
  if (s.mag > cal.magOn)       gripped = true;
  else if (s.mag < cal.magOff) gripped = false;

  bool leftPressed  = digitalRead(LEFT_CLICK_PIN)  == PRESSED;
  bool rightPressed = digitalRead(RIGHT_CLICK_PIN) == PRESSED;

  if (!calMode) {
    // ---- scroll ----
    long newPosition = encRead();
    long diff = newPosition - oldPosition;
    if (diff != 0) {
      long amount = diff * set.scrollStep;
      while (amount != 0) {
        int step = amount > 127 ? 127 : (amount < -127 ? -127 : (int)amount);
        mouseMove(0, 0, step);  amount -= step;
      }
      oldPosition = newPosition;
    }

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

    static unsigned long lastButton = 0;
    if (leftPressed || rightPressed) lastButton = millis();
    if (gripped && millis() - lastButton > set.buttonQuietMs)
      mouseMove(moveX, moveY, 0);
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
    send(b);
  }

  delay(10);
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
  oldPosition = encRead();
}
