// DuneBroom v2 motor controller firmware.
// Arduino Uno R3 + 4-channel I2C encoder motor driver (address 0x34).
//
// The Pi sends newline-terminated commands at 115200 baud. Distances are in
// encoder ticks: the Pi converts mm and degrees with its calibration config,
// so recalibrating never requires reflashing.
//
//   Command                          Reply
//   PING                             PONG dunebroom_motor <version>
//   H                                (none) heartbeat, feeds the watchdog
//   MAP <lch> <rch> <lsign> <rsign>  OK MAP   driver channel (0-3) of each track and its forward sign
//   D <ticks> [speed]                OK D, then DONE D <reason> <left> <right>   drive straight, -ticks = backward
//   R <ticks> [speed]                OK R, then DONE R ...                       rotate in place, +ticks = clockwise
//   M <left> <right>                 OK M     continuous track speeds (teleop); M 0 0 stops
//   C <ch> <speed>                   OK C     spin one raw driver channel (wiring test)
//   S                                OK S     stop now (a running move also reports DONE ... STOP)
//   E                                ENC <ms> <left> <right>
//   X                                RAW <ms> <e0> <e1> <e2> <e3>
//   B                                BAT <millivolts>
//
// Diagnostics (wiring and driver-board debugging):
//   P <ch> <duty>                    OK P     open-loop PWM on one raw channel (no encoder feedback)
//   RD <reg> <len>                   REG <reg> <b0> ... <bN>   read 1-16 raw bytes from the driver
//   WR <reg> <b0> [b1 b2 b3]         OK WR    write raw bytes to the driver
//
// Speeds are the driver's closed-loop speed units (-100..100).
// DONE reports ticks travelled by each track since the move began.
// DONE reasons: TARGET, STOP, WATCHDOG (no command for WATCHDOG_MS), STALL
// (tracks stopped turning), I2C (driver not answering).
// Errors: ERR <code>.

#include <Wire.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define FW_VERSION "2"

const uint8_t DRV_ADDR = 0x34;
const uint8_t REG_BATTERY = 0x00;
const uint8_t REG_MOTOR_TYPE = 0x14;
const uint8_t REG_ENC_POLARITY = 0x15;
const uint8_t REG_PWM = 0x1F;
const uint8_t REG_SPEED = 0x33;
const uint8_t REG_ENC_TOTAL = 0x3C;
const uint8_t MOTOR_TYPE_JGB37_520_12V_150RPM = 3;

const unsigned long WATCHDOG_MS = 600;
const unsigned long CONTROL_MS = 20;
const unsigned long RAMP_MS = 300;    // accelerate from MIN_SPEED to cruise over this long
const unsigned long STALL_MS = 700;   // stop if progress < STALL_TICKS for this long
const long STALL_TICKS = 5;
const long SLOW_TICKS = 400;          // decelerate over the last SLOW_TICKS of a move
const long KP_DIV = 8;                // straightness: 1 speed unit per KP_DIV ticks of left/right mismatch
const long MAX_CORR = 20;
const int MIN_SPEED = 12;
const int MAX_SPEED = 100;
const int DEFAULT_SPEED = 50;

enum Mode : uint8_t { IDLE, DRIVE, ROTATE, MANUAL, CHANNEL, OPENLOOP };

uint8_t leftCh = 0, rightCh = 1;
int8_t leftSign = 1, rightSign = 1;

Mode mode = IDLE;
long targetTicks = 0;
int8_t dirL = 1, dirR = 1;
int cruiseSpeed = 0;
long startL = 0, startR = 0;
unsigned long moveStartMs = 0, lastCmdMs = 0, lastControlMs = 0, stallRefMs = 0;
long stallRefTicks = 0;

char lineBuf[48];
uint8_t lineLen = 0;
bool lineOverflow = false;

// ---- driver I/O ----

bool writeReg(uint8_t reg, const uint8_t *val, uint8_t len) {
  Wire.beginTransmission(DRV_ADDR);
  Wire.write(reg);
  Wire.write(val, len);
  return Wire.endTransmission() == 0;
}

bool readReg(uint8_t reg, uint8_t *val, uint8_t len) {
  Wire.beginTransmission(DRV_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom(DRV_ADDR, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) val[i] = Wire.read();
  return true;
}

bool readRawEncoders(long raw[4]) {
  uint8_t b[16];
  if (!readReg(REG_ENC_TOTAL, b, 16)) return false;
  for (uint8_t i = 0; i < 4; i++) {
    raw[i] = (long)((uint32_t)b[4 * i] | ((uint32_t)b[4 * i + 1] << 8) |
                    ((uint32_t)b[4 * i + 2] << 16) | ((uint32_t)b[4 * i + 3] << 24));
  }
  return true;
}

bool readSides(long &l, long &r) {
  long raw[4];
  if (!readRawEncoders(raw)) return false;
  l = raw[leftCh] * leftSign;
  r = raw[rightCh] * rightSign;
  return true;
}

int clampSpeed(long s) { return (int)constrain(s, -MAX_SPEED, MAX_SPEED); }

bool writeSpeeds(const int8_t s[4]) { return writeReg(REG_SPEED, (const uint8_t *)s, 4); }

bool setSides(long l, long r) {
  int8_t s[4] = {0, 0, 0, 0};
  s[leftCh] = clampSpeed(l) * leftSign;
  s[rightCh] = clampSpeed(r) * rightSign;
  return writeSpeeds(s);
}

bool stopMotors() {
  const int8_t s[4] = {0, 0, 0, 0};
  bool pwmOk = writeReg(REG_PWM, (const uint8_t *)s, 4);
  return writeSpeeds(s) && pwmOk;
}

// ---- motion ----

char modeCode() {
  switch (mode) {
    case DRIVE: return 'D';
    case ROTATE: return 'R';
    case MANUAL: return 'M';
    case OPENLOOP: return 'P';
    default: return 'C';
  }
}

void finishMove(const __FlashStringHelper *reason) {
  stopMotors();
  long l = startL, r = startR;
  readSides(l, r);
  Serial.print(F("DONE "));
  Serial.print(modeCode());
  Serial.print(' ');
  Serial.print(reason);
  Serial.print(' ');
  Serial.print(l - startL);
  Serial.print(' ');
  Serial.println(r - startR);
  mode = IDLE;
}

bool beginMode(Mode m) {
  if (!readSides(startL, startR)) return false;
  mode = m;
  moveStartMs = stallRefMs = lastControlMs = millis();
  stallRefTicks = 0;
  return true;
}

void controlStep(unsigned long now) {
  long l, r;
  if (!readSides(l, r)) { finishMove(F("I2C")); return; }
  long pl = (l - startL) * dirL;
  long pr = (r - startR) * dirR;
  long p = (pl + pr) / 2;
  if (p >= targetTicks) { finishMove(F("TARGET")); return; }

  if (p - stallRefTicks >= STALL_TICKS) {
    stallRefTicks = p;
    stallRefMs = now;
  } else if (now - stallRefMs > STALL_MS) {
    finishMove(F("STALL"));
    return;
  }

  long base = cruiseSpeed;
  unsigned long elapsed = now - moveStartMs;
  if (elapsed < RAMP_MS) base = MIN_SPEED + (long)(cruiseSpeed - MIN_SPEED) * (long)elapsed / (long)RAMP_MS;
  long remaining = targetTicks - p;
  if (remaining < SLOW_TICKS) {
    long slow = MIN_SPEED + (long)(cruiseSpeed - MIN_SPEED) * remaining / SLOW_TICKS;
    if (slow < base) base = slow;
  }
  long corr = constrain((pl - pr) / KP_DIV, -MAX_CORR, MAX_CORR);
  if (!setSides((base - corr) * dirL, (base + corr) * dirR)) finishMove(F("I2C"));
}

// ---- commands ----

void printErr(const __FlashStringHelper *code) {
  Serial.print(F("ERR "));
  Serial.println(code);
}

void startTicksMove(Mode m, long ticks, long speed) {
  if (mode != IDLE) { printErr(F("BUSY")); return; }
  int8_t sign = ticks < 0 ? -1 : 1;
  dirL = sign;
  dirR = (m == ROTATE) ? -sign : sign;
  targetTicks = labs(ticks);
  cruiseSpeed = (int)constrain(labs(speed), (long)MIN_SPEED, (long)MAX_SPEED);
  if (!beginMode(m)) { printErr(F("I2C")); return; }
  Serial.println(m == DRIVE ? F("OK D") : F("OK R"));
  controlStep(millis());
}

void handleLine(char *line) {
  char *cmd = strtok(line, " \t");
  if (!cmd) return;
  for (char *p = cmd; *p; p++) *p = toupper(*p);
  long a[5];
  uint8_t n = 0;
  char *tok;
  while (n < 5 && (tok = strtok(NULL, " \t")) != NULL) a[n++] = atol(tok);

  if (!strcmp(cmd, "H")) return;
  if (!strcmp(cmd, "PING")) { Serial.println(F("PONG dunebroom_motor " FW_VERSION)); return; }

  if (!strcmp(cmd, "S")) {
    if (mode != IDLE) finishMove(F("STOP"));
    else stopMotors();
    Serial.println(F("OK S"));
    return;
  }

  if (!strcmp(cmd, "MAP")) {
    if (n != 4 || a[0] < 0 || a[0] > 3 || a[1] < 0 || a[1] > 3 || a[0] == a[1] ||
        labs(a[2]) != 1 || labs(a[3]) != 1) { printErr(F("ARGS")); return; }
    if (mode != IDLE) { printErr(F("BUSY")); return; }
    leftCh = a[0]; rightCh = a[1]; leftSign = a[2]; rightSign = a[3];
    Serial.println(F("OK MAP"));
    return;
  }

  if (!strcmp(cmd, "D") || !strcmp(cmd, "R")) {
    if (n < 1) { printErr(F("ARGS")); return; }
    startTicksMove(cmd[0] == 'D' ? DRIVE : ROTATE, a[0], n >= 2 ? a[1] : DEFAULT_SPEED);
    return;
  }

  if (!strcmp(cmd, "M")) {
    if (n != 2) { printErr(F("ARGS")); return; }
    if (mode != IDLE && mode != MANUAL) { printErr(F("BUSY")); return; }
    if (a[0] == 0 && a[1] == 0) {
      if (mode == MANUAL) finishMove(F("STOP"));
      else stopMotors();
      Serial.println(F("OK M"));
      return;
    }
    if (mode == IDLE && !beginMode(MANUAL)) { printErr(F("I2C")); return; }
    if (!setSides(a[0], a[1])) { finishMove(F("I2C")); return; }
    Serial.println(F("OK M"));
    return;
  }

  if (!strcmp(cmd, "C")) {
    if (n != 2 || a[0] < 0 || a[0] > 3) { printErr(F("ARGS")); return; }
    if (mode != IDLE && mode != CHANNEL) { printErr(F("BUSY")); return; }
    if (mode == IDLE && !beginMode(CHANNEL)) { printErr(F("I2C")); return; }
    int8_t s[4] = {0, 0, 0, 0};
    s[a[0]] = clampSpeed(a[1]);
    if (!writeSpeeds(s)) { finishMove(F("I2C")); return; }
    Serial.println(F("OK C"));
    return;
  }

  if (!strcmp(cmd, "P")) {
    if (n != 2 || a[0] < 0 || a[0] > 3) { printErr(F("ARGS")); return; }
    if (mode != IDLE && mode != OPENLOOP) { printErr(F("BUSY")); return; }
    if (mode == IDLE && !beginMode(OPENLOOP)) { printErr(F("I2C")); return; }
    int8_t s[4] = {0, 0, 0, 0};
    s[a[0]] = clampSpeed(a[1]);
    if (!writeReg(REG_PWM, (const uint8_t *)s, 4)) { finishMove(F("I2C")); return; }
    Serial.println(F("OK P"));
    return;
  }

  if (!strcmp(cmd, "RD")) {
    if (n != 2 || a[0] < 0 || a[0] > 255 || a[1] < 1 || a[1] > 16) { printErr(F("ARGS")); return; }
    uint8_t b[16];
    if (!readReg((uint8_t)a[0], b, (uint8_t)a[1])) { printErr(F("I2C")); return; }
    Serial.print(F("REG "));
    Serial.print(a[0]);
    for (uint8_t i = 0; i < a[1]; i++) { Serial.print(' '); Serial.print(b[i]); }
    Serial.println();
    return;
  }

  if (!strcmp(cmd, "WR")) {
    if (n < 2 || a[0] < 0 || a[0] > 255) { printErr(F("ARGS")); return; }
    uint8_t b[4];
    for (uint8_t i = 1; i < n; i++) b[i - 1] = (uint8_t)a[i];
    if (!writeReg((uint8_t)a[0], b, n - 1)) { printErr(F("I2C")); return; }
    Serial.println(F("OK WR"));
    return;
  }

  if (!strcmp(cmd, "E")) {
    long l, r;
    if (!readSides(l, r)) { printErr(F("I2C")); return; }
    Serial.print(F("ENC "));
    Serial.print(millis());
    Serial.print(' ');
    Serial.print(l);
    Serial.print(' ');
    Serial.println(r);
    return;
  }

  if (!strcmp(cmd, "X")) {
    long raw[4];
    if (!readRawEncoders(raw)) { printErr(F("I2C")); return; }
    Serial.print(F("RAW "));
    Serial.print(millis());
    for (uint8_t i = 0; i < 4; i++) { Serial.print(' '); Serial.print(raw[i]); }
    Serial.println();
    return;
  }

  if (!strcmp(cmd, "B")) {
    uint8_t v[2];
    if (!readReg(REG_BATTERY, v, 2)) { printErr(F("I2C")); return; }
    Serial.print(F("BAT "));
    Serial.println((unsigned)(v[0] | (v[1] << 8)));
    return;
  }

  printErr(F("UNKNOWN"));
}

void readSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      lastCmdMs = millis();
      if (lineOverflow) printErr(F("TOOLONG"));
      else if (lineLen > 0) handleLine(lineBuf);
      lineLen = 0;
      lineOverflow = false;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineOverflow = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setWireTimeout(3000, true);
  delay(200);
  writeReg(REG_MOTOR_TYPE, &MOTOR_TYPE_JGB37_520_12V_150RPM, 1);
  delay(5);
  const uint8_t polarity = 0;
  writeReg(REG_ENC_POLARITY, &polarity, 1);
  stopMotors();
  Serial.println(F("READY dunebroom_motor " FW_VERSION));
}

void loop() {
  readSerial();
  unsigned long now = millis();
  if (mode != IDLE && now - lastCmdMs > WATCHDOG_MS) {
    finishMove(F("WATCHDOG"));
    return;
  }
  if ((mode == DRIVE || mode == ROTATE) && now - lastControlMs >= CONTROL_MS) {
    lastControlMs = now;
    controlStep(now);
  }
}
