// Serial-command motor control for DuneBroom.
// Same I2C motor driver setup as the DuneBroom_Code sketches, but the robot
// stays STOPPED until told to move.
// Commands (9600 baud, newline terminated):
//   F<ms>  forward for <ms> milliseconds, then stop   e.g. F500
//   B<ms>  backward for <ms> milliseconds, then stop
//   L<ms>  turn left for <ms> milliseconds, then stop
//   S      stop now
#include <Wire.h>

#define I2C_ADDR                     0x34
#define MOTOR_TYPE_ADDR              0x14
#define MOTOR_ENCODER_POLARITY_ADDR  0x15
#define MOTOR_FIXED_SPEED_ADDR       0x33
#define MOTOR_TYPE_JGB37_3865_520_12V_150RPM 3

uint8_t MotorType = MOTOR_TYPE_JGB37_3865_520_12V_150RPM;
uint8_t MotorEncoderPolarity = 0;
int8_t stopCmd[4]     = {0, 0, 0, 0};
int8_t forwardCmd[4]  = {50, 50, 50, 50};
int8_t backwardCmd[4] = {-50, -50, 0, 0};
int8_t turnLeftCmd[4] = {-30, 30, 0, 0};

bool writeReg(uint8_t reg, const int8_t *val, unsigned int len) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  for (unsigned int i = 0; i < len; i++) Wire.write((uint8_t)val[i]);
  return Wire.endTransmission() == 0;
}

int readReg(uint8_t reg, uint8_t *val, unsigned int len) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return -1;
  Wire.requestFrom((uint8_t)I2C_ADDR, (uint8_t)len);
  unsigned int i = 0;
  for (; i < len && Wire.available(); i++) val[i] = Wire.read();
  return i;
}

void printDiag() {
  uint8_t v[2] = {0, 0};
  int n = readReg(0x00, v, 2);
  Serial.print("BAT_mV "); Serial.print(n == 2 ? (int)(v[0] | (v[1] << 8)) : -1);
  int32_t enc[4] = {0, 0, 0, 0};
  n = readReg(0x3C, (uint8_t *)enc, 16);
  Serial.print(" ENC");
  for (int i = 0; i < 4; i++) { Serial.print(' '); Serial.print(enc[i]); }
  Serial.println();
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  delay(200);
  writeReg(MOTOR_TYPE_ADDR, (int8_t *)&MotorType, 1);
  delay(5);
  writeReg(MOTOR_ENCODER_POLARITY_ADDR, (int8_t *)&MotorEncoderPolarity, 1);
  writeReg(MOTOR_FIXED_SPEED_ADDR, stopCmd, 4);
  Serial.println("READY");
}

void runFor(const int8_t *cmd, long ms) {
  if (ms < 0) ms = 0;
  if (ms > 10000) ms = 10000;  // safety cap
  bool ok = writeReg(MOTOR_FIXED_SPEED_ADDR, cmd, 4);
  delay(ms);
  writeReg(MOTOR_FIXED_SPEED_ADDR, stopCmd, 4);
  Serial.print(ok ? "DONE " : "I2C_ERR ");
  Serial.println(ms);
}

void loop() {
  if (Serial.available() > 0) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();
    if (msg.length() == 0) return;
    char c = toupper(msg.charAt(0));
    long ms = msg.substring(1).toInt();
    if (c == 'F') runFor(forwardCmd, ms);
    else if (c == 'B') runFor(backwardCmd, ms);
    else if (c == 'L') runFor(turnLeftCmd, ms);
    else if (c == 'V') printDiag();
    else if (c == 'P') {  // raw PWM test: P<ms>, duty 60% on M1/M2
      int8_t pwm[4] = {60, 60, 60, 60};
      writeReg(0x1F, pwm, 4); delay(ms > 3000 ? 3000 : ms);
      writeReg(0x1F, stopCmd, 4); writeReg(MOTOR_FIXED_SPEED_ADDR, stopCmd, 4);
      Serial.println("PWM_DONE");
    }
    else if (c == 'S') { writeReg(MOTOR_FIXED_SPEED_ADDR, stopCmd, 4); Serial.println("STOPPED"); }
    else Serial.println("UNKNOWN");
  }
}
