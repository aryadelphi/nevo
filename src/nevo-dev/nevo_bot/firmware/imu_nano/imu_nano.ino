/*
  MPU6050 -> Serial (Arduino Nano)
  No external libraries required (uses Wire only).

  Wiring (Nano):
    MPU6050 VCC -> 5V (most GY-521 boards have a regulator) or 3.3V
    MPU6050 GND -> GND
    MPU6050 SDA -> A4
    MPU6050 SCL -> A5
    MPU6050 AD0 -> GND (address 0x68) or VCC (address 0x69)

  Output (115200 baud), CSV per line with NMEA-style checksum:
    ax,ay,az (g), gx,gy,gz (deg/s), temp (C)*HH
  HH = XOR of all characters before '*', two hex digits. Lines starting
  with '#' are comments (no checksum).
*/

#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;         // 0x69 if AD0 is HIGH
const unsigned long SAMPLE_MS = 10;    // 100 Hz output rate

// Scale factors for the ranges configured below
const float ACCEL_SCALE = 16384.0;     // LSB/g   for +/-2 g
const float GYRO_SCALE  = 131.0;       // LSB/dps for +/-250 dps

float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

// Reads all 14 data bytes in one burst: accel(6) temp(2) gyro(6)
bool readRaw(int16_t &ax, int16_t &ay, int16_t &az,
             int16_t &t,
             int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                    // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  t  = (Wire.read() << 8) | Wire.read();
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
  return true;
}

void calibrateGyro(int samples) {
  long sx = 0, sy = 0, sz = 0;
  int16_t ax, ay, az, t, gx, gy, gz;
  int n = 0;
  for (int i = 0; i < samples; i++) {
    if (readRaw(ax, ay, az, t, gx, gy, gz)) {
      sx += gx; sy += gy; sz += gz;
      n++;
    }
    delay(3);
  }
  if (n > 0) {
    gyroBiasX = (float)sx / n;
    gyroBiasY = (float)sy / n;
    gyroBiasZ = (float)sz / n;
  }
}

void startI2C() {
  Wire.begin();
  Wire.setClock(400000);               // 400 kHz I2C
  Wire.setWireTimeout(3000, true);     // 3 ms: a stuck bus returns an error instead of hanging
}

void configureMpu() {
  writeReg(0x6B, 0x01);                // PWR_MGMT_1: wake up, PLL with X gyro clock
  delay(50);
  writeReg(0x1A, 0x03);                // CONFIG: DLPF ~44 Hz
  writeReg(0x19, 0x04);                // SMPLRT_DIV: 1 kHz / (1+4) = 200 Hz
  writeReg(0x1B, 0x00);                // GYRO_CONFIG:  +/-250 dps
  writeReg(0x1C, 0x00);                // ACCEL_CONFIG: +/-2 g
}

// A slave interrupted mid-transfer (glitch, loose wire) can hold SDA low forever.
// Clock SCL until it lets go, then send a STOP.
void clearBus() {
  Wire.end();
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, OUTPUT);
  for (uint8_t i = 0; i < 9 && digitalRead(SDA) == LOW; i++) {
    digitalWrite(SCL, LOW);  delayMicroseconds(5);
    digitalWrite(SCL, HIGH); delayMicroseconds(5);
  }
  pinMode(SDA, OUTPUT);
  digitalWrite(SDA, LOW);  delayMicroseconds(5);
  digitalWrite(SCL, HIGH); delayMicroseconds(5);
  digitalWrite(SDA, HIGH); delayMicroseconds(5);
  pinMode(SDA, INPUT);
  pinMode(SCL, INPUT);
}

void setup() {
  Serial.begin(115200);
  startI2C();
  delay(100);
  configureMpu();
  delay(50);

  uint8_t who = readReg(0x75);         // WHO_AM_I, genuine chip returns 0x68
  while (who == 0xFF) {                // keep retrying instead of hanging until a power cycle
    Serial.println(F("# MPU6050 not found - check wiring/address"));
    delay(1000);
    clearBus();
    startI2C();
    configureMpu();
    who = readReg(0x75);
  }
  Serial.print(F("# WHO_AM_I = 0x"));
  Serial.println(who, HEX);            // clones may report 0x70/0x72/0x98; usually still work

  Serial.println(F("# Calibrating gyro, keep sensor still..."));
  calibrateGyro(500);
  Serial.println(F("# ax,ay,az,gx,gy,gz,temp*checksum"));
}

void loop() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < SAMPLE_MS) return;
  last += SAMPLE_MS;                   // fixed 100 Hz grid
  if (now - last >= SAMPLE_MS) last = now;  // fell behind (e.g. after setup): resync, no burst

  // 2026-10-19: the MPU stopped answering mid-run (loose wire / vibration) and the old
  // code printed "# read error" forever. Now: after 10 failed reads, clear the bus and
  // re-configure the MPU, retrying every 0.5 s. The gyro bias is kept (no recalibration:
  // the robot may be moving).
  static uint16_t errors = 0;
  static unsigned long lastReinit = 0;
  int16_t ax, ay, az, t, gx, gy, gz;
  bool ok = readRaw(ax, ay, az, t, gx, gy, gz);
  if (ok && ax == 0 && ay == 0 && az == 0) ok = false;  // MPU power-cycled: asleep, reads zeros
  if (!ok) {
    errors++;
    if (errors == 1 || errors % 100 == 0) {
      Serial.print(F("# read error x"));
      Serial.println(errors);
    }
    if (errors >= 10 && now - lastReinit >= 500) {
      lastReinit = now;
      clearBus();
      startI2C();
      configureMpu();
      Serial.println(F("# i2c reinit (gyro bias kept)"));
    }
    return;
  }
  if (errors) {
    Serial.print(F("# recovered after "));
    Serial.print(errors);
    Serial.println(F(" read errors"));
    errors = 0;
  }

  // AVR printf has no %f, so format each value with dtostrf
  const float v[7] = { ax / ACCEL_SCALE, ay / ACCEL_SCALE, az / ACCEL_SCALE,
                       (gx - gyroBiasX) / GYRO_SCALE, (gy - gyroBiasY) / GYRO_SCALE,
                       (gz - gyroBiasZ) / GYRO_SCALE, t / 340.0 + 36.53 };
  const uint8_t prec[7] = {3, 3, 3, 2, 2, 2, 2};
  char line[96], num[16];
  line[0] = '\0';
  for (uint8_t i = 0; i < 7; i++) {
    dtostrf(v[i], 1, prec[i], num);
    if (i) strcat(line, ",");
    strcat(line, num);
  }

  uint8_t cs = 0;
  for (const char *p = line; *p; p++) cs ^= *p;
  Serial.print(line);
  Serial.print('*');
  if (cs < 0x10) Serial.print('0');
  Serial.println(cs, HEX);
}
