/*
  ESP32 sensor collector for Raspberry Pi control.

  Responsibilities:
    - Read MPU6050 accel/gyro at 50 Hz.
    - Poll UWB distances without doing position solve/control decisions.
    - Count wheel encoder ticks in interrupts.
    - Send one compact JSON line at 20 Hz.

  Raspberry Pi should handle position solving, filtering, correction,
  straight-driving decisions, and motor control.
*/

#include <Arduino.h>
#include <Wire.h>

const uint32_t SERIAL_BAUD = 115200;

const uint8_t MPU6050_ADDR = 0x68;
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;
const uint32_t I2C_CLOCK_HZ = 400000;

const uint8_t MPU6050_REG_PWR_MGMT_1 = 0x6B;
const uint8_t MPU6050_REG_ACCEL_XOUT_H = 0x3B;
const uint8_t MPU6050_REG_GYRO_XOUT_H = 0x43;

const uint32_t IMU_PERIOD_MS = 20;      // 50 Hz
const uint32_t JSON_PERIOD_MS = 50;     // 20 Hz
const uint32_t UWB_POLL_PERIOD_MS = 5;  // Non-blocking hook cadence
const uint32_t UWB_STALE_MS = 300;
const uint16_t I2C_TIMEOUT_MS = 2;

// Set these pins to match your encoder wiring.
const int ENC_L_PIN = 34;
const int ENC_R_PIN = 35;

const uint8_t ANCHOR_COUNT = 4;
const float G_TO_MPS2 = 9.80665f;
const float DEG_TO_RAD_F = 0.01745329252f;

struct ImuSample {
  bool ok;
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  uint32_t tMs;
};

struct UwbSample {
  bool ok;
  float x;
  float y;
  float d[ANCHOR_COUNT];
  float rssi[ANCHOR_COUNT];
  uint32_t tMs;
};

ImuSample imu = {};
UwbSample uwb = {};

volatile int32_t encLeftTicks = 0;
volatile int32_t encRightTicks = 0;

uint32_t lastImuReadMs = 0;
uint32_t lastJsonSendMs = 0;
uint32_t lastUwbPollMs = 0;
uint8_t nextAnchorIndex = 0;

void IRAM_ATTR onLeftEncoderTick() {
  encLeftTicks++;
}

void IRAM_ATTR onRightEncoderTick() {
  encRightTicks++;
}

static void initEncoder() {
  if (ENC_L_PIN >= 0) {
    pinMode(ENC_L_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_L_PIN), onLeftEncoderTick, RISING);
  }
  if (ENC_R_PIN >= 0) {
    pinMode(ENC_R_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_R_PIN), onRightEncoderTick, RISING);
  }
}

static bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

static bool readBytes(uint8_t reg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const uint8_t received = Wire.requestFrom(
    static_cast<int>(MPU6050_ADDR),
    static_cast<int>(length),
    static_cast<int>(true)
  );
  if (received != length) {
    const uint8_t pending = Wire.available();
    for (uint8_t i = 0; i < pending; i++) {
      Wire.read();
    }
    return false;
  }

  for (uint8_t i = 0; i < length; i++) {
    buffer[i] = Wire.read();
  }
  return true;
}

static int16_t wordFromBytes(const uint8_t *data, uint8_t index) {
  return static_cast<int16_t>((data[index] << 8) | data[index + 1]);
}

static bool initImu() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_CLOCK_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  return writeRegister(MPU6050_REG_PWR_MGMT_1, 0x00);
}

static void pollImu(uint32_t nowMs) {
  if (nowMs - lastImuReadMs < IMU_PERIOD_MS) {
    return;
  }
  lastImuReadMs = nowMs;

  uint8_t accelData[6] = {};
  uint8_t gyroData[6] = {};
  const bool accelOk = readBytes(MPU6050_REG_ACCEL_XOUT_H, accelData, sizeof(accelData));
  const bool gyroOk = readBytes(MPU6050_REG_GYRO_XOUT_H, gyroData, sizeof(gyroData));

  if (!accelOk || !gyroOk) {
    imu.ok = false;
    return;
  }

  const int16_t rawAx = wordFromBytes(accelData, 0);
  const int16_t rawAy = wordFromBytes(accelData, 2);
  const int16_t rawAz = wordFromBytes(accelData, 4);
  const int16_t rawGx = wordFromBytes(gyroData, 0);
  const int16_t rawGy = wordFromBytes(gyroData, 2);
  const int16_t rawGz = wordFromBytes(gyroData, 4);

  imu.ax = (rawAx / 16384.0f) * G_TO_MPS2;
  imu.ay = (rawAy / 16384.0f) * G_TO_MPS2;
  imu.az = (rawAz / 16384.0f) * G_TO_MPS2;
  imu.gx = (rawGx / 131.0f) * DEG_TO_RAD_F;
  imu.gy = (rawGy / 131.0f) * DEG_TO_RAD_F;
  imu.gz = (rawGz / 131.0f) * DEG_TO_RAD_F;
  imu.tMs = nowMs;
  imu.ok = true;
}

/*
  UWB hooks.

  Keep these functions strictly non-blocking:
    - Check an IRQ flag, status register, or library "available()" flag.
    - Parse only when a complete frame is already available.
    - Return false immediately when there is no fresh UWB frame.

  Do not put delay(), while waiting, or blocking read/ranging calls here.
*/
static bool readUwbDistanceNonBlocking(uint8_t anchorId, float *distanceM, float *rxPowerDbm) {
  (void)anchorId;
  *distanceM = NAN;
  *rxPowerDbm = NAN;
  return false;
}

static bool readUwbPositionNonBlocking(float *xM, float *yM) {
  *xM = NAN;
  *yM = NAN;
  return false;
}

static void initUwb() {
  for (uint8_t i = 0; i < ANCHOR_COUNT; i++) {
    uwb.d[i] = NAN;
    uwb.rssi[i] = NAN;
  }
  uwb.x = NAN;
  uwb.y = NAN;
  uwb.ok = false;
}

static void pollUwbNonBlocking(uint32_t nowMs) {
  if (nowMs - lastUwbPollMs < UWB_POLL_PERIOD_MS) {
    return;
  }
  lastUwbPollMs = nowMs;

  const uint8_t anchorId = nextAnchorIndex + 1;
  float distanceM = NAN;
  float rxPowerDbm = NAN;
  float xM = NAN;
  float yM = NAN;

  if (readUwbPositionNonBlocking(&xM, &yM)) {
    uwb.x = xM;
    uwb.y = yM;
    uwb.tMs = nowMs;
    uwb.ok = true;
  }

  if (readUwbDistanceNonBlocking(anchorId, &distanceM, &rxPowerDbm)) {
    uwb.d[nextAnchorIndex] = distanceM;
    uwb.rssi[nextAnchorIndex] = rxPowerDbm;
    uwb.tMs = nowMs;
    uwb.ok = true;
  }

  if (uwb.ok && nowMs - uwb.tMs > UWB_STALE_MS) {
    uwb.ok = false;
  }

  nextAnchorIndex = (nextAnchorIndex + 1) % ANCHOR_COUNT;
}

static void printJsonFloat(const char *key, float value, uint8_t precision, bool comma = true) {
  Serial.print('"');
  Serial.print(key);
  Serial.print("\":");
  if (isfinite(value)) {
    Serial.print(value, precision);
  } else {
    Serial.print("null");
  }
  if (comma) {
    Serial.print(',');
  }
}

static void sendJson(uint32_t nowMs) {
  if (nowMs - lastJsonSendMs < JSON_PERIOD_MS) {
    return;
  }
  lastJsonSendMs = nowMs;

  int32_t leftTicks = 0;
  int32_t rightTicks = 0;
  noInterrupts();
  leftTicks = encLeftTicks;
  rightTicks = encRightTicks;
  interrupts();

  Serial.print('{');
  Serial.print("\"t\":");
  Serial.print(nowMs);
  Serial.print(',');
  printJsonFloat("uwb_x", uwb.x, 3);
  printJsonFloat("uwb_y", uwb.y, 3);
  printJsonFloat("uwb_d1", uwb.d[0], 3);
  printJsonFloat("uwb_d2", uwb.d[1], 3);
  printJsonFloat("uwb_d3", uwb.d[2], 3);
  printJsonFloat("uwb_d4", uwb.d[3], 3);
  Serial.print("\"uwb_t\":");
  Serial.print(uwb.tMs);
  Serial.print(',');
  printJsonFloat("imu_ax", imu.ok ? imu.ax : NAN, 4);
  printJsonFloat("imu_ay", imu.ok ? imu.ay : NAN, 4);
  printJsonFloat("imu_az", imu.ok ? imu.az : NAN, 4);
  printJsonFloat("imu_gx", imu.ok ? imu.gx : NAN, 6);
  printJsonFloat("imu_gy", imu.ok ? imu.gy : NAN, 6);
  printJsonFloat("imu_gz", imu.ok ? imu.gz : NAN, 6);
  Serial.print("\"imu_t\":");
  Serial.print(imu.tMs);
  Serial.print(',');
  Serial.print("\"enc_l\":");
  Serial.print(leftTicks);
  Serial.print(',');
  Serial.print("\"enc_r\":");
  Serial.print(rightTicks);
  Serial.print(',');
  Serial.print("\"imu_ok\":");
  Serial.print(imu.ok ? "true" : "false");
  Serial.print(',');
  Serial.print("\"uwb_ok\":");
  Serial.print(uwb.ok ? "true" : "false");
  Serial.println('}');
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  imu.ok = initImu();
  initEncoder();
  initUwb();
}

void loop() {
  const uint32_t nowMs = millis();

  // UWB is checked first and must return immediately when no fresh data exists.
  pollUwbNonBlocking(nowMs);

  // MPU6050 and JSON output are independent timed tasks; either can fail
  // without stopping the other task or the UWB polling path.
  pollImu(nowMs);
  sendJson(nowMs);
}
