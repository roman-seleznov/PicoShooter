#include <Wire.h>
#include <Mouse.h>

#define MPU6050_ADDR 0x68 // Default I2C address (0x69 if AD0 high)
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_GYRO_XOUT_H 0x43 // Gyro X (roll)
#define MPU6050_GYRO_ZOUT_H 0x47 // Gyro Z (yaw)

#define SENSITIVITY 0.5 // High for fast movement
#define THRESHOLD 0.2 // Low for responsiveness
#define CALIBRATION_SAMPLES 200 // Samples for calibration

bool mpuInitialized = false;
float gyroXOffset = 0, gyroZOffset = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) { delay(10); } // Wait up to 2s for serial

  // Initialize I2C1 on GP2 (SDA) and GP3 (SCL)
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.begin();

  // Initialize MPU6050
  Wire1.beginTransmission(MPU6050_ADDR);
  if (Wire1.endTransmission() == 0) {
    Wire1.beginTransmission(MPU6050_ADDR);
    Wire1.write(MPU6050_PWR_MGMT_1);
    Wire1.write(0x00); // Clear sleep bit
    if (Wire1.endTransmission() == 0) {
      mpuInitialized = true;
    }
  }

  if (!mpuInitialized) {
    Serial.println("MPU6050 initialization failed");
    return;
  }

  Mouse.begin(); // Initialize USB mouse

  // Calibrate gyro X (roll) and Z (yaw)
  int32_t sumX = 0, sumZ = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    Wire1.beginTransmission(MPU6050_ADDR);
    Wire1.write(MPU6050_GYRO_XOUT_H);
    if (Wire1.endTransmission(false) == 0) {
      Wire1.requestFrom(MPU6050_ADDR, 6, true); // Read X, Y, Z
      if (Wire1.available() >= 6) {
        int16_t gyroX = Wire1.read() << 8 | Wire1.read();
        Wire1.read(); Wire1.read(); // Skip Y
        int16_t gyroZ = Wire1.read() << 8 | Wire1.read();
        sumX += gyroX;
        sumZ += gyroZ;
      }
    }
    delay(10);
  }
  gyroXOffset = sumX / (float)CALIBRATION_SAMPLES;
  gyroZOffset = sumZ / (float)CALIBRATION_SAMPLES;
}

void loop() {
  if (!mpuInitialized) {
    Serial.println("MPU6050 not responding");
    delay(500);
    return;
  }

  // Read gyro X (roll) and Z (yaw)
  Wire1.beginTransmission(MPU6050_ADDR);
  Wire1.write(MPU6050_GYRO_XOUT_H);
  if (Wire1.endTransmission(false) == 0) {
    Wire1.requestFrom(MPU6050_ADDR, 6, true);
    if (Wire1.available() >= 6) {
      int16_t gyroX = Wire1.read() << 8 | Wire1.read();
      Wire1.read(); Wire1.read(); // Skip Y
      int16_t gyroZ = Wire1.read() << 8 | Wire1.read();
      float rollRate = (gyroX - gyroXOffset) / 131.0; // ±250 deg/s
      float yawRate = (gyroZ - gyroZOffset) / 131.0;
      if (abs(rollRate) < THRESHOLD) rollRate = 0;
      if (abs(yawRate) < THRESHOLD) yawRate = 0;
      int mouseX = round(-yawRate * SENSITIVITY); // Invert yaw for X movement
      int mouseY = round(rollRate * SENSITIVITY); // Reverse roll for Y movement
      if (mouseX != 0 || mouseY != 0) {
        Mouse.move(mouseX, mouseY);
      }
    }
  }

  delay(10); // Update at ~100Hz
}