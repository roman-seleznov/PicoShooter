#include <Wire.h>
#include <Mouse.h>
#include <Keyboard.h>

#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_ZOUT_H 0x47
#define SENSITIVITY 0.5
#define THRESHOLD 2.5
#define CALIBRATION_SAMPLES 200

#define TOUCH_PIN 20 // GP20 for TTP223 OUT

bool mpuInitialized = false;
float gyroXOffset = 0, gyroZOffset = 0;
bool mouseEnabled = true; // Flag to control mouse movement
String serialBuffer = ""; // Buffer for UART messages

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) { delay(10); }

  delay(1000);


  Serial1.begin(115200, SERIAL_8N1);

  // Initialize I2C1 on GP2 (SDA) and GP3 (SCL)
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  Wire1.begin();

  // Initialize MPU6050
  Wire1.beginTransmission(MPU6050_ADDR);
  if (Wire1.endTransmission() == 0) {
    Wire1.beginTransmission(MPU6050_ADDR);
    Wire1.write(MPU6050_PWR_MGMT_1);
    Wire1.write(0x00);
    if (Wire1.endTransmission() == 0) {
      mpuInitialized = true;
    }
  }

  if (!mpuInitialized) {
    Serial.println("MPU6050 initialization failed");
    return;
  }

   // Touch sensor setup
  pinMode(TOUCH_PIN, INPUT);

  Mouse.begin();
  Keyboard.begin(); // Initialize USB keyboard

  // Calibrate gyro X (roll) and Z (yaw)
  int32_t sumX = 0, sumZ = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    Wire1.beginTransmission(MPU6050_ADDR);
    Wire1.write(MPU6050_GYRO_XOUT_H);
    if (Wire1.endTransmission(false) == 0) {
      Wire1.requestFrom(MPU6050_ADDR, 6, true);
      if (Wire1.available() >= 6) {
        int16_t gyroX = Wire1.read() << 8 | Wire1.read();
        Wire1.read(); Wire1.read();
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

   // Read touch sensor (momentary mode)
  bool touchState = digitalRead(TOUCH_PIN);
  if (touchState == HIGH) {
    if (mouseEnabled) {
      mouseEnabled = false; // Disable mouse while holding touch
      Serial.println("DISABLE MOUSE");
    }
  } else {
    if (!mouseEnabled) {
      mouseEnabled = true; // Enable mouse when released
      Serial.println("ENABLE MOUSE");
    }
  }

  // Handle UART input
  while (Serial1.available()) {
    char c = Serial1.read();
    serialBuffer += c;
    if (c == '\n') {
      serialBuffer.trim();
      if (serialBuffer == "[TOUCH ON]") {
        Serial.println("TOUCH ON");
      } else if (serialBuffer == "[TOUCH OFF]") {
        Serial.println("TOUCH OFF");
      } else if (serialBuffer == "[R]") {
        Keyboard.press('R');
        delay(10);
        Keyboard.release('R');
      }
      serialBuffer = "";
    }
  }

  // Read gyro data only if mouse is enabled
  if (mouseEnabled) {
    Wire1.beginTransmission(MPU6050_ADDR);
    Wire1.write(MPU6050_GYRO_XOUT_H);
    if (Wire1.endTransmission(false) == 0) {
      Wire1.requestFrom(MPU6050_ADDR, 6, true);
      if (Wire1.available() >= 6) {
        int16_t gyroX = Wire1.read() << 8 | Wire1.read();
        Wire1.read(); Wire1.read();
        int16_t gyroZ = Wire1.read() << 8 | Wire1.read();
        float rollRate = (gyroX - gyroXOffset) / 131.0;
        float yawRate = (gyroZ - gyroZOffset) / 131.0;

        if (abs(rollRate) < THRESHOLD) rollRate = 0;
        if (abs(yawRate) < THRESHOLD) yawRate = 0;

        int mouseX = round(-yawRate * SENSITIVITY);
        int mouseY = round(rollRate * SENSITIVITY);

        if (mouseX != 0 || mouseY != 0) {
          Mouse.move(mouseX, mouseY);
        }
      }
    }
  }

  delay(2); //Update at 500 Hz
}