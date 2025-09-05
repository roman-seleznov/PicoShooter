#include <Wire.h>
#include <Mouse.h>
#include <Keyboard.h> // Kept for potential UART commands

// Configuration Constants
const uint8_t MPU6050_ADDRESS = 0x68;         // MPU6050 I2C address
const uint8_t MPU6050_PWR_MGMT_1 = 0x6B;      // Power management register
const uint8_t MPU6050_GYRO_XOUT_H = 0x43;     // Gyro X-axis high byte
const uint8_t I2C_ADDRESS = 0x40;             // M5Stack Scroll Unit I2C address
const uint8_t SDA_PIN = 2;                    // GP2 for SDA
const uint8_t SCL_PIN = 3;                    // GP3 for SCL
const uint8_t ENCODER_REGISTER = 0x10;        // Register for 4-byte encoder value
const uint8_t BUTTON_REGISTER = 0x20;         // Register for 1-byte button state
const uint8_t LED_REGISTER = 0x30;            // Register for 3-byte RGB LED control
const uint8_t TOUCH_PIN = 16;                 // GP20 for TTP223 touch sensor
const float SENSITIVITY = 0.5;                // Mouse movement sensitivity
const float THRESHOLD = 0.5;                  // Gyro threshold to ignore noise
const uint16_t BLINK_DURATION_MS = 200;       // Blink on/off duration in milliseconds
const uint8_t BLINK_CYCLES = 2;               // Number of on/off cycles (800ms total)
const uint32_t CALIBRATION_SAMPLES = 200;     // Samples for gyro calibration
const uint32_t PRE_CALIBRATION_DELAY_MS = 5000; // 5-second delay before calibration
const uint32_t POST_CALIBRATION_GREEN_MS = 1000; // 5-second green LED after calibration

// State Variables
int32_t previousEncoderValue = 0;             // Last encoder value
bool previousButtonState = false;             // Last button state
unsigned long lastEventTime = 0;              // Time of last event
bool ledActive = false;                       // LED blinking state
uint8_t ledColor[3] = {0, 0, 0};              // RGB array [Red, Green, Blue]
bool mpuInitialized = false;                  // MPU6050 initialization status
float gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0; // Gyro calibration offsets
bool mouseEnabled = true;                     // Mouse enable state
String serialBuffer = "";                     // Buffer for UART input

// Initialize Hardware
void setup() {
  Serial.begin(115200);                       // Start Serial for minimal feedback
  delay(2000);                                // Allow Serial to connect
  Serial.println("Scroll Unit and Gyro HID Started");

  configureI2C();                             // Set up I2C pins and bus
  indicatePreCalibration();                   // Blink red LED for 5 seconds
  initializeMPU6050();                        // Set up MPU6050
  pinMode(TOUCH_PIN, INPUT);                  // Configure touch pin
  Mouse.begin();                              // Initialize HID mouse
  Keyboard.begin();                           // Initialize keyboard for UART
  indicatePostCalibration();                  // Turn on green LED for 5 seconds
}

// Main Program Loop
void loop() {
  if (!mpuInitialized) {
    Serial.println("MPU not initialized");
    delay(500);                               // Wait if MPU6050 fails
    return;
  }

  updateTouchState();                         // Toggle mouse enable with touch
  processUARTInput();                         // Handle UART commands
  moveMouseWithGyro();                        // Control mouse with gyro
  processScroll();                            // Handle encoder-based scrolling
  processClick();                             // Handle button-based middle press/release
  manageLed();                                // Control LED blinking
  delay(2);                                   // Short delay for stability
}

// Configure I2C Communication
void configureI2C() {
  Wire1.setSDA(SDA_PIN);
  Wire1.setSCL(SCL_PIN);
  Wire1.begin();
}

// Initialize MPU6050
void initializeMPU6050() {
  Wire1.beginTransmission(MPU6050_ADDRESS);
  if (Wire1.endTransmission() == 0) {
    Wire1.beginTransmission(MPU6050_ADDRESS);
    Wire1.write(MPU6050_PWR_MGMT_1);
    Wire1.write(0x00);                        // Wake up MPU6050
    if (Wire1.endTransmission() == 0) {
      mpuInitialized = true;
      calibrateGyro();                        // Calibrate gyro offsets
    }
  }
  if (!mpuInitialized) {
    Serial.println("MPU6050 initialization failed");
  }
}

// Calibrate Gyroscope Offsets with Blue Blink
void calibrateGyro() {
  int32_t sumX = 0, sumY = 0, sumZ = 0;
  for (uint32_t i = 0; i < CALIBRATION_SAMPLES; i++) {
    int16_t gyroX, gyroY, gyroZ;
    readGyroData(&gyroX, &gyroY, &gyroZ);
    sumX += gyroX;
    sumY += gyroY;
    sumZ += gyroZ;
    // Blink blue during calibration
    if (i % 10 == 0) {
      setLedColor(0, 0, 255); // Blue on
      delay(100);
      setLedColor(0, 0, 0);   // Blue off
    }
    delay(10);
  }
  gyroXOffset = sumX / (float)CALIBRATION_SAMPLES;
  gyroYOffset = sumY / (float)CALIBRATION_SAMPLES;
  gyroZOffset = sumZ / (float)CALIBRATION_SAMPLES;
}

// Read Gyro Data from MPU6050
void readGyroData(int16_t* x, int16_t* y, int16_t* z) {
  Wire1.beginTransmission(MPU6050_ADDRESS);
  Wire1.write(MPU6050_GYRO_XOUT_H);
  Wire1.endTransmission(false);
  Wire1.requestFrom(MPU6050_ADDRESS, 6);
  if (Wire1.available() >= 6) {
    *x = (Wire1.read() << 8) | Wire1.read();
    *y = (Wire1.read() << 8) | Wire1.read();
    *z = (Wire1.read() << 8) | Wire1.read();
  }
}

// Move Mouse with Gyro Data
void moveMouseWithGyro() {
  if (!mouseEnabled) return;

  int16_t gyroX, gyroY, gyroZ;
  readGyroData(&gyroX, &gyroY, &gyroZ);
  float rollRate = (gyroX - gyroXOffset) / 131.0;
  float pitchRate = (gyroY - gyroYOffset) / 131.0;
  float yawRate = (gyroZ - gyroZOffset) / 131.0;

  if (abs(rollRate) < THRESHOLD) rollRate = 0;
  if (abs(yawRate) < THRESHOLD) yawRate = 0;

  int mouseX = round(-yawRate * SENSITIVITY);   // Horizontal movement
  int mouseY = round(rollRate * SENSITIVITY);   // Vertical movement (inverted)
  if (mouseX != 0 || mouseY != 0) {
    Mouse.move(mouseX, mouseY);
  }
}

// Update Touch Sensor State
void updateTouchState() {
  bool currentTouch = digitalRead(TOUCH_PIN);
  static bool lastTouch = false;
  if (currentTouch && !lastTouch) {
    mouseEnabled = false;
    Serial.println("Mouse Disabled");
  } else if (!currentTouch && lastTouch) {
    mouseEnabled = true;
    Serial.println("Mouse Enabled");
  }
  lastTouch = currentTouch;
}

// Process UART Input from Serial1
void processUARTInput() {
  while (Serial1.available()) {
    char c = Serial1.read();
    serialBuffer += c;
    if (c == ']') {
      serialBuffer.trim();
      if (serialBuffer == "[TOUCH ON]") {
        mouseEnabled = false;
        Serial.println("Mouse Disabled via UART");
      } else if (serialBuffer == "[TOUCH OFF]") {
        mouseEnabled = true;
        Serial.println("Mouse Enabled via UART");
      } else if (serialBuffer == "[R]") {
        Keyboard.press('R');
        delay(10);
        Keyboard.release('R');
        Serial.println("R Key Pressed");
      }
      serialBuffer = "";
    }
  }
}

// Process Encoder for Scrolling
void processScroll() {
  int32_t currentValue = readEncoderValue();
  if (currentValue != previousEncoderValue) {
    if (!mouseEnabled) {
      mouseEnabled = true;                    // Re-enable mouse on scroll
      Serial.println("Mouse Enabled via Scroll");
    }
    int8_t scrollDirection = (currentValue > previousEncoderValue) ? 1 : -1; // 1 for up, -1 for down
    Mouse.move(0, 0, scrollDirection);              // Scroll mouse
    setBlinkColor(scrollDirection > 0 ? 0 : 255,    // Green for up
                  scrollDirection > 0 ? 255 : 0,    // Blue for down
                  0);                               // No red
    previousEncoderValue = currentValue;
  }
}

// Process Button for Middle Press/Release
void processClick() {
  bool currentState = readButtonState();
    if(previousButtonState != currentState) {
      if (currentState == 1) { // Button pressed
        Mouse.press(MOUSE_MIDDLE);                       // Trigger press event
        setLedColor(0, 0, 255);
      } else { // Button released
        Mouse.release(MOUSE_MIDDLE);                     // Trigger release event
        setBlinkColor(0, 0, 0);
      }
  }
  previousButtonState = currentState;
}

// Manage LED Blinking
void manageLed() {
  unsigned long currentTime = millis();
  if (ledActive && (currentTime - lastEventTime >= BLINK_DURATION_MS)) {
    if (currentTime - lastEventTime < BLINK_CYCLES * BLINK_DURATION_MS) {
      setLedColor(0, 0, 0); // Turn off LED
    } else {
      ledActive = false;    // Reset after cycles
      setLedColor(0, 0, 0); // Ensure LED is off
    }
  }
}

// Read Encoder Value from Register 0x10
int32_t readEncoderValue() {
  Wire1.beginTransmission(I2C_ADDRESS);
  Wire1.write(ENCODER_REGISTER);
  Wire1.endTransmission();
  Wire1.requestFrom(I2C_ADDRESS, 4);
  int32_t value = 0;
  if (Wire1.available() >= 4) {
    value |= Wire1.read();       // Least significant byte
    value |= Wire1.read() << 8;
    value |= Wire1.read() << 16;
    value |= Wire1.read() << 24; // Most significant byte
  }
  return value;
}

// Read Button State from Register 0x20
bool readButtonState() {
  Wire1.beginTransmission(I2C_ADDRESS);
  Wire1.write(BUTTON_REGISTER);
  Wire1.endTransmission();
  Wire1.requestFrom(I2C_ADDRESS, 1);
  if (Wire1.available() >= 1) {
    return Wire1.read() == 0; // True if pressed (0), per protocol
  }
  return previousButtonState; // Fallback if no data
}

// Set Blink Color and Start Timer
void setBlinkColor(uint8_t red, uint8_t green, uint8_t blue) {
  ledColor[0] = red;    // Red component
  ledColor[1] = green;  // Green component
  ledColor[2] = blue;   // Blue component
  lastEventTime = millis();
  ledActive = true;
  setLedColor(red, green, blue); // Turn on immediately
}

// Set LED Color at Register 0x30
void setLedColor(uint8_t red, uint8_t green, uint8_t blue) {
  Wire1.beginTransmission(I2C_ADDRESS);
  Wire1.write(LED_REGISTER);
  Wire1.write(0);       // NULL byte as per protocol
  Wire1.write(red);     // Red
  Wire1.write(green);   // Green
  Wire1.write(blue);    // Blue
  Wire1.endTransmission();
}

// Indicate Pre-Calibration with Red Blink
void indicatePreCalibration() {
  unsigned long startTime = millis();
  while (millis() - startTime < PRE_CALIBRATION_DELAY_MS) {
    setLedColor(255, 0, 0); // Red on
    delay(BLINK_DURATION_MS);
    setLedColor(0, 0, 0);   // Red off
    delay(BLINK_DURATION_MS);
  }
}

// Indicate Post-Calibration with Green On
void indicatePostCalibration() {
  setLedColor(0, 255, 0); // Green on
  delay(POST_CALIBRATION_GREEN_MS); // 5 seconds
  setLedColor(0, 0, 0);   // Green off
}