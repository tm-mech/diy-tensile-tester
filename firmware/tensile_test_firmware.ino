/*
 * Tensile Test Firmware
 * 
 * Controls a stepper-driven tensile testing machine with force measurement,
 * accelerometer-based step loss detection, and safety systems.
 * 
 * SERIAL COMMANDS (from Python host):
 *   START         - Start tensile test (motion + data acquisition)
 *   STOP          - Emergency stop
 *   SET_SPEED:xx  - Set speed in mm/min (e.g. SET_SPEED:5)
 *   SET_DIR:x     - Set direction (1 = pull, -1 = return)
 *   TARE          - Zero force measurement
 *   STATUS        - Return current state
 *   RESET         - Reset to IDLE
 *   FORCE         - Return current force reading
 *   UP / DOWN     - Manual jog for specimen positioning
 * 
 * SERIAL OUTPUT (to Python host):
 *   DATA;millis;steps;raw_force;ax;ay;az;endstop;step_loss
 *   STATUS;state;speed_mm_min;direction
 *   EVENT;event_name
 */

#include "HX711.h"
#include <Wire.h>

// =============================================================================
// PIN CONFIGURATION
// =============================================================================

// Stepper motor
const int PIN_STEP = 9;
const int PIN_DIR  = 6;

// HX711 load cell ADC
const int PIN_HX711_DT  = 2;
const int PIN_HX711_SCK = 3;

// Endstop
const int PIN_ENDSTOP = 8;

// ADXL345 accelerometer (I2C)
const int ADXL345_ADDR = 0x53;

// =============================================================================
// MACHINE PARAMETERS
// =============================================================================

const float LEAD_MM        = 4.0;              // Ball screw pitch [mm]
const int   STEPS_PER_REV  = 200;              // Motor full steps per revolution
const int   MICROSTEPS     = 4;                // Microstepping factor
const float STEPS_PER_MM   = (STEPS_PER_REV * MICROSTEPS) / LEAD_MM;

const int   FORCE_LIMIT_N  = 800;              // Force limit [N]
const float FORCE_CALIBRATION = 2.217E-04;     // Raw ADC → Newton conversion factor
const long  FORCE_LIMIT_RAW = abs(FORCE_LIMIT_N / FORCE_CALIBRATION);

// =============================================================================
// STATE MACHINE
// =============================================================================

enum State {
  STATE_IDLE,
  STATE_RUNNING,
  STATE_STOPPED,
  STATE_ERROR,
  STATE_JOG
};

State currentState = STATE_IDLE;

// =============================================================================
// MOTION VARIABLES
// =============================================================================

float targetSpeed_mm_min = 2;     // Test speed [mm/min]
float jogSpeed_mm_min    = 120;   // Manual jog speed [mm/min]
int   moveDirection      = 1;     // 1 = pull, -1 = return
int   jogDirection       = 1;     // 1 = up, -1 = down

unsigned long stepInterval_us = 0;
unsigned long jogInterval_us  = 0;
unsigned long lastStepTime_us = 0;
long totalSteps = 0;

// =============================================================================
// SENSOR VARIABLES
// =============================================================================

HX711 scale;
long forceOffset    = 0;    // Tare value
long lastValidForce = 0;    // Most recent valid force reading

int16_t ax, ay, az;         // Accelerometer axes

// Step loss detection state
bool stepLossDetected    = false;
bool stepLossCheckReady  = false;
unsigned long runStartTime = 0;

// Over-force protection: require 3 consecutive readings above limit
int overLimitCount = 0;

// =============================================================================
// TIMING
// =============================================================================

const int DATA_RATE_HZ = 10;
const unsigned long DATA_INTERVAL_MS = 1000 / DATA_RATE_HZ;
unsigned long lastDataTime = 0;

// Serial input buffer
String inputBuffer = "";

// =============================================================================
// SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);

  // Stepper pins
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  digitalWrite(PIN_DIR, HIGH);

  // Endstop (active low, internal pullup)
  pinMode(PIN_ENDSTOP, INPUT_PULLUP);

  // HX711 load cell
  scale.begin(PIN_HX711_DT, PIN_HX711_SCK);
  unsigned long hxTimeout = millis() + 3000;
  while (!scale.is_ready() && millis() < hxTimeout) {
    delay(100);
  }

  // I2C bus recovery (clock out any stuck slave)
  pinMode(A5, OUTPUT);
  for (int i = 0; i < 10; i++) {
    digitalWrite(A5, HIGH);
    delayMicroseconds(5);
    digitalWrite(A5, LOW);
    delayMicroseconds(5);
  }
  pinMode(A5, INPUT);

  // ADXL345 accelerometer
  Wire.begin();
  setupADXL345();

  // Calculate initial step intervals
  updateStepInterval();

  Serial.println("EVENT;READY");
}

// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
  // 1. Process serial commands
  processSerial();

  // 2. Safety: endstop check (always active)
  if (digitalRead(PIN_ENDSTOP) == LOW) {
    if (currentState == STATE_RUNNING || currentState == STATE_JOG) {
      currentState = STATE_STOPPED;
      Serial.println("EVENT;ENDSTOP_TRIGGERED");
    }
  }

  // 3. Safety: over-force protection (3 consecutive readings required)
  if (currentState == STATE_RUNNING && abs(lastValidForce) > FORCE_LIMIT_RAW) {
    overLimitCount++;
    if (overLimitCount >= 3) {
      currentState = STATE_STOPPED;
      Serial.print("EVENT;FORCE_LIMIT:");
      Serial.print(lastValidForce * FORCE_CALIBRATION, 1);
      Serial.print("N;LIMIT:");
      Serial.print(FORCE_LIMIT_N);
      Serial.println("N");
    }
  } else {
    overLimitCount = 0;
  }

  // 4. State machine
  switch (currentState) {
    case STATE_IDLE:
      break;
    case STATE_RUNNING:
      doStep();
      break;
    case STATE_JOG:
      doJog();
      break;
    case STATE_STOPPED:
      break;
    case STATE_ERROR:
      break;
  }

  // 5. Data acquisition (only while running)
  if (currentState == STATE_RUNNING) {
    sendDataIfDue();
  }
}

// =============================================================================
// MOTION CONTROL
// =============================================================================

void doStep() {
  unsigned long now_us = micros();

  if (now_us - lastStepTime_us >= stepInterval_us) {
    lastStepTime_us = now_us;

    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_STEP, LOW);

    totalSteps += moveDirection;
  }
}

void doJog() {
  unsigned long now_us = micros();

  if (now_us - lastStepTime_us >= jogInterval_us) {
    lastStepTime_us = now_us;

    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_STEP, HIGH);

    totalSteps += jogDirection;
  }
}

void updateStepInterval() {
  // Test speed
  float stepsPerSecond = STEPS_PER_MM * (targetSpeed_mm_min / 60.0);
  stepInterval_us = (stepsPerSecond > 0) ? (1000000.0 / stepsPerSecond) : 1000000;

  // Jog speed
  float jogStepsPerSecond = STEPS_PER_MM * (jogSpeed_mm_min / 60.0);
  jogInterval_us = (jogStepsPerSecond > 0) ? (1000000.0 / jogStepsPerSecond) : 1000000;
}

// =============================================================================
// SENSORS
// =============================================================================

void setupADXL345() {
  // Enable measurement mode
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(0x2D);  // POWER_CTL register
  Wire.write(0x08);  // Measure bit
  Wire.endTransmission();

  // Set range to ±4g
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(0x31);  // DATA_FORMAT register
  Wire.write(0x01);  // ±4g range
  Wire.endTransmission();

  // Activity detection threshold (8 × 62.5 mg = 500 mg)
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(0x24);  // THRESH_ACT register
  Wire.write(8);
  Wire.endTransmission();

  // Enable activity detection on all axes
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(0x27);  // ACT_INACT_CTL register
  Wire.write(0xF0);  // AC-coupled, X/Y/Z enabled
  Wire.endTransmission();

  // Map activity interrupt to INT1
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(0x2E);  // INT_ENABLE register
  Wire.write(0x10);  // Activity interrupt enabled
  Wire.endTransmission();
}

void readADXL345() {
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(0x32);  // First data register (DATAX0)
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345_ADDR, 6, true);

  ax = Wire.read() | (Wire.read() << 8);
  ay = Wire.read() | (Wire.read() << 8);
  az = Wire.read() | (Wire.read() << 8);
}

// =============================================================================
// DATA ACQUISITION & STEP LOSS DETECTION
// =============================================================================

void sendDataIfDue() {
  unsigned long now = millis();

  if (now - lastDataTime < DATA_INTERVAL_MS) return;
  lastDataTime = now;

  // Read force
  if (scale.is_ready()) {
    lastValidForce = scale.read() - forceOffset;
  }

  // Read accelerometer
  readADXL345();

  // Step loss detection (skip first second to let vibrations settle)
  if (now - runStartTime > 1000) {
    // Read activity interrupt source
    Wire.beginTransmission(ADXL345_ADDR);
    Wire.write(0x30);  // INT_SOURCE register
    Wire.endTransmission(false);
    Wire.requestFrom(ADXL345_ADDR, 1, true);
    byte intSource = Wire.read();

    if (!stepLossCheckReady) {
      // Discard first interrupt reading (may be stale)
      stepLossCheckReady = true;
    } else if ((intSource & 0x10) && !stepLossDetected) {
      // Activity interrupt fired — check if force is still high.
      // If force has dropped, this is specimen fracture, not step loss.
      if (abs(lastValidForce) > FORCE_LIMIT_RAW / 10) {
        stepLossDetected = true;
        Serial.println("EVENT;STEP_LOSS");
      }
    }
  }

  // Read endstop
  int endstopState = digitalRead(PIN_ENDSTOP);

  // Transmit data: DATA;millis;steps;raw_force;ax;ay;az;endstop;step_loss
  Serial.print("DATA;");
  Serial.print(now);
  Serial.print(";");
  Serial.print(totalSteps);
  Serial.print(";");
  Serial.print(lastValidForce);
  Serial.print(";");
  Serial.print(ax);
  Serial.print(";");
  Serial.print(ay);
  Serial.print(";");
  Serial.print(az);
  Serial.print(";");
  Serial.print(endstopState);
  Serial.print(";");
  Serial.println(stepLossDetected ? 1 : 0);
}

// =============================================================================
// SERIAL COMMAND PROCESSING
// =============================================================================

void processSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        executeCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}

void executeCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "START") {
    if (currentState == STATE_IDLE || currentState == STATE_STOPPED) {
      digitalWrite(PIN_DIR, moveDirection > 0 ? HIGH : LOW);
      totalSteps = 0;
      stepLossDetected = false;
      stepLossCheckReady = false;
      overLimitCount = 0;
      runStartTime = millis();
      currentState = STATE_RUNNING;
      Serial.println("EVENT;STARTED");
    }
  }
  else if (cmd == "STOP") {
    currentState = STATE_STOPPED;
    Serial.println("EVENT;STOPPED");
  }
  else if (cmd == "UP") {
    jogDirection = 1;
    digitalWrite(PIN_DIR, LOW);
    currentState = STATE_JOG;
    Serial.println("EVENT;JOG_UP");
  }
  else if (cmd == "DOWN") {
    jogDirection = -1;
    digitalWrite(PIN_DIR, HIGH);
    currentState = STATE_JOG;
    Serial.println("EVENT;JOG_DOWN");
  }
  else if (cmd.startsWith("SET_SPEED:")) {
    float newSpeed = cmd.substring(10).toFloat();
    if (newSpeed > 0 && newSpeed < 600) {
      targetSpeed_mm_min = newSpeed;
      updateStepInterval();
      Serial.print("EVENT;SPEED_SET:");
      Serial.println(targetSpeed_mm_min);
    }
  }
  else if (cmd.startsWith("SET_DIR:")) {
    int newDir = cmd.substring(8).toInt();
    if (newDir == 1 || newDir == -1) {
      moveDirection = newDir;
      Serial.print("EVENT;DIR_SET:");
      Serial.println(moveDirection);
    }
  }
  else if (cmd == "TARE") {
    if (scale.is_ready()) {
      forceOffset = scale.read();
      lastValidForce = 0;
      Serial.println("EVENT;TARED");
    }
  }
  else if (cmd == "STATUS") {
    Serial.print("STATUS;");
    Serial.print(currentState);
    Serial.print(";");
    Serial.print(targetSpeed_mm_min);
    Serial.print(";");
    Serial.println(moveDirection);
  }
  else if (cmd == "RESET") {
    currentState = STATE_IDLE;
    totalSteps = 0;
    lastValidForce = 0;
    Serial.println("EVENT;RESET");
  }
  else if (cmd == "FORCE") {
    if (scale.is_ready()) {
      lastValidForce = scale.read() - forceOffset;
    }
    Serial.print("EVENT;FORCE:");
    Serial.println(lastValidForce * FORCE_CALIBRATION, 3);
  }
  else {
    Serial.print("EVENT;UNKNOWN_CMD:");
    Serial.println(cmd);
  }
}
