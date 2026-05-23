#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <math.h>

// ====== FRAM Setup ======
#define FRAM_SIZE 32768
#define FRAM_I2C_ADDR 0x50

struct FlightLogEntry {
  unsigned long timestamp;
  float tempC;
  float pitch;
  float roll;
  uint8_t servo1_pos;
  uint8_t servo2_pos;
  float altitude;
};

unsigned int framAddr = 0;

unsigned long logStartTime = 0;
const unsigned long LOG_DURATION_MS = 60000; // 1 minute
const unsigned int RECORD_SIZE = sizeof(FlightLogEntry);

// FRAM write helper
void framWriteData(unsigned int addr, byte* data, int len) {
  Wire.beginTransmission(FRAM_I2C_ADDR);
  Wire.write((addr >> 8) & 0xFF);
  Wire.write(addr & 0xFF);
  for (int i = 0; i < len; i++) {
    Wire.write(data[i]);
  }
  Wire.endTransmission();
}

// FRAM read helper
void framReadData(unsigned int addr, byte* buffer, int len) {
  Wire.beginTransmission(FRAM_I2C_ADDR);
  Wire.write((addr >> 8) & 0xFF);
  Wire.write(addr & 0xFF);
  Wire.endTransmission();

  Wire.requestFrom(FRAM_I2C_ADDR, len);
  for (int i = 0; i < len && Wire.available(); i++) {
    buffer[i] = Wire.read();
  }
}

// Wipe FRAM function
void wipeFRAM() {
  const int chunkSize = 16;
  byte zeros[chunkSize] = {0};
  for (unsigned int addr = 0; addr < FRAM_SIZE; addr += chunkSize) {
    framWriteData(addr, zeros, chunkSize);
  }
  Serial.println("FRAM wiped");
}

// ====== BME280 Setup ======
#define SEALEVELPRESSURE_HPA (1013.25)
#define METERS_TO_FEET 3.28084
Adafruit_BME280 bme;
float baselineAltitude = 0.0;

// ====== TVC Servos ======
Servo servo1; // pitch servo on pin 10
Servo servo2; // roll servo on pin 11

// Servo center and limits (adjust to your servos' calibration)
const int servo1_mid = 112, servo1_left = 180, servo1_right = 44;  // pitch servo limits
const int servo2_mid = 45,  servo2_left = 0,   servo2_right = 90;   // roll servo limits

bool invertServo1 = true;  // invert pitch servo output if needed
bool invertServo2 = false; // invert roll servo output if needed

// ====== PID Constants ======
float Kp = 1.5;
float Ki = 0.1;
float Kd = 0.05;

float pitchErrorSum = 0, rollErrorSum = 0;
float lastPitchError = 0, lastRollError = 0;
float deadband = 2.0;

// ====== MPU6050 Vars ======
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
unsigned long lastTime;
float kalmanPitch = 0.0, pitchBias = 0.0, P_pitch[2][2] = {{0,0},{0,0}};
float kalmanRoll  = 0.0, rollBias  = 0.0, P_roll[2][2]  = {{0,0},{0,0}};

float servo1_pos_smooth = servo1_mid;
float servo2_pos_smooth = servo2_mid;
const float smoothingFactor = 0.1;

// ====== Flight State ======
enum FlightStage {
  STAGE_PREFLIGHT,
  STAGE_COUNTDOWN,
  STAGE_LAUNCH,
  STAGE_TVC_ACTIVE,
  STAGE_BURNOUT,
  STAGE_APOGEE,
  STAGE_DESCENT,
  STAGE_PARACHUTE,
  STAGE_RECOVERY
};
FlightStage stage = STAGE_PREFLIGHT;

// ====== Flight Vars ======
float currentAltitude = 0.0;
float lastAltitudeForApogee = 0.0;
float peakAltitude = 0.0;
unsigned long stageStartTime = 0;
unsigned long lastLogTime = 0;
unsigned long lastSerialPrintTime = 0;
unsigned long burnoutTime = 0;
bool tvcEnabled = false;
bool loggingEnabled = false;
unsigned long tvcStartTime = 0;

// ====== Burnout detection vars ======
float lastAltitudeForBurnout = 0.0;
unsigned long lastAltitudeCheckTime = 0;
const unsigned long burnoutDetectionInterval = 1000;  // check every 1 second
const float burnoutAltitudeThreshold = 1.0; // feet, minimal increase to consider motor still burning

// ====== Functions ======
void framWriteString(const String &data) {
  for (unsigned int i = 0; i < data.length(); i++) {
    Wire.beginTransmission(FRAM_I2C_ADDR);
    Wire.write((framAddr >> 8) & 0xFF);
    Wire.write(framAddr & 0xFF);
    Wire.write((byte)data[i]);
    Wire.endTransmission();
    framAddr++;
  }
}

void framWriteLine(const String &line) {
  framWriteString(line + "\n");
}

void readMPU6050() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);

  accX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  accY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  accZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
  Wire.read(); Wire.read(); // skip temperature bytes
  gyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  gyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  gyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
}

float kalmanUpdate(float newAngle, float newRate, float dt,
                   float &kalmanAngle, float &bias, float P[2][2]) {
  float rate = newRate - bias;
  kalmanAngle += dt * rate;

  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + 0.001);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += 0.003 * dt;

  float S = P[0][0] + 0.03;
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  float y = newAngle - kalmanAngle;

  kalmanAngle += K[0] * y;
  bias       += K[1] * y;

  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return kalmanAngle;
}

void logFlightData(const String &event = "") {
  unsigned long t = millis();
  float tempC = bme.readTemperature();
  String line = String(t) + "," +
                String(tempC, 2) + "," +
                String(kalmanPitch, 2) + "," +
                String(kalmanRoll, 2) + "," +
                String((int)servo1_pos_smooth) + "," +
                String((int)servo2_pos_smooth) + "," +
                String(currentAltitude, 2);
  if (event.length() > 0) line += "," + event;
  if (loggingEnabled) framWriteLine(line);
}

String serialInput = "";

void checkSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {  // end of command
      serialInput.trim();           // remove whitespace
      if (serialInput.length() > 0) {
        if (serialInput.equalsIgnoreCase("READLOG")) {
          printLogs();  // your existing log print function
        } 
        else if (serialInput.equalsIgnoreCase("WIPE")) {
          wipeFRAM();
          framAddr = 0;  // reset write address to start
          Serial.println("FRAM wiped and write pointer reset.");
        }
        else {
          Serial.println("Unknown command: " + serialInput);
        }
        serialInput = "";  // clear after processing
      }
    } else {
      serialInput += c;  // accumulate chars
    }
  }
}


void printLogs() {
  const unsigned int maxRead = framAddr;  // only read up to last written address
  const int chunkSize = 32;
  byte buffer[chunkSize + 1];

  Serial.println("Reading logs from FRAM:");

  unsigned int addr = 0;
  while (addr < maxRead) {
    int bytesToRead = chunkSize;
    if (addr + bytesToRead > maxRead) {
      bytesToRead = maxRead - addr;
    }

    framReadData(addr, buffer, bytesToRead);
    buffer[bytesToRead] = 0;

    Serial.print((char*)buffer);

    addr += bytesToRead;
  }
  Serial.println("\n--- End of logs ---");
}


void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  framAddr = 0;

  servo1.attach(10); // pitch servo on pin 10
  servo2.attach(11); // roll servo on pin 11

  // Center servos on boot
  servo1.write(servo1_mid);
  servo2.write(servo2_mid);

  logStartTime = millis(); 


  // MPU6050 init
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // BME280 init
  if (!bme.begin(0x76)) {
    Serial.println("BME280 missing!");
    while (1);
  }

  // Baseline altitude averaging
  float sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += bme.readAltitude(SEALEVELPRESSURE_HPA);
    delay(5);
  }
  baselineAltitude = sum / 100.0;

  Serial.println("Baseline altitude set");

  stage = STAGE_COUNTDOWN;
  stageStartTime = millis();
  lastTime = millis();
  lastAltitudeCheckTime = millis();
  lastAltitudeForBurnout = 0;

  Serial.println("Preflight complete");
  Serial.println("Starting countdown...");

  delay(3000);
  loggingEnabled = true;
  stage = STAGE_COUNTDOWN;
  stageStartTime = millis();

  lastLogTime = 0;
  lastSerialPrintTime = 0;

  Serial.println("Starting countdown...");
}

bool printingEnabled = true;  // global or static variable

void loop() {
  readMPU6050();
  checkSerialCommands();
  unsigned long now = millis();
  if (loggingEnabled && (now - logStartTime > LOG_DURATION_MS)) {
    loggingEnabled = false;
    printingEnabled = false; 
    Serial.println("Logging stopped after 1 minute");
    Serial.println("Printing stopped");
  }
  


  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // Altitude reading smoothing
  float altSum = 0;
  for (int i = 0; i < 5; i++) {
    altSum += bme.readAltitude(SEALEVELPRESSURE_HPA);
    delay(2);
  }
  float burstAvg = altSum / 5.0;
  currentAltitude = (burstAvg - baselineAltitude) * METERS_TO_FEET;

  if (currentAltitude > peakAltitude) peakAltitude = currentAltitude;

  // Orientation calculation (pitch, roll)
  float pitchAcc = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180 / PI;
  float rollAcc  = atan2(accY, accZ) * 180 / PI;
  kalmanPitch = kalmanUpdate(pitchAcc, gyroY, dt, kalmanPitch, pitchBias, P_pitch);
  kalmanRoll  = kalmanUpdate(rollAcc,  gyroX, dt, kalmanRoll,  rollBias,  P_roll);

  // Log every 0.1s to FRAM (only if logging enabled)
static unsigned long lastAltitudeLogTime = 0;
static unsigned long lastFullLogTime = 0;

if (loggingEnabled) {
  if (stage == STAGE_COUNTDOWN || stage == STAGE_PREFLIGHT) {
    // Before launch: altitude every 0.1s, full data every 0.5s

    if (now - lastAltitudeLogTime >= 100) {
      lastAltitudeLogTime = now;
      // Log altitude only line:
      unsigned long t = millis();
      String line = String(t) + "," + String(currentAltitude, 2);
      framWriteLine(line);
    }

    if (now - lastFullLogTime >= 500) {
      lastFullLogTime = now;
      // Log full data
      logFlightData();
    }
  } else {
    // After launch: log full data every 0.1s
    if (now - lastLogTime >= 100) {
      lastLogTime = now;
      logFlightData();
    }
  }
}


  // Print all data every 0.5s including current stage
  if (printingEnabled && (now - lastSerialPrintTime >= 500)) {
    lastSerialPrintTime = now;

    Serial.print("Stage: ");
    switch(stage) {
      case STAGE_PREFLIGHT: Serial.print("Preflight"); break;
      case STAGE_COUNTDOWN: Serial.print("Countdown"); break;
      case STAGE_LAUNCH: Serial.print("Launch"); break;
      case STAGE_TVC_ACTIVE: Serial.print("TVC Active"); break;
      //case STAGE_BURNOUT: Serial.print("Burnout"); break;
      case STAGE_APOGEE: Serial.print("Apogee"); break;
      case STAGE_DESCENT: Serial.print("Descent"); break;
      case STAGE_PARACHUTE: Serial.print("Parachute"); break;
      case STAGE_RECOVERY: Serial.print("Recovery"); break;
    }
    Serial.print(" | Altitude: "); Serial.print(currentAltitude, 2);
    Serial.print(" ft | Pitch: "); Serial.print(kalmanPitch, 2);
    Serial.print(" | Roll: "); Serial.print(kalmanRoll, 2);
    Serial.print(" | Peak Alt: "); Serial.print(peakAltitude, 2);
    Serial.print(" | Servo1: "); Serial.print((int)servo1_pos_smooth);
    Serial.print(" | Servo2: "); Serial.println((int)servo2_pos_smooth);
  }

  // Flight stage logic
  switch(stage) {
    case STAGE_COUNTDOWN: {
      
      
      loggingEnabled = true;
      tvcEnabled = false;
    
      unsigned long elapsed = millis() - stageStartTime;
      int secondsLeft = 10 - (elapsed / 1000);
    
      static int lastPrintedSecond = 11; // outside of function would be better, but static here works
    
      if (secondsLeft >= 0 && secondsLeft != lastPrintedSecond) {
        Serial.print("Countdown: ");
        Serial.println(secondsLeft);
        lastPrintedSecond = secondsLeft;
      }
    
      if (elapsed >= 10000) {  // 10 seconds elapsed
        stage = STAGE_LAUNCH;   // move to launch stage
        stageStartTime = millis();
        Serial.println("Countdown complete, launching...");
        lastPrintedSecond = 11; // reset for next countdown if needed
      }
      break;
    }

    case STAGE_LAUNCH:     
      loggingEnabled = true;
      servo1_pos_smooth = servo1_mid;
      servo2_pos_smooth = servo2_mid;
      servo1.write(servo1_mid);
      servo2.write(servo2_mid);
      if (currentAltitude > 6.0) { // 6 feet threshold
        logStartTime = now;
        stage = STAGE_TVC_ACTIVE;
        stageStartTime = now;
        tvcEnabled = true;
        tvcStartTime = now;
        Serial.println("Launch detected");
        Serial.println("TVC activated");
    }
    break;
    

    case STAGE_TVC_ACTIVE:
      // TVC runs here
      if (currentAltitude < peakAltitude - 5) { // altitude drop detected
        stage = STAGE_APOGEE;
        //tvcEnabled = false; // disable TVC at apogee
        Serial.println("Apogee detected - switching to recovery");
        Serial.println(lastAltitudeForApogee);
      }
      lastAltitudeForApogee = currentAltitude;
      break;


    //case STAGE_BURNOUT:
      // Wait for apogee: vertical velocity near zero or negative (altitude stops rising)
      //if (currentAltitude < peakAltitude - 5) {
        //stage = STAGE_APOGEE;
        //stageStartTime = now;
        //Serial.println("Apogee reached");
      //}
      //break;

    case STAGE_APOGEE:
      // Start descent tracking, activate parachute etc.
      if (currentAltitude < 100) {
        stage = STAGE_DESCENT;
        stageStartTime = now;
        Serial.println("Descent started");
      }
      break;

    case STAGE_DESCENT:
      // Detect parachute deployment by deceleration or altitude change pattern
      if (currentAltitude < 50) {
        stage = STAGE_PARACHUTE;
        stageStartTime = now;
        Serial.println("Parachute deployed");
      }
      break;

    case STAGE_PARACHUTE:
      // Wait until near ground
      if (currentAltitude < 10) {
        stage = STAGE_RECOVERY;
        stageStartTime = now;
        Serial.println("Recovery stage");
        // On recovery, center servos and disable TVC control
        //tvcEnabled = false;
      }
      break;

    case STAGE_RECOVERY:
      // Do nothing or reset system here if needed
      break;
}

  // PID & TVC servo control (servo output never disables except at recovery)
  if (tvcEnabled) {
    // Calculate errors (desired is 0 degrees pitch and roll)
    float pitchError = 0 - kalmanPitch;
    float rollError  = 0 - kalmanRoll;

    // Integrate errors with deadband
    if (abs(pitchError) > deadband) pitchErrorSum += pitchError * dt; else pitchErrorSum = 0;
    if (abs(rollError)  > deadband) rollErrorSum  += rollError  * dt; else rollErrorSum  = 0;

    // Calculate derivative errors
    float pitchDerivative = (pitchError - lastPitchError) / dt;
    float rollDerivative  = (rollError  - lastRollError)  / dt;

    // PID output
    float pitchOutput = Kp * pitchError + Ki * pitchErrorSum + Kd * pitchDerivative;
    float rollOutput  = Kp * rollError  + Ki * rollErrorSum  + Kd * rollDerivative;

    lastPitchError = pitchError;
    lastRollError = rollError;

    // Calculate servo positions with inversion and limits
    int servo1Target = servo1_mid + (invertServo1 ? -pitchOutput : pitchOutput);
    int servo2Target = servo2_mid + (invertServo2 ? -rollOutput : rollOutput);

    servo1Target = constrain(servo1Target, servo1_right, servo1_left);
    servo2Target = constrain(servo2Target, servo2_left, servo2_right);

    // Smooth servo movements
    servo1_pos_smooth = servo1_pos_smooth + smoothingFactor * (servo1Target - servo1_pos_smooth);
    servo2_pos_smooth = servo2_pos_smooth + smoothingFactor * (servo2Target - servo2_pos_smooth);

    servo1.write((int)servo1_pos_smooth);
    servo2.write((int)servo2_pos_smooth);

  
    // If not recovery, do not disable or center servos; keep last positions
  }
}
