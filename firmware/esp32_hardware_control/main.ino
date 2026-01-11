#include <SPI.h>
#include <Wire.h>
#include <mcp2515.h>
#include <Bluepad32.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <FastLED.h>
#include "Adafruit_VL53L0X.h"
#include <Adafruit_PWMServoDriver.h> 

// --- BLUEPAD32 CONFIG ---
const String allowedMAC = "a0:5a:5e:a6:3a:aa"; 
ControllerPtr myController = nullptr;

// --- HARDWARE PIN CONFIG ---
const int CS_Pin = 4;
const int SSR_Pin = 15;
#define LED_PIN 25
#define NUM_LEDS 8
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB

// --- TOF XSHUT PIN ---
#define TOF1_PIN 13

// --- I2C CONFIGURATION ---
// Bus 0 (Wire): BNO055 (0x28), ToF (0x30), Servo Driver (0x40)
// Bus 1 (Wire1): Communication with micro_ros ESP32
#define I2C_SLAVE_ADDR 0x10
#define RX_SDA 32
#define RX_SCL 33

// ================================================================
//   SERVO ARM CONFIGURATION (Integrated)
// ================================================================
#define SERVO_FREQ 50 

int servoPulseMin[4] = {150, 150, 0, 0}; 
int servoPulseMax[4] = {600, 600, 550, 800};
int servoMinLimit[4] = {30,  0,  30,  25}; 
int servoMaxLimit[4] = {135, 115, 150, 90};

// --- AUTO SEQUENCE STATE MACHINE ---
#define SEQ_NONE 0
#define SEQ_PICK 1
#define SEQ_PLACE 2

int currentSeqMode = SEQ_NONE;
int seqStep = 0;           
unsigned long seqTimer = 0;

// --- TOGGLE STATES ---
bool isGripperClosed = false; 
bool isArmDown = false;      

// --- DATA STRUCTURES ---
struct Packet {
  float vx;
  float vy;
  float wz;
  float trigger_seq;
};

struct PID_Config { float Kp, Ki, Kd, integral, prevError; };

struct FeedbackPacket {
  float heading;
  float tof_distance;
  float rpm[4];
};
FeedbackPacket sensorData;

// --- OBJECTS ---
CRGB leds[NUM_LEDS];
MCP2515 mcp2515(CS_Pin);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
struct can_frame canRead;
struct can_frame canWrite;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- GLOBAL VARIABLES ---
volatile Packet rxData;
volatile bool newData = false;
unsigned long last_i2c_time = 0;
bool ros_active = false;
bool has_contact = false;
int servoAngles[4] = {135, 5, 150, 30};

// ToF Readings
uint16_t dist1 = 0;

// Motor State
float currentRPM_Motor1 = 0, currentRPM_Motor2 = 0, currentRPM_Motor3 = 0, currentRPM_Motor4 = 0;
float input_Vx = 0.0, input_Vy = 0.0, input_Wz = 0.0; 

// Heading State
float robotHeading = 0.0;    // Actual heading (corrected)
float targetHeading = 0.0;   // Desired heading
float headingOffset = 0.0;   // Offset for "Reset Heading"
float headingKp = 5;  
float headingKi = 0.0;  
float headingKd = 0.5;  
float headingIntegral = 0.0;
float headingPrevError = 0.0;

// LED State
int currentLEDMode = 0;
uint8_t gHue = 0;

// Button One-Shot Logic
bool btnOptionsWasPressed = false;
bool btnShareWasPressed = false;
unsigned long lastServoInputTime = 0; // Debounce for arm

// Motor PID Configuration
PID_Config pidMotor1 = {25.0, 1.5, 1.00, 0.0, 0.0}; 
PID_Config pidMotor2 = {17.5, 1.0, 1.00, 0.0, 0.0};
PID_Config pidMotor3 = {25.0, 1.5, 1.00, 0.0, 0.0};
PID_Config pidMotor4 = {17.5, 1.0, 1.00, 0.0, 0.0};

// Motor Constants
const uint32_t RM_CMD_ID_1_4 = 0x200;
const float MAX_RPM_Target = 300.0f;

// --- INTERRUPT SERVICE ROUTINE ---
void receiveEvent(int howMany) {
  if (howMany == sizeof(Packet)) {
    Wire1.readBytes((uint8_t *)&rxData, sizeof(Packet));
    newData = true;
  } else {
    while(Wire1.available()) Wire1.read(); // Flush garbage
  }
}

void requestEvent() {
  Wire1.write((uint8_t *)&sensorData, sizeof(FeedbackPacket));
}

// ================================================================
//    HELPER FUNCTIONS
// ================================================================

String getMacAddress(ControllerPtr ctl) {
    ControllerProperties properties = ctl->getProperties();
    const uint8_t* addr = properties.btaddr;
    char str[18];
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x", 
            addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    return String(str);
}

void onConnectedController(ControllerPtr ctl) {
    String mac = getMacAddress(ctl);
    mac.toLowerCase(); 
    if (mac == allowedMAC) {
        Serial.printf("SAFE: Controller %s connected.\n", mac.c_str());
        myController = ctl;
        myController->setColorLED(125, 0, 125); 
    } else {
        Serial.printf("WARNING: Blocked unauthorized controller: %s\n", mac.c_str());
        ctl->setColorLED(255, 0, 0);
        ctl->disconnect();
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    if (myController == ctl) {
        Serial.println("Controller Disconnected.");
        myController = nullptr;
    }
}

void FlashLED(CRGB color, int wait = 200) {
  fill_solid(leds, NUM_LEDS, color);
  FastLED.show();
  delay(wait);
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
}

// --- SERVO FUNCTIONS ---
void setServoAngle(uint8_t servoNum, int angle) {
  if (servoNum > 3) return; 
  angle = constrain(angle, servoMinLimit[servoNum], servoMaxLimit[servoNum]);
  int pulse = map(angle, 0, 180, servoPulseMin[servoNum], servoPulseMax[servoNum]);
  pwm.setPWM(servoNum, 0, pulse);
}

void UpdateAllServos() {
    for (int i = 0; i < 4; i++) {
        setServoAngle(i, servoAngles[i]);
    }
}

void ProcessMotorFeedback(bool IsM3508, uint32_t CAN_ID, uint8_t RPM_H, uint8_t RPM_L) { 
  int16_t RPM_Raw = (RPM_H << 8) | RPM_L; 
  float gearReduction = (IsM3508) ? 19.0f : 36.0f; // Simplified logic
  float RPM_True = (float)RPM_Raw / gearReduction;
  
  if (CAN_ID == 0x201) currentRPM_Motor1 = RPM_True;
  else if (CAN_ID == 0x202) currentRPM_Motor2 = RPM_True;
  else if (CAN_ID == 0x203) currentRPM_Motor3 = RPM_True;
  else if (CAN_ID == 0x204) currentRPM_Motor4 = RPM_True;
}

void UpdateFromComms() {
  if (newData) {
    input_Vx = -rxData.vx;
    input_Vy = rxData.vy;
    targetHeading -= rxData.wz; 

    if (currentSeqMode == SEQ_NONE) {
        // 1.0 = PICK
        if (rxData.trigger_seq > 0.9 && rxData.trigger_seq < 1.1) {
            StartPick();
        }
        // 2.0 = PLACE
        else if (rxData.trigger_seq > 1.9 && rxData.trigger_seq < 2.1) {
            StartPlace();
        }
        // 3.0 = RESET
        else if (rxData.trigger_seq > 2.9 && rxData.trigger_seq < 3.1) {
            StartReset();
        }
    }

    if (!has_contact) {
      Serial.println("SUCCESS: Micro-ROS ESP32 Detected!");
      has_contact = true;
    }
    last_i2c_time = millis();
    ros_active = true;
    newData = false;
  }
}

void HandleIncomingCAN() {
  while (mcp2515.readMessage(&canRead) == MCP2515::ERROR_OK) {
    if (canRead.can_id >= 0x201 && canRead.can_id <= 0x208) { 
      ProcessMotorFeedback(false, canRead.can_id, canRead.data[2], canRead.data[3]);
    }
  }
}

void SetupToF() {
  pinMode(TOF1_PIN, OUTPUT);
  digitalWrite(TOF1_PIN, LOW); delay(10);
  digitalWrite(TOF1_PIN, HIGH); delay(10);
  if (lox.begin()) {
    Serial.println("ToF 1 Ready");
    FlashLED(CRGB::Green);
  } else {
    Serial.println("ToF 1 Failed to Boot!");
    FlashLED(CRGB::Orange, 500);
  }
}

void ReadToF() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) { 
    dist1 = measure.RangeMilliMeter;
  } else {
    dist1 = 8190; 
  }
}

void ReadIMU() {
  sensors_event_t event;
  bno.getEvent(&event);
  float correctedHeading = event.orientation.x - headingOffset;
  if (correctedHeading < 0) correctedHeading += 360;
  if (correctedHeading > 360) correctedHeading -= 360;
  robotHeading = correctedHeading;
  if (robotHeading > 180) robotHeading -= 360;
}

float GetRotationCorrection() {
  float error = targetHeading - robotHeading;
  if (error > 180) error -= 360;
  if (error < -180) error += 360;

  float P = error * headingKp;
  headingIntegral += error;
  if (headingIntegral > 500) headingIntegral = 500;
  if (headingIntegral < -500) headingIntegral = -500;
  float I = headingIntegral * headingKi;
  float D = headingKd * (error - headingPrevError);
  headingPrevError = error;

  float rotationSpeed = P + I + D;
  if (abs(error) < 1.0) {
    rotationSpeed = 0;
    headingIntegral = 0; 
  }
  return rotationSpeed;
}

void StartReset() {
    Serial.println("COMMAND: RESET SERVOS");
    currentSeqMode = SEQ_NONE;
    seqStep = 0;
    
    servoAngles[0] = 135; 
    servoAngles[1] = 5; 
    servoAngles[2] = 150; 
    servoAngles[3] = 35;
    
    isGripperClosed = false;
    isArmDown = false;
    
    UpdateAllServos();
}

void StartPick() {
    if (currentSeqMode == SEQ_NONE) {
        Serial.println("STARTING PICK SEQUENCE...");
        currentSeqMode = SEQ_PICK;
        seqStep = 1;
        seqTimer = millis() - 1000; // Trigger Step 1 immediately
    }
}

void StartPlace() {
    if (currentSeqMode == SEQ_NONE) {
        Serial.println("STARTING PLACE SEQUENCE...");
        currentSeqMode = SEQ_PLACE;
        seqStep = 4; // Start at Step 4
        seqTimer = millis() - 1000;
    }
}

void RunPickSequence() {
    if (currentSeqMode != SEQ_PICK) return;
    
    int stepDelay = 600; 
    unsigned long now = millis();
    if (now - seqTimer < stepDelay) return;

    switch (seqStep) {
        case 1: // Arm Down
            servoAngles[0] = 52; servoAngles[1] = 30; servoAngles[2] = 141;
            UpdateAllServos();
            seqTimer = now;
            seqStep++; 
            break;
        case 2: // Gripper Close
            servoAngles[3] = 90;
            UpdateAllServos();
            seqTimer = now;
            seqStep++;
            break;
        case 3: // Arm Up (With Object)
            servoAngles[0] = 135; servoAngles[1] = 5; servoAngles[2] = 150;
            UpdateAllServos();
            
            // End of Pick
            currentSeqMode = SEQ_NONE;
            isGripperClosed = true; // State: Holding object
            Serial.println("PICK: Complete");
            break;
    }
}

void RunPlaceSequence() {
    if (currentSeqMode != SEQ_PLACE) return;
    
    int stepDelay = 600; 
    unsigned long now = millis();
    if (now - seqTimer < stepDelay) return;

    switch (seqStep) {
        case 4: // Arm Down (With Object)
            servoAngles[0] = 77; servoAngles[1] = 34; servoAngles[2] = 117;
            UpdateAllServos();
            seqTimer = now;
            seqStep++;
            break;
        case 5: // Gripper Open (Drop)
            servoAngles[3] = 35;
            UpdateAllServos();
            seqTimer = now;
            seqStep++;
            break;
        case 6: // Arm Up (Empty)
            servoAngles[0] = 135; servoAngles[1] = 5; servoAngles[2] = 150;
            servoAngles[3] = 35; // Reset Gripper slightly
            UpdateAllServos();
            
            // End of Place
            currentSeqMode = SEQ_NONE;
            isGripperClosed = false; // State: Empty
            Serial.println("PLACE: Complete");
            break;
    }
}

// --- CONTROLLER INPUT HANDLING ---
void HandleArmInput() {
    if (millis() - lastServoInputTime < 250) return;
    bool inputDetected = false;

    // --- BUTTON CROSS (X): SMART TOGGLE (Pick OR Place) ---
    if (myController->a()) { 
        if (currentSeqMode == SEQ_NONE) {
            if (isGripperClosed) {
                StartPlace();
            } else {
                StartPick();
            }
            inputDetected = true;
        }
    }

    // --- BUTTON SQUARE: EMERGENCY RESET ---
    else if (myController->x()) {
        StartReset(); // [UPDATED] Uses the unified reset function
        inputDetected = true;
    }

    // --- BUTTON TRIANGLE: TOGGLE ARM UP/DOWN ---
    else if (myController->y() && currentSeqMode == SEQ_NONE) {
        if (isArmDown) {
            servoAngles[0] = 135; servoAngles[1] = 5; servoAngles[2] = 150;
            isArmDown = false;
        } else {
            servoAngles[0] = 49; servoAngles[1] = 26; servoAngles[2] = 145;
            isArmDown = true;
        }
        UpdateAllServos();
        inputDetected = true;
    }

    // --- BUTTON CIRCLE: TOGGLE GRIPPER ---
    else if (myController->b() && currentSeqMode == SEQ_NONE) {
        if (isGripperClosed) {
            servoAngles[3] = 35;
            isGripperClosed = false;
            Serial.println("Manual: Gripper OPEN");
        } else {
            servoAngles[3] = 90;
            isGripperClosed = true;
            Serial.println("Manual: Gripper CLOSE");
        }
        UpdateAllServos();
        inputDetected = true;
    }

    if (inputDetected) {
        lastServoInputTime = millis();
    }
}

void HandleNonDriveFeatures() {
  if (myController && myController->isConnected()) {
      
      HandleArmInput();

      // OPTIONS: Reset Gyro
      if ((myController->miscButtons() & 0x04) != 0) {
        if (!btnOptionsWasPressed) {
          sensors_event_t event;
          bno.getEvent(&event);
          headingOffset = event.orientation.x; 
          targetHeading = 0; 
          Serial.println("Heading Reset!");
          FlashLED(CRGB::White, 100);
          btnOptionsWasPressed = true;
        }
      } else {
        btnOptionsWasPressed = false;
      }
      
      // SHARE: Change LEDs
      if ((myController->miscButtons() & 0x02) != 0) {
        if(!btnShareWasPressed) { 
          currentLEDMode++;  
          if (currentLEDMode > 3) currentLEDMode = 0; 
          btnShareWasPressed = true; 
        }
      } else { 
        btnShareWasPressed = false; 
      }
  }
}

void UpdateLEDs() {
  if (!myController || !myController->isConnected()) {
    uint8_t val = beatsin8(30, 50, 255); 
    fill_solid(leds, NUM_LEDS, CHSV(192, 255, val));
    FastLED.show();
    return;
  }
  EVERY_N_MILLISECONDS(20) { gHue++; } 
  switch (currentLEDMode) {
    case 0: {
        fadeToBlackBy(leds, NUM_LEDS, 10); 
        int pos = (millis() / 100) % NUM_LEDS; 
        leds[pos] = CRGB::Purple;
        break;
      }
    case 1: fill_rainbow(leds, NUM_LEDS, gHue, 7); break;
    case 2: {
        int validDist = constrain(dist1, 0, 1000);
        uint8_t hue = map(validDist, 50, 500, 0, 96);
        fill_solid(leds, NUM_LEDS, CHSV(hue, 255, 255));
        break;
      }
    case 3: fill_solid(leds, NUM_LEDS, CRGB::Black); break;
  }
  FastLED.show();
}

bool HandlePS4Input() {
  if (!(myController && myController->isConnected())) return false;

  HandleNonDriveFeatures();

  bool stickMoved = (abs(myController->axisY()) > 15 || abs(myController->axisX()) > 15);
  bool rotationPressed = (myController->l1() || myController->r1());

  if (stickMoved || rotationPressed) {
      ros_active = false; // Override ROS
      
      input_Vx = (float)myController->axisY();
      input_Vy = (float)myController->axisX();
      
      if (myController->l1()) targetHeading -= 1.0; 
      if (myController->r1()) targetHeading += 1.0;
      
      if (targetHeading > 180)  targetHeading -= 360;
      if (targetHeading < -180) targetHeading += 360;
      
      return true; // Manual Mode Active
  }
  return false; 
}

void HandleSafetyTimeout() {
  if (ros_active && (millis() - last_i2c_time > 500)) {
      ros_active = false;
      input_Vx = 0; 
      input_Vy = 0;
  }
}

int16_t CalculatePID(float targetRPM, float actualRPM, PID_Config &pid) {
  float error = targetRPM - actualRPM;
  float P = pid.Kp * error;
  pid.integral += error;
  if (pid.integral > 500) pid.integral = 500;
  if (pid.integral < -500) pid.integral = -500;
  float I = pid.Ki * pid.integral;
  float D = pid.Kd * (error - pid.prevError);
  pid.prevError = error;
  float output = P + I + D;
  if (output > 16384.0f) output = 16384.0f;
  if (output < -16384.0f) output = -16384.0f;
  return (int16_t)output;
}

void ComputeMotorOutputs(float in_vx, float in_vy, int16_t &m1, int16_t &m2, int16_t &m3, int16_t &m4) {
    float Wz = GetRotationCorrection();

    float drive_Forward = in_vx; 
    float drive_Strafe = in_vy;

    float target_RR = drive_Strafe + drive_Forward + Wz; 
    float target_FR = -drive_Strafe + drive_Forward + Wz; 
    float target_FL = -drive_Strafe - drive_Forward + Wz; 
    float target_RL = drive_Strafe - drive_Forward + Wz;

    float max_val = max(abs(target_FL), max(abs(target_FR), max(abs(target_RL), abs(target_RR))));
    if (max_val > 512.0f) {
      float scale = 512.0f / max_val;
      target_RR *= scale; target_FR *= scale; target_FL *= scale; target_RL *= scale;
    }

    float scaleFactor = MAX_RPM_Target / 512.0f; 
    m1 = CalculatePID(target_RR * scaleFactor, currentRPM_Motor1, pidMotor1);
    m2 = CalculatePID(target_FR * scaleFactor, currentRPM_Motor2, pidMotor2);
    m3 = CalculatePID(target_FL * scaleFactor, currentRPM_Motor3, pidMotor3);
    m4 = CalculatePID(target_RL * scaleFactor, currentRPM_Motor4, pidMotor4);
}

void SendCANCommand1(int16_t m1, int16_t m2, int16_t m3, int16_t m4) {
  canWrite.can_id = RM_CMD_ID_1_4; 
  canWrite.can_dlc = 8;            
  int16_t allMotors[4] = {m1, m2, m3, m4};
  for (int i = 0; i < 4; i++) {
    canWrite.data[i * 2]     = (allMotors[i] >> 8) & 0xFF; 
    canWrite.data[i * 2 + 1] = allMotors[i] & 0xFF;        
  }
  mcp2515.sendMessage(&canWrite);
}

void PrintDebug() {
  static unsigned long last_debug_time = 0;
  if (millis() - last_debug_time < 200) return;
  last_debug_time = millis();

  Serial.print("[ROS] ");
  Serial.print("Active:"); Serial.print(ros_active);
  Serial.print(" [Arm] ");
  Serial.print(servoAngles[0]); Serial.print("/");
  Serial.print(servoAngles[1]); Serial.print("/");
  Serial.print(servoAngles[2]); Serial.print("/");
  Serial.println(servoAngles[3]);
}

// ================================================================
//    MAIN SETUP & LOOP
// ================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Integrated Controller Start");

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(125);
  fill_solid(leds, NUM_LEDS, CRGB::Blue);
  FastLED.show();
  delay(500);

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys(); 
  BP32.enableVirtualDevice(false);

  pinMode(SSR_Pin, OUTPUT);
  digitalWrite(SSR_Pin, HIGH);

  Wire.begin();
  SetupToF();
  
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  
  for (int i=0; i<4; i++) {
      setServoAngle(i, servoAngles[i]);
      delay(20); 
  }
  FlashLED(CRGB::Green);

  Wire.setClock(400000); // Speed up I2C

  if (!bno.begin()) {
    Serial.println("No BNO!");
    fill_solid(leds, NUM_LEDS, CRGB::Red);
    FastLED.show();
    while(1); 
  }
  Serial.println("BNO Ready");
  FlashLED(CRGB::Green);
  delay(100);
  bno.setExtCrystalUse(true);

  Wire1.begin(I2C_SLAVE_ADDR, RX_SDA, RX_SCL, 100000); 
  Wire1.onReceive(receiveEvent);
  Wire1.onRequest(requestEvent);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ); 
  mcp2515.setNormalMode();

  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();

  canWrite.can_id = RM_CMD_ID_1_4; 
  canWrite.can_dlc = 8;            
  for(int i = 0; i < 8; i++) canWrite.data[i] = 0x00;
}

void loop() {
  sensorData.heading = robotHeading;
  sensorData.tof_distance = (float)dist1;
  sensorData.rpm[0] = currentRPM_Motor1;
  sensorData.rpm[1] = currentRPM_Motor2;
  sensorData.rpm[2] = currentRPM_Motor3;
  sensorData.rpm[3] = currentRPM_Motor4;

  UpdateFromComms();
  BP32.update();
  HandleIncomingCAN();
  ReadIMU();
  RunPickSequence();
  RunPlaceSequence();

  static unsigned long last_tof_time = 0;
  if (millis() - last_tof_time > 50) {
    ReadToF();
    last_tof_time = millis();
  }

  bool isManual = HandlePS4Input();

  if (!isManual) {
    HandleSafetyTimeout(); 
    if (!ros_active) {
      input_Vx = 0.0;
      input_Vy = 0.0;
    }
  }

  PrintDebug();
  UpdateLEDs();
  
  int16_t m1 = 0, m2 = 0, m3 = 0, m4 = 0;
  ComputeMotorOutputs(input_Vx, input_Vy, m1, m2, m3, m4);
  SendCANCommand1(m1, m2, m3, m4);
}
