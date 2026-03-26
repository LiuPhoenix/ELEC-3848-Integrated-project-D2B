/*****************************************************
Light Tracking Robot with Color Detection + PID Wall Align
Version: 2.21 (Phase 1: Approach 31cm before color detection)
Date: 2026-03-27

Flow:
1. Calibrate: Light ambient -> Light dark -> RED (button) -> GREEN (button)
2. Phase 1 Move Forward:
   - Step 1: Forward until avg distance <= 31cm (STRAIGHT, no parallel adjust)
   - Step 2: Then start color detection loop
   - If color detected -> exit loop
   - Use distance/parallel/light to ASSIST movement direction
3. After color detected:
   - Wall align (parallel first!)
   - Rotate 90° directly (NO move to 24cm)
4. Phase 2 alignment and final approach:
   - Forward until 15cm from wall
   - Wall parallel align
   - Side strafe to 24cm
   - Step approach to 8cm (SKIP 20cm adjustment)

Key Changes from v2.20:
- Phase 1A: Approach to 31cm straight (NO wall parallel adjustment)
- Phase 1B: Start color detection after reaching 31cm
- Prevents early color detection before reaching target area
*****************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//============================================================
// PIN DEFINITIONS
//============================================================

//------------------------
// OLED Display (I2C)
//------------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//------------------------
// Color Sensor (TCS3200)
//------------------------
#define S0 4
#define S1 28
#define S2 7
#define S3 10
#define sensorOut 11

//------------------------
// Light Sensors (Photoresistors)
//------------------------
#define LIGHT_LEFT  A0
#define LIGHT_RIGHT A2

//------------------------
// Ultrasonic Sensors (HC-SR04)
//------------------------
#define TRIG_FRONT_LEFT   44
#define ECHO_FRONT_LEFT   45
#define TRIG_FRONT_RIGHT  46
#define ECHO_FRONT_RIGHT  13

#define TRIG_SIDE_LEFT    30
#define ECHO_SIDE_LEFT    29
#define TRIG_SIDE_RIGHT   33
#define ECHO_SIDE_RIGHT   32

//------------------------
// Motor Driver (L298N)
//------------------------
#define PWMA  12
#define DIRA1 34
#define DIRA2 35

#define PWMB  8
#define DIRB1 37
#define DIRB2 36

#define PWMC  6
#define DIRC1 43
#define DIRC2 42

#define PWMD  5
#define DIRD1 A4
#define DIRD2 A5

//------------------------
// Start Switch
//------------------------
#define START_SWITCH_PIN 25

//============================================================
// GLOBAL OBJECTS
//============================================================
MPU6050 mpu(Wire);

//============================================================
// GLOBAL VARIABLES
//============================================================

//------------------------
// Gyroscope Variables
//------------------------
float currentYaw = 0;
float rotationStartYaw = 0;

//------------------------
// Color Sensor Variables
//------------------------
int redFrequency = 0;
int greenFrequency = 0;
int redColor = 0;
int greenColor = 0;

// Auto-calibrated values (will be set during calibration)
int redMin = 999, redMax = 0;
int greenMin = 999, greenMax = 0;

// Default fallback values (if calibration fails)
int redMinDefault = 730, redMaxDefault = 770;
int greenMinDefault = 940, greenMaxDefault = 980;

//------------------------
// Light Sensor Variables
//------------------------
int lightLeftRaw = 0;
int lightRightRaw = 0;
int lightLeftBase = 0;
int lightRightBase = 0;
int lightLeftMin = 0;
int lightRightMin = 0;

int lightLeftIntensity = 0;
int lightRightIntensity = 0;

int avgLightLeft = 0;
int avgLightRight = 0;

//------------------------
// Ultrasonic Variables
//------------------------
long distance_front_left = 0;
long distance_front_right = 0;
long distance_side_left = 0;
long distance_side_right = 0;

long avgFrontLeft = 999;
long avgFrontRight = 999;

//------------------------
// Motor Speed (SEPARATED)
//------------------------
int Motor_PWM_Forward = 60;   // Forward/Backward speed
int Motor_PWM_Rotate = 50;    // Rotation speed (usually slower for precision)
int Motor_PWM_Strafe = 80;    // Strafe (left/right) speed

//------------------------
// Motor Calibration (Wheel Speed Compensation)
//------------------------
int LeftWheel_Adjust = 8;     // Left wheel PWM adjustment (+/-)
int RightWheel_Adjust = 0;    // Right wheel PWM adjustment (+/-)

//============================================================
// ROBOT STATE MACHINE
//============================================================
enum RobotState {
  STATE_IDLE,
  STATE_CALIBRATING_LIGHT,
  STATE_WAIT_RED_CALIBRATE,     // Waiting for button to start red calibration
  STATE_CALIBRATE_COLOR_RED,    // Calibrating red
  STATE_WAIT_GREEN_CALIBRATE,   // Waiting for button to start green calibration
  STATE_CALIBRATE_COLOR_GREEN,  // Calibrating green
  STATE_READY,
  STATE_MOVE_FORWARD,           // Phase 1 main movement loop
  STATE_ALIGN_WALL_ONLY,        // Wall align after color detected
  // STATE_MOVE_TO_TURN_POINT removed - turn directly after wall align
  STATE_ROTATING_90,
  STATE_MOVE_FORWARD_2,
  STATE_WALL_ALIGN_2,
  STATE_SIDE_DISTANCE_ALIGN,
  // STATE_ALIGN_DISTANCE_2 removed - skip 20cm adjustment
  STATE_FINAL_APPROACH,
  STATE_COMPLETE
};

RobotState currentState = STATE_IDLE;
RobotState previousState = STATE_IDLE;
char detectedColor = 'N';

//------------------------
// Calibration
//------------------------
int calibrateStep = 0;

//------------------------
// Tolerances & Distances
//------------------------
// Light alignment thresholds (percentage-based)
#define LIGHT_ERROR_PERCENT       10.0   // Max 10% difference between sensors
#define LIGHT_MIN_AVG_INTENSITY   80     // Min average intensity 80%

#define WALL_PARALLEL_TOL     2          // Wall parallel tolerance (cm)
#define MIN_SAFE_DIST         15         // Minimum safe distance
#define MAX_WALL_DIST         35         // Maximum distance to consider valid
#define COLOR_TURN_DIST       24         // Distance to turn after color (kept for reference)
#define FINAL_STOP_DIST       8          // Final stop distance
#define SIDE_TARGET_DIST      24         // Target side distance Phase 2
#define ROTATE_TARGET_ANGLE   80.0

// Phase 1 approach distance (start color detection after reaching this)
#define PHASE1_APPROACH_DIST  31         // Forward to 31cm before color detection

// Step-by-step approach distances
#define SLOW_APPROACH_DIST_PHASE2  15    // Stop at 15cm in Phase 2

const unsigned long ALIGN_SAMPLE_MS = 300;
const int ALIGN_PULSE_MS = 80;
const int DISTANCE_ADJUST_PULSE_MS = 100;
const int FORWARD_STEP_MS = 100;
const int FORWARD_MEASURE_PAUSE_MS = 20;
const int STEP_APPROACH_MS = 60;

//------------------------
// While Loop Safety Limits
//------------------------
const int MAX_STEPS_PHASE1 = 500;        // Increased since color is the only exit
const int MAX_STEPS_PHASE2 = 100;
const int MAX_ALIGN_LOOP = 50;
const int MAX_DISTANCE_ADJUST = 20;
const int MAX_SIDE_ALIGN_LOOP = 30;
const int MAX_STEP_APPROACH = 100;

//------------------------
// PID for Wall Alignment
//------------------------
float wallError = 0;
float wallPrevError = 0;
float wallIntegral = 0;
unsigned long wallPrevTime = 0;

float wallKp = 8.0;
float wallKi = 0.15;
float wallKd = 3.0;

int wallPidMinPulse = 25;
int wallPidMaxPulse = 140;
float wallIntegralLimit = 80.0;

//============================================================
// MOTOR CONTROL MACROS
//============================================================
#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW);  digitalWrite(DIRA2,HIGH); analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP()          do{digitalWrite(DIRA1,LOW);  digitalWrite(DIRA2,LOW);  analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH); digitalWrite(DIRA2,LOW);  analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,LOW);  digitalWrite(DIRB2,HIGH); analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP()          do{digitalWrite(DIRB1,LOW);  digitalWrite(DIRB2,LOW);  analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,HIGH); digitalWrite(DIRB2,LOW);  analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW);  digitalWrite(DIRC2,HIGH); analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP()          do{digitalWrite(DIRC1,LOW);  digitalWrite(DIRC2,LOW);  analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH); digitalWrite(DIRC2,LOW);  analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,LOW);  digitalWrite(DIRD2,HIGH); analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP()          do{digitalWrite(DIRD1,LOW);  digitalWrite(DIRD2,LOW);  analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,HIGH); digitalWrite(DIRD2,LOW);  analogWrite(PWMD,pwm);}while(0)

//============================================================
// MOTOR MOVEMENT FUNCTIONS (with wheel speed compensation)
//============================================================
void moveForward(int pwm) {
  int leftPwm = pwm + LeftWheel_Adjust;
  int rightPwm = pwm + RightWheel_Adjust;
  
  MOTORA_FORWARD(leftPwm);
  MOTORB_BACKOFF(rightPwm);
  MOTORC_FORWARD(leftPwm);
  MOTORD_BACKOFF(rightPwm);
}

void moveBackward(int pwm) {
  int leftPwm = pwm + LeftWheel_Adjust;
  int rightPwm = pwm + RightWheel_Adjust;
  
  MOTORA_BACKOFF(leftPwm);
  MOTORB_FORWARD(rightPwm);
  MOTORC_BACKOFF(leftPwm);
  MOTORD_FORWARD(rightPwm);
}

void moveLeft(int pwm) {
  MOTORA_BACKOFF(pwm);
  MOTORB_FORWARD(pwm);
  MOTORC_FORWARD(pwm);
  MOTORD_BACKOFF(pwm);
}

void moveRight(int pwm) {
  MOTORA_FORWARD(pwm);
  MOTORB_BACKOFF(pwm);
  MOTORC_BACKOFF(pwm);
  MOTORD_FORWARD(pwm);
}

void rotateLeft(int pwm) {
  MOTORA_BACKOFF(pwm);
  MOTORB_BACKOFF(pwm);
  MOTORC_BACKOFF(pwm);
  MOTORD_BACKOFF(pwm);
}

void rotateRight(int pwm) {
  MOTORA_FORWARD(pwm);
  MOTORB_FORWARD(pwm);
  MOTORC_FORWARD(pwm);
  MOTORD_FORWARD(pwm);
}

void stopAll() {
  MOTORA_STOP();
  MOTORB_STOP();
  MOTORC_STOP();
  MOTORD_STOP();
}

//============================================================
// OLED DISPLAY FUNCTIONS
//============================================================
void displayState(String line1, String line2, String line3) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println(line1);

  display.setCursor(0, 12);
  display.println(line2);

  display.setCursor(0, 24);
  display.println(line3);

  display.display();
}

void displayColorCalibrate(String colorName, int freq, int minVal, int maxVal, String instruction) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("CAL ");
  display.println(colorName);

  display.setCursor(0, 12);
  display.print("F:"); display.print(freq);
  display.print(" Min:"); display.print(minVal);
  display.print(" Max:"); display.println(maxVal);

  display.setCursor(0, 24);
  display.println(instruction);

  display.display();
}

void displayColor(int r, int g, char result) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("COLOR DETECT");

  display.setCursor(0, 12);
  display.print("R:"); display.print(r);
  display.print(" G:"); display.println(g);

  display.setCursor(0, 24);
  display.print("Result: ");
  if (result == 'R') display.println("RED");
  else if (result == 'G') display.println("GREEN");
  else display.println("---");

  display.display();
}

void displayRotation(float rotated) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("ROTATING");

  display.setCursor(0, 12);
  display.print("Angle: "); display.print((int)rotated); display.println(" deg");

  display.setCursor(0, 24);
  display.println(detectedColor == 'R' ? "RIGHT 90" : "LEFT 90");

  display.display();
}

void displayPhase1Status(int step, char colorResult, long fl, long fr, 
                         int ll, int lr, int wallErr, String assist) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("S"); display.print(step);
  display.print(" C:"); display.print(colorResult == 'R' ? "RED" : (colorResult == 'G' ? "GRN" : "--"));
  display.print(" W:"); display.println(wallErr);

  display.setCursor(0, 12);
  display.print("FL:"); display.print(fl);
  display.print(" FR:"); display.print(fr);
  display.print(" L:"); display.print(ll); display.print(" R:"); display.println(lr);

  display.setCursor(0, 24);
  display.print("Assist: "); display.println(assist);

  display.display();
}

void displayWallAlign(long fl, long fr, int wallErr, bool wallOK, String action) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("WALL ALIGN");
  display.print(" "); display.println(wallOK ? "OK" : "ADJ");

  display.setCursor(0, 12);
  display.print("FL:"); display.print(fl);
  display.print(" FR:"); display.print(fr);
  display.print(" Err:"); display.println(wallErr);

  display.setCursor(0, 24);
  display.println(action);

  display.display();
}

void displaySideAlign(long sideDist, int targetDist, String sensorName, String action) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("SIDE ALIGN ");
  display.println(sensorName);

  display.setCursor(0, 12);
  display.print("Side:"); display.print(sideDist);
  display.print("cm Tgt:"); display.print(targetDist); display.println("cm");

  display.setCursor(0, 24);
  display.println(action);

  display.display();
}

void displayStepApproach(long currentDist, int targetDist, String phase) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("STEP APPROACH ");
  display.println(phase);

  display.setCursor(0, 12);
  display.print("Now:"); display.print(currentDist);
  display.print(" Tgt:"); display.println(targetDist);

  display.setCursor(0, 24);
  display.println("Final stage");

  display.display();
}

//============================================================
// ULTRASONIC FUNCTIONS
//============================================================
long measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999;
  return (duration / 2.0) / 29.1;
}

void updateDistances() {
  distance_front_left = measureDistance(TRIG_FRONT_LEFT, ECHO_FRONT_LEFT);
  delay(5);
  distance_front_right = measureDistance(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT);
  delay(5);
  distance_side_left = measureDistance(TRIG_SIDE_LEFT, ECHO_SIDE_LEFT);
  delay(5);
  distance_side_right = measureDistance(TRIG_SIDE_RIGHT, ECHO_SIDE_RIGHT);
}

long getAverageFrontDistance() {
  if (distance_front_left == 999 && distance_front_right == 999) return 999;
  if (distance_front_left == 999) return distance_front_right;
  if (distance_front_right == 999) return distance_front_left;
  return (distance_front_left + distance_front_right) / 2;
}

void sampleFrontAverage(unsigned long sampleMs = ALIGN_SAMPLE_MS) {
  stopAll();

  unsigned long start = millis();
  long sumFL = 0, sumFR = 0;
  int countFL = 0, countFR = 0;

  while (millis() - start < sampleMs) {
    long fl = measureDistance(TRIG_FRONT_LEFT, ECHO_FRONT_LEFT);
    delay(5);
    long fr = measureDistance(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT);
    delay(5);

    if (fl != 999) {
      sumFL += fl;
      countFL++;
    }

    if (fr != 999) {
      sumFR += fr;
      countFR++;
    }
  }

  avgFrontLeft = (countFL > 0) ? (sumFL / countFL) : 999;
  avgFrontRight = (countFR > 0) ? (sumFR / countFR) : 999;

  distance_front_left = avgFrontLeft;
  distance_front_right = avgFrontRight;
}

long sampleSideDistance(int trigPin, int echoPin, unsigned long sampleMs = ALIGN_SAMPLE_MS) {
  stopAll();

  unsigned long start = millis();
  long sum = 0;
  int count = 0;

  while (millis() - start < sampleMs) {
    long dist = measureDistance(trigPin, echoPin);
    delay(5);

    if (dist != 999) {
      sum += dist;
      count++;
    }
  }

  return (count > 0) ? (sum / count) : 999;
}

//============================================================
// GYROSCOPE FUNCTIONS
//============================================================
void updateGyro() {
  mpu.update();
  currentYaw = mpu.getAngleZ();
}

float getRotatedAngle(float startAngle, float currentAngle) {
  float diff = currentAngle - startAngle;
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;
  return abs(diff);
}

//============================================================
// LIGHT SENSOR FUNCTIONS
//============================================================
void readLightSensors() {
  lightLeftRaw = analogRead(LIGHT_LEFT);
  lightRightRaw = analogRead(LIGHT_RIGHT);
}

void calculateIntensity() {
  int leftRange = lightLeftBase - lightLeftMin;
  int rightRange = lightRightBase - lightRightMin;

  if (leftRange <= 0) leftRange = 1;
  if (rightRange <= 0) rightRange = 1;

  lightLeftIntensity = ((lightLeftRaw - lightLeftMin) * 100) / leftRange;
  lightRightIntensity = ((lightRightRaw - lightRightMin) * 100) / rightRange;

  lightLeftIntensity = constrain(lightLeftIntensity, 0, 100);
  lightRightIntensity = constrain(lightRightIntensity, 0, 100);
}

void calibrateLightAmbient() {
  displayState("LIGHT CAL", "Ambient light", "Keep open");
  delay(2000);

  int samples = 20;
  lightLeftBase = 0;
  lightRightBase = 0;

  for (int i = 0; i < samples; i++) {
    lightLeftBase += analogRead(LIGHT_LEFT);
    lightRightBase += analogRead(LIGHT_RIGHT);
    delay(50);
  }

  lightLeftBase /= samples;
  lightRightBase /= samples;

  Serial.print("Ambient - L:"); Serial.print(lightLeftBase);
  Serial.print(" R:"); Serial.println(lightRightBase);

  displayState("BASE OK", "L:" + String(lightLeftBase), "R:" + String(lightRightBase));
  delay(1000);
}

void calibrateLightDark() {
  displayState("LIGHT CAL", "Cover sensors!", "");
  delay(3000);

  displayState("CALIBRATING", "Keep covered!", "");

  int samples = 20;
  lightLeftMin = 0;
  lightRightMin = 0;

  for (int i = 0; i < samples; i++) {
    lightLeftMin += analogRead(LIGHT_LEFT);
    lightRightMin += analogRead(LIGHT_RIGHT);
    delay(50);
  }

  lightLeftMin /= samples;
  lightRightMin /= samples;

  Serial.print("Dark - L:"); Serial.print(lightLeftMin);
  Serial.print(" R:"); Serial.println(lightRightMin);

  displayState("MIN OK", "L:" + String(lightLeftMin), "R:" + String(lightRightMin));
  delay(1000);
}

//============================================================
// LIGHT CONDITION CHECK FUNCTIONS
//============================================================
float calculateLightErrorPercent() {
  int diff = abs(lightLeftIntensity - lightRightIntensity);
  int maxIntensity = max(lightLeftIntensity, lightRightIntensity);

  if (maxIntensity == 0) return 100.0;
  return (float)diff * 100.0 / (float)maxIntensity;
}

int getAverageLightIntensity() {
  return (lightLeftIntensity + lightRightIntensity) / 2;
}

bool isLightIntensityHighEnough() {
  int avgIntensity = getAverageLightIntensity();
  return (avgIntensity >= LIGHT_MIN_AVG_INTENSITY);
}

//============================================================
// COLOR SENSOR FUNCTIONS
//============================================================
void readColorSensor() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  redFrequency = pulseIn(sensorOut, LOW);
  redColor = map(redFrequency, redMin, redMax, 255, 0);
  redColor = constrain(redColor, 0, 255);

  delay(50);

  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greenFrequency = pulseIn(sensorOut, LOW);
  greenColor = map(greenFrequency, greenMin, greenMax, 255, 0);
  greenColor = constrain(greenColor, 0, 255);
}

int readRedFrequency() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  return pulseIn(sensorOut, LOW);
}

int readGreenFrequency() {
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  return pulseIn(sensorOut, LOW);
}

char detectColor() {
  readColorSensor();

  Serial.print("Color - R:"); Serial.print(redColor);
  Serial.print(" G:"); Serial.println(greenColor);
  Serial.print("  Freq - R:"); Serial.print(redFrequency);
  Serial.print(" G:"); Serial.println(greenFrequency);

  displayColor(redColor, greenColor, 'N');

  if (redColor > greenColor + 20) {
    Serial.println("RED detected!");
    displayColor(redColor, greenColor, 'R');
    return 'R';
  }
  else if (greenColor > redColor + 20) {
    Serial.println("GREEN detected!");
    displayColor(redColor, greenColor, 'G');
    return 'G';
  }
  return 'N';
}

//============================================================
// COLOR CALIBRATION FUNCTIONS
//============================================================
void doColorCalibrateRed() {
  static int sampleCount = 0;
  static unsigned long lastSampleTime = 0;
  const int TOTAL_SAMPLES = 30;
  
  if (sampleCount == 0) {
    redMin = 999;
    redMax = 0;
    sampleCount = 1;
    displayColorCalibrate("RED", 0, redMin, redMax, "Place RED");
    Serial.println("=== RED Color Calibration Start ===");
    Serial.println("Place RED color in front of sensor!");
    delay(2000);
    return;
  }
  
  if (millis() - lastSampleTime < 100) {
    return;
  }
  lastSampleTime = millis();
  
  int freq = readRedFrequency();
  delay(10);
  
  if (freq > 0 && freq < 999) {
    if (freq < redMin) redMin = freq;
    if (freq > redMax) redMax = freq;
  }
  
  sampleCount++;
  
  displayColorCalibrate("RED", freq, redMin, redMax, 
                        "Sampling " + String(sampleCount) + "/" + String(TOTAL_SAMPLES));
  
  Serial.print("Red sample "); Serial.print(sampleCount);
  Serial.print(" Freq:"); Serial.print(freq);
  Serial.print(" Min:"); Serial.print(redMin);
  Serial.print(" Max:"); Serial.println(redMax);
  
  if (sampleCount >= TOTAL_SAMPLES) {
    if (redMax > redMin) {
      Serial.println("=== RED Calibration Complete ===");
      Serial.print("Red Min:"); Serial.print(redMin);
      Serial.print(" Max:"); Serial.println(redMax);
      displayState("RED CAL OK", "Min:" + String(redMin), "Max:" + String(redMax));
    } else {
      redMin = redMinDefault;
      redMax = redMaxDefault;
      Serial.println("=== RED Calibration FAILED, using defaults ===");
      displayState("RED CAL FAIL", "Using defaults", "");
    }
    delay(1500);
    sampleCount = 0;
    currentState = STATE_WAIT_GREEN_CALIBRATE;
  }
}

void doColorCalibrateGreen() {
  static int sampleCount = 0;
  static unsigned long lastSampleTime = 0;
  const int TOTAL_SAMPLES = 30;
  
  if (sampleCount == 0) {
    greenMin = 999;
    greenMax = 0;
    sampleCount = 1;
    displayColorCalibrate("GREEN", 0, greenMin, greenMax, "Place GREEN");
    Serial.println("=== GREEN Color Calibration Start ===");
    Serial.println("Place GREEN color in front of sensor!");
    delay(2000);
    return;
  }
  
  if (millis() - lastSampleTime < 100) {
    return;
  }
  lastSampleTime = millis();
  
  int freq = readGreenFrequency();
  delay(10);
  
  if (freq > 0 && freq < 999) {
    if (freq < greenMin) greenMin = freq;
    if (freq > greenMax) greenMax = freq;
  }
  
  sampleCount++;
  
  displayColorCalibrate("GREEN", freq, greenMin, greenMax, 
                        "Sampling " + String(sampleCount) + "/" + String(TOTAL_SAMPLES));
  
  Serial.print("Green sample "); Serial.print(sampleCount);
  Serial.print(" Freq:"); Serial.print(freq);
  Serial.print(" Min:"); Serial.print(greenMin);
  Serial.print(" Max:"); Serial.println(greenMax);
  
  if (sampleCount >= TOTAL_SAMPLES) {
    if (greenMax > greenMin) {
      Serial.println("=== GREEN Calibration Complete ===");
      Serial.print("Green Min:"); Serial.print(greenMin);
      Serial.print(" Max:"); Serial.println(greenMax);
      displayState("GREEN CAL OK", "Min:" + String(greenMin), "Max:" + String(greenMax));
    } else {
      greenMin = greenMinDefault;
      greenMax = greenMaxDefault;
      Serial.println("=== GREEN Calibration FAILED, using defaults ===");
      displayState("GREEN CAL FAIL", "Using defaults", "");
    }
    delay(1500);
    sampleCount = 0;
    currentState = STATE_READY;
  }
}

//============================================================
// PID WALL ALIGN FUNCTIONS
//============================================================
void resetWallPID() {
  wallError = 0;
  wallPrevError = 0;
  wallIntegral = 0;
  wallPrevTime = 0;
}

int computeWallPIDPulse(float error) {
  unsigned long now = millis();
  float dt = (wallPrevTime == 0) ? 0.05 : (now - wallPrevTime) / 1000.0;

  if (dt <= 0) dt = 0.05;
  if (dt > 0.3) dt = 0.3;

  wallIntegral += error * dt;
  wallIntegral = constrain(wallIntegral, -wallIntegralLimit, wallIntegralLimit);

  float derivative = (error - wallPrevError) / dt;
  float output = wallKp * error + wallKi * wallIntegral + wallKd * derivative;

  wallPrevError = error;
  wallPrevTime = now;

  int pulse = (int)abs(output);
  pulse = constrain(pulse, wallPidMinPulse, wallPidMaxPulse);

  return pulse;
}

//============================================================
// WALL ALIGNMENT FUNCTIONS
//============================================================
int wallAlignLoopCount = 0;

void resetWallAlign() {
  wallAlignLoopCount = 0;
  resetWallPID();
}

void doWallAlignmentOnly(RobotState nextState) {
  
  wallAlignLoopCount++;
  
  if (wallAlignLoopCount > MAX_ALIGN_LOOP) {
    stopAll();
    displayState("WALL TIMEOUT", "Max loops", "Proceeding");
    Serial.println(">>> Wall alignment timeout!");
    delay(300);
    resetWallAlign();
    currentState = nextState;
    return;
  }

  sampleFrontAverage(ALIGN_SAMPLE_MS);

  int wallErr = distance_front_left - distance_front_right;
  bool wallOK = abs(wallErr) <= WALL_PARALLEL_TOL;

  displayWallAlign(distance_front_left, distance_front_right, wallErr, wallOK, "");

  Serial.print("Wall Align Loop "); Serial.print(wallAlignLoopCount);
  Serial.print(" | FL:"); Serial.print(distance_front_left);
  Serial.print(" FR:"); Serial.print(distance_front_right);
  Serial.print(" Err:"); Serial.print(wallErr);
  Serial.print(" OK:"); Serial.println(wallOK ? "YES" : "NO");

  if (wallOK) {
    stopAll();
    displayWallAlign(distance_front_left, distance_front_right, wallErr, wallOK, "OK!");
    Serial.println(">>> Wall parallel!");
    delay(300);
    resetWallAlign();
    currentState = nextState;
    return;
  }

  int pulseMs = computeWallPIDPulse(wallErr);
  
  Serial.print(">>> Adjusting wall, pulse: "); Serial.println(pulseMs);
  displayWallAlign(distance_front_left, distance_front_right, wallErr, wallOK, "ROTATE");

  if (wallErr > 0) {
    rotateRight(Motor_PWM_Rotate);
  } else {
    rotateLeft(Motor_PWM_Rotate);
  }
  
  delay(pulseMs);
  stopAll();
  delay(50);
}

//============================================================
// SIDE DISTANCE ALIGNMENT (Phase 2)
//============================================================
int sideAlignLoopCount = 0;

void resetSideAlign() {
  sideAlignLoopCount = 0;
}

void doSideDistanceAlignment(RobotState nextState) {
  
  sideAlignLoopCount++;
  
  if (sideAlignLoopCount > MAX_SIDE_ALIGN_LOOP) {
    stopAll();
    displayState("SIDE TIMEOUT", "Max loops", "Proceeding");
    Serial.println(">>> Side distance alignment timeout!");
    delay(300);
    resetSideAlign();
    currentState = nextState;
    return;
  }

  int trigPin, echoPin;
  String sensorName;
  
  if (detectedColor == 'G') {
    trigPin = TRIG_SIDE_RIGHT;
    echoPin = ECHO_SIDE_RIGHT;
    sensorName = "RIGHT";
  } else {
    trigPin = TRIG_SIDE_LEFT;
    echoPin = ECHO_SIDE_LEFT;
    sensorName = "LEFT";
  }

  long sideDist = sampleSideDistance(trigPin, echoPin, ALIGN_SAMPLE_MS);
  
  if (sideDist == 999) {
    stopAll();
    displaySideAlign(sideDist, SIDE_TARGET_DIST, sensorName, "NO READING");
    Serial.println(">>> Side sensor no reading!");
    delay(100);
    return;
  }

  int sideErr = sideDist - SIDE_TARGET_DIST;
  
  Serial.print("Side Align Loop "); Serial.print(sideAlignLoopCount);
  Serial.print(" | Sensor:"); Serial.print(sensorName);
  Serial.print(" Dist:"); Serial.print(sideDist);
  Serial.print(" Target:"); Serial.print(SIDE_TARGET_DIST);
  Serial.print(" Err:"); Serial.println(sideErr);

  if (abs(sideErr) <= 1) {
    stopAll();
    displaySideAlign(sideDist, SIDE_TARGET_DIST, sensorName, "OK!");
    Serial.println(">>> Side distance aligned!");
    delay(300);
    resetSideAlign();
    currentState = nextState;
    return;
  }

  String action;
  
  if (detectedColor == 'G') {
    if (sideErr > 0) {
      action = "RIGHT ->";
      moveRight(Motor_PWM_Strafe);
    } else {
      action = "<- LEFT";
      moveLeft(Motor_PWM_Strafe);
    }
  } else {
    if (sideErr > 0) {
      action = "<- LEFT";
      moveLeft(Motor_PWM_Strafe);
    } else {
      action = "RIGHT ->";
      moveRight(Motor_PWM_Strafe);
    }
  }

  displaySideAlign(sideDist, SIDE_TARGET_DIST, sensorName, action);
  Serial.print(">>> Action: "); Serial.println(action);

  delay(ALIGN_PULSE_MS);
  stopAll();
  delay(50);
}

//============================================================
// STEP-BY-STEP DISTANCE APPROACH FUNCTIONS
//============================================================
void doStepByStepApproachPhase2(RobotState nextState) {
  
  static int stepCount = 0;
  
  if (stepCount == 0) {
    stepCount = 1;
    Serial.println("=== Phase 2 Step-by-Step Approach (15cm -> 8cm) ===");
  }
  
  if (stepCount > MAX_STEP_APPROACH) {
    stopAll();
    displayState("APPROACH DONE", "Max steps", "COMPLETE!");
    Serial.println(">>> Final approach timeout!");
    delay(300);
    stepCount = 0;
    currentState = nextState;
    return;
  }

  updateDistances();
  long avgDist = getAverageFrontDistance();

  if (avgDist == 999) {
    stopAll();
    displayStepApproach(999, FINAL_STOP_DIST, "P2");
    delay(100);
    return;
  }

  Serial.print("Step Approach P2 | Step:"); Serial.print(stepCount);
  Serial.print(" Dist:"); Serial.print(avgDist);
  Serial.print(" Target:"); Serial.println(FINAL_STOP_DIST);

  displayStepApproach(avgDist, FINAL_STOP_DIST, "P2");

  if (avgDist <= FINAL_STOP_DIST) {
    stopAll();
    displayStepApproach(avgDist, FINAL_STOP_DIST, "OK!");
    Serial.println(">>> Phase 2 target reached!");
    delay(300);
    stepCount = 0;
    currentState = nextState;
    return;
  }

  if (avgDist > FINAL_STOP_DIST) {
    moveForward(Motor_PWM_Forward);
    delay(STEP_APPROACH_MS);
    stopAll();
    delay(FORWARD_MEASURE_PAUSE_MS);
  }

  stepCount++;
}

//============================================================
// AUTONOMOUS ROBOT LOGIC
//============================================================
void autonomousControl() {
  if (currentState == STATE_FINAL_APPROACH || 
      currentState == STATE_MOVE_FORWARD_2 || 
      currentState == STATE_SIDE_DISTANCE_ALIGN || 
      currentState == STATE_ALIGN_WALL_ONLY) {
    updateDistances();
  }

  bool stateChanged = (currentState != previousState);
  if (stateChanged) {
    previousState = currentState;
  }

  switch (currentState) {

    case STATE_IDLE:
      stopAll();
      if (stateChanged) {
        displayState("IDLE", "Press switch", "to calibrate");
      }
      break;

    case STATE_CALIBRATING_LIGHT:
      // Handled in loop() with button presses
      break;

    case STATE_WAIT_RED_CALIBRATE:
      if (stateChanged) {
        displayState("RED CAL", "Place RED", "Press button");
      }
      // Waiting for button press in loop()
      break;

    case STATE_CALIBRATE_COLOR_RED:
      doColorCalibrateRed();
      break;

    case STATE_WAIT_GREEN_CALIBRATE:
      if (stateChanged) {
        displayState("GREEN CAL", "Place GREEN", "Press button");
      }
      // Waiting for button press in loop()
      break;

    case STATE_CALIBRATE_COLOR_GREEN:
      doColorCalibrateGreen();
      break;

    case STATE_READY:
      stopAll();
      if (stateChanged) {
        displayState("READY!", "Press switch", "to START");
      }
      break;

    //============================================================
    // PHASE 1: Move Forward (First approach 31cm, then detect color)
    //============================================================
    case STATE_MOVE_FORWARD:
      {
        if (stateChanged) {
          Serial.println("=== Phase 1: Move Forward ===");
          Serial.println("Step 1: Approach to 31cm (NO color detection)");
          Serial.println("Step 2: Then detect color");
        }

        bool keepMoving = true;
        int stepCount = 0;
        String assistAction = "";
        bool approachComplete = false;  // Flag for 31cm approach

        // ============================================================
        // PHASE 1A: Approach to 31cm FIRST (no color detection, JUST STRAIGHT)
        // ============================================================
        Serial.println(">>> Phase 1A: Approaching 31cm (straight, no parallel adjust)...");
        
        while (!approachComplete && stepCount < MAX_STEPS_PHASE1) {
          // Read distance sensors
          long fl = measureDistance(TRIG_FRONT_LEFT, ECHO_FRONT_LEFT);
          delay(5);
          long fr = measureDistance(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT);
          delay(5);
          
          distance_front_left = fl;
          distance_front_right = fr;
          long avgDist = getAverageFrontDistance();
          
          // Display approach status
          display.clearDisplay();
          display.setTextSize(1);
          display.setTextColor(SSD1306_WHITE);
          display.setCursor(0, 0);
          display.print("APPROACH 31cm");
          display.setCursor(0, 12);
          display.print("FL:"); display.print(fl);
          display.print(" FR:"); display.println(fr);
          display.setCursor(0, 24);
          display.print("Avg:"); display.print(avgDist);
          display.print(" Step:"); display.println(stepCount);
          display.display();
          
          Serial.print("Approach Step "); Serial.print(stepCount);
          Serial.print(" | FL:"); Serial.print(fl);
          Serial.print(" FR:"); Serial.print(fr);
          Serial.print(" Avg:"); Serial.println(avgDist);
          
          // Check if reached 31cm
          if (avgDist != 999 && avgDist <= PHASE1_APPROACH_DIST) {
            approachComplete = true;
            Serial.println(">>> Reached 31cm! Starting color detection...");
            displayState("AT 31cm", "Color detect ON", "");
            delay(300);
            break;
          }
          
          // NO wall parallel adjustment - just go straight!
          
          // Forward movement
          moveForward(Motor_PWM_Forward);
          delay(FORWARD_STEP_MS);
          stopAll();
          delay(FORWARD_MEASURE_PAUSE_MS);
          
          stepCount++;
        }
        
        // Safety check - if approach failed
        if (!approachComplete) {
          Serial.println(">>> Approach timeout, proceeding anyway...");
          displayState("APPROACH TIMEOUT", "Color detect ON", "");
          delay(500);
        }

        // ============================================================
        // PHASE 1B: Color Detection Loop (after reaching 31cm)
        // ============================================================
        Serial.println(">>> Phase 1B: Color Detection Loop");
        Serial.println(">>> Distance/Parallel/Light = ASSIST movement");

        while (keepMoving && stepCount < MAX_STEPS_PHASE1) {
          
          // ============================================================
          // STEP 1: Check color FIRST (EXIT CONDITION)
          // ============================================================
          readColorSensor();
          char colorResult = 'N';
          
          if (redColor > greenColor + 20) {
            colorResult = 'R';
          } else if (greenColor > redColor + 20) {
            colorResult = 'G';
          }

          // ============================================================
          // STEP 2: Read sensors for assistance
          // ============================================================
          long fl = measureDistance(TRIG_FRONT_LEFT, ECHO_FRONT_LEFT);
          delay(5);
          long fr = measureDistance(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT);
          delay(5);
          
          distance_front_left = fl;
          distance_front_right = fr;
          
          readLightSensors();
          calculateIntensity();
          
          int wallErr = fl - fr;
          float lightErr = calculateLightErrorPercent();
          
          // ============================================================
          // STEP 3: Check EXIT condition (Color detected)
          // ============================================================
          if (colorResult != 'N') {
            detectedColor = colorResult;
            String colorStr = (detectedColor == 'R') ? "RED->Right" : "GREEN->Left";
            displayState("COLOR FOUND!", colorStr, "Wall align then turn");
            Serial.print(">>> COLOR DETECTED: "); Serial.println(colorStr);
            Serial.println(">>> EXITING Phase 1 loop!");
            Serial.println(">>> Next: Wall align -> Turn directly");
            delay(500);
            break;  // EXIT the while loop
          }

          // ============================================================
          // STEP 4: ASSIST movement (NOT exit conditions)
          // ============================================================
          assistAction = "FWD";
          
          // Assist 1: Avoid getting too close to wall
          long avgDist = getAverageFrontDistance();
          if (avgDist != 999 && avgDist <= MIN_SAFE_DIST) {
            // Too close, back up a bit
            displayPhase1Status(stepCount, colorResult, fl, fr, 
                               lightLeftIntensity, lightRightIntensity, wallErr, "BACKUP");
            Serial.println(">>> Assist: Too close, backing up");
            moveBackward(Motor_PWM_Forward);
            delay(FORWARD_STEP_MS);
            stopAll();
            delay(FORWARD_MEASURE_PAUSE_MS);
            stepCount++;
            continue;  // Skip to next iteration
          }
          
          // Assist 2: Wall parallel adjustment (small rotation while moving)
          if (abs(wallErr) > WALL_PARALLEL_TOL && fl != 999 && fr != 999) {
            int pulseMs = computeWallPIDPulse(wallErr);
            
            displayPhase1Status(stepCount, colorResult, fl, fr, 
                               lightLeftIntensity, lightRightIntensity, wallErr, "ROTATE");
            Serial.print(">>> Assist: Wall parallel adjust, pulse: "); Serial.println(pulseMs);
            
            if (wallErr > 0) {
              rotateRight(Motor_PWM_Rotate);
            } else {
              rotateLeft(Motor_PWM_Rotate);
            }
            delay(pulseMs);
            stopAll();
            delay(50);
            resetWallPID();
          }
          
          // Assist 3: Light alignment (strafe towards light)
          if (lightErr > LIGHT_ERROR_PERCENT && isLightIntensityHighEnough()) {
            displayPhase1Status(stepCount, colorResult, fl, fr, 
                               lightLeftIntensity, lightRightIntensity, wallErr, "STRAFE");
            Serial.print(">>> Assist: Light align, L:"); Serial.print(lightLeftIntensity);
            Serial.print(" R:"); Serial.println(lightRightIntensity);
            
            if (lightLeftIntensity > lightRightIntensity) {
              moveLeft(Motor_PWM_Strafe);
            } else {
              moveRight(Motor_PWM_Strafe);
            }
            delay(ALIGN_PULSE_MS);
            stopAll();
            delay(50);
          }

          // ============================================================
          // STEP 5: Normal forward movement
          // ============================================================
          displayPhase1Status(stepCount, colorResult, fl, fr, 
                             lightLeftIntensity, lightRightIntensity, wallErr, assistAction);
          
          Serial.print("Step "); Serial.print(stepCount);
          Serial.print(" | Color:"); Serial.print(colorResult);
          Serial.print(" | FL:"); Serial.print(fl);
          Serial.print(" FR:"); Serial.print(fr);
          Serial.print(" | WallErr:"); Serial.print(wallErr);
          Serial.print(" | LightErr:"); Serial.print(lightErr, 1);
          Serial.print("% | Action:"); Serial.println(assistAction);
          
          moveForward(Motor_PWM_Forward);
          delay(FORWARD_STEP_MS);
          stopAll();
          delay(FORWARD_MEASURE_PAUSE_MS);
          
          stepCount++;
        }

        // After loop exits (color found or max steps)
        stopAll();
        Serial.print("Phase 1 complete: "); Serial.print(stepCount);
        Serial.print(" steps, Color: "); Serial.println(detectedColor);
        
        if (detectedColor != 'N') {
          currentState = STATE_ALIGN_WALL_ONLY;
        } else {
          displayState("NO COLOR", "Max steps", "Retry?");
          delay(1000);
          currentState = STATE_READY;
        }
      }
      break;

    //============================================================
    // Wall Only Alignment (After color detected) -> Turn directly
    //============================================================
    case STATE_ALIGN_WALL_ONLY:
      if (stateChanged) {
        resetWallAlign();
        String colorStr = (detectedColor == 'R') ? "RED->Right" : "GREEN->Left";
        displayState("WALL ALIGN", colorStr, "Then turn");
        Serial.print("=== Wall Alignment (Color: "); Serial.print(detectedColor); Serial.println(") ===");
        Serial.println(">>> After wall align, will turn directly (no move to 24cm)");
      }
      // Wall align FIRST, then turn directly (skip STATE_MOVE_TO_TURN_POINT)
      doWallAlignmentOnly(STATE_ROTATING_90);
      break;

    case STATE_ROTATING_90:
      {
        if (stateChanged) {
          for (int i = 0; i < 5; i++) {
            mpu.update();
            delay(5);
          }
          rotationStartYaw = mpu.getAngleZ();
          displayState("ROTATING", "90 degrees", "");
        }

        updateGyro();

        float rotated = getRotatedAngle(rotationStartYaw, currentYaw);
        displayRotation(rotated);

        if (rotated >= ROTATE_TARGET_ANGLE) {
          stopAll();
          displayState("ROTATED!", "Done", "");
          delay(500);
          currentState = STATE_MOVE_FORWARD_2;
        } else {
          if (detectedColor == 'R') {
            rotateRight(Motor_PWM_Rotate);
          } else {
            rotateLeft(Motor_PWM_Rotate);
          }
        }
      }
      break;

    //============================================================
    // PHASE 2: Move Forward until 15cm from wall
    //============================================================
    case STATE_MOVE_FORWARD_2:
      {
        if (stateChanged) {
          displayState("MOVE FWD 2", "Stop at 15cm", "");
          Serial.println("=== Phase 2: Move Forward until 15cm ===");
        }

        bool keepMoving = true;
        int stepCount2 = 0;

        while (keepMoving && stepCount2 < MAX_STEPS_PHASE2) {
          moveForward(Motor_PWM_Forward);
          delay(FORWARD_STEP_MS);
          stopAll();
          delay(FORWARD_MEASURE_PAUSE_MS);

          long fl = measureDistance(TRIG_FRONT_LEFT, ECHO_FRONT_LEFT);
          delay(5);
          long fr = measureDistance(TRIG_FRONT_RIGHT, ECHO_FRONT_RIGHT);
          delay(5);

          distance_front_left = fl;
          distance_front_right = fr;

          stepCount2++;

          Serial.print("Step2 "); Serial.print(stepCount2);
          Serial.print(" | FL:"); Serial.print(fl);
          Serial.print(" FR:"); Serial.println(fr);

          // Stop when reaching 15cm from wall
          long avgDist = getAverageFrontDistance();
          if (avgDist != 999 && avgDist <= SLOW_APPROACH_DIST_PHASE2) {
            keepMoving = false;
            Serial.println(">>> Reached 15cm stop point!");
          }
        }

        stopAll();
        Serial.print("Phase 2 forward complete: "); Serial.print(stepCount2); Serial.println(" steps");
        currentState = STATE_WALL_ALIGN_2;
      }
      break;

    //============================================================
    // PHASE 2: Wall Parallel Alignment
    //============================================================
    case STATE_WALL_ALIGN_2:
      if (stateChanged) {
        resetWallAlign();
        displayState("WALL ALIGN 2", "Parallel only", "");
        Serial.println("=== Wall Alignment Phase 2 ===");
      }
      doWallAlignmentOnly(STATE_SIDE_DISTANCE_ALIGN);
      break;

    //============================================================
    // PHASE 2: Side Distance Alignment (strafe to 24cm)
    //============================================================
    case STATE_SIDE_DISTANCE_ALIGN:
      if (stateChanged) {
        resetSideAlign();
        String turnDir = (detectedColor == 'G') ? "L->RIGHT sens" : "R->LEFT sens";
        displayState("SIDE ALIGN", turnDir, "Target: 24cm");
        Serial.println("=== Side Distance Alignment (24cm) ===");
      }
      // Skip STATE_ALIGN_DISTANCE_2, go directly to STATE_FINAL_APPROACH
      doSideDistanceAlignment(STATE_FINAL_APPROACH);
      break;

    //============================================================
    // PHASE 2: Final Step-by-Step Approach (to 8cm)
    //============================================================
    case STATE_FINAL_APPROACH:
      if (stateChanged) {
        displayState("FINAL APPROACH", "15cm -> 8cm", "");
        Serial.println("=== Final Step-by-Step Approach (15cm -> 8cm) ===");
      }
      doStepByStepApproachPhase2(STATE_COMPLETE);
      break;

    case STATE_COMPLETE:
      stopAll();
      if (stateChanged) {
        displayState("COMPLETE!", "Task done!", ":)");
        Serial.println("========== COMPLETE! ==========");
      }
      break;
  }
}

//============================================================
// SETUP
//============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("================================");
  Serial.println("Light Tracking Robot v2.21");
  Serial.println("Phase 1: Approach 31cm -> Color detect");
  Serial.println("Phase 2: 15cm->parallel->24cm->8cm");
  Serial.println("================================");

  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 failed"));
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("v2.21");
  display.display();
  delay(1000);

  Serial.println("Init MPU6050...");
  mpu.begin();
  Serial.println("MPU6050 ready");

  pinMode(PWMA, OUTPUT);
  pinMode(DIRA1, OUTPUT);
  pinMode(DIRA2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRB1, OUTPUT);
  pinMode(DIRB2, OUTPUT);
  pinMode(PWMC, OUTPUT);
  pinMode(DIRC1, OUTPUT);
  pinMode(DIRC2, OUTPUT);
  pinMode(PWMD, OUTPUT);
  pinMode(DIRD1, OUTPUT);
  pinMode(DIRD2, OUTPUT);
  stopAll();

  pinMode(TRIG_FRONT_LEFT, OUTPUT);
  pinMode(ECHO_FRONT_LEFT, INPUT);
  pinMode(TRIG_FRONT_RIGHT, OUTPUT);
  pinMode(ECHO_FRONT_RIGHT, INPUT);
  pinMode(TRIG_SIDE_LEFT, OUTPUT);
  pinMode(ECHO_SIDE_LEFT, INPUT);
  pinMode(TRIG_SIDE_RIGHT, OUTPUT);
  pinMode(ECHO_SIDE_RIGHT, INPUT);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  pinMode(LIGHT_LEFT, INPUT);
  pinMode(LIGHT_RIGHT, INPUT);

  pinMode(START_SWITCH_PIN, INPUT_PULLUP);

  displayState("IDLE", "Press switch", "to calibrate");
  Serial.println("Setup complete!");
}

//============================================================
// MAIN LOOP
//============================================================
void loop() {
  static bool switchPressed = false;
  static unsigned long lastPressTime = 0;

  if (digitalRead(START_SWITCH_PIN) == LOW && !switchPressed) {
    if (millis() - lastPressTime > 500) {
      switchPressed = true;
      lastPressTime = millis();

      // Handle button press based on current state
      if (currentState == STATE_IDLE) {
        currentState = STATE_CALIBRATING_LIGHT;
        calibrateStep = 0;
        displayState("CALIBRATION", "Press for base", "");
      }
      else if (currentState == STATE_CALIBRATING_LIGHT) {
        if (calibrateStep == 0) {
          calibrateLightAmbient();
          calibrateStep = 1;
          displayState("CAL STEP 2", "Press for dark", "");
        } else if (calibrateStep == 1) {
          calibrateLightDark();
          // After light calibration, wait for RED calibration
          currentState = STATE_WAIT_RED_CALIBRATE;
        }
      }
      else if (currentState == STATE_WAIT_RED_CALIBRATE) {
        // Button pressed, start RED calibration
        currentState = STATE_CALIBRATE_COLOR_RED;
      }
      else if (currentState == STATE_WAIT_GREEN_CALIBRATE) {
        // Button pressed, start GREEN calibration
        currentState = STATE_CALIBRATE_COLOR_GREEN;
      }
      else if (currentState == STATE_READY) {
        currentState = STATE_MOVE_FORWARD;
      }
    }
  }

  if (digitalRead(START_SWITCH_PIN) == HIGH) {
    switchPressed = false;
  }

  // Run autonomous control for all states except waiting states
  if (currentState != STATE_CALIBRATING_LIGHT) {
    autonomousControl();
  }

  delay(10);
}
