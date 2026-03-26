/*****************************************************
Light Tracking Robot with Color Detection + PID Wall Align
Version: 2.16 (Simplified: Direct distance adjust to 28cm)
Date: 2026-03-26

Flow:
1. Move forward (while loop with conditions)
2. Stop when wall detected
3. Combined alignment loop (Phase 1):
   - Check parallel wall first
   - Check light intensity: Error < 3%, Average >= 80%
   - Adjust until both conditions met
4. Adjust distance to 28cm (step-by-step forward or backward)
5. Detect color
6. Move forward until front average distance = 24cm
7. Rotate 90° (gyro)
8. Move forward (while loop)
9. Phase 2 alignment:
   - Wall parallel alignment (front sensors)
   - Side distance alignment (side sensor based on turn direction)
   - Front distance adjustment: 20cm
10. Final approach: step-by-step from 15cm to 8cm
11. Complete

Changes from v2.15:
- Removed STATE_RECHECK_PARALLEL
- Removed STATE_RECHECK_LIGHT
- Simplified flow: ALIGN_COMBINED -> ADJUST_TO_28CM -> DETECT_COLOR
- Direct step-by-step distance adjustment to 28cm
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

int redMin = 390, redMax = 480;
int greenMin = 630, greenMax = 670;

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
int Motor_PWM_Rotate = 70;    // Rotation speed (usually slower for precision)
int Motor_PWM_Strafe = 80;    // Strafe (left/right) speed

//------------------------
// Motor Calibration (Wheel Speed Compensation)
//------------------------
// If robot drifts left, increase RightWheel_Adjust (positive value)
// If robot drifts right, increase LeftWheel_Adjust (positive value)
// Adjust percentage: 2% of 60 = ~1.2, use integer values
int LeftWheel_Adjust = 6;     // Left wheel PWM adjustment (+/-)
int RightWheel_Adjust = 0;    // Right wheel PWM adjustment (+/-), +2 to compensate slower right wheel

//============================================================
// ROBOT STATE MACHINE
//============================================================
enum RobotState {
  STATE_IDLE,
  STATE_CALIBRATING,
  STATE_READY,
  STATE_MOVE_FORWARD,
  STATE_ALIGN_COMBINED,        // Combined wall + light alignment (Phase 1)
  STATE_ADJUST_TO_28CM,        // Adjust distance to 28cm (step-by-step)
  STATE_DETECT_COLOR,
  STATE_MOVE_TO_TURN_POINT,
  STATE_ROTATING_90,
  STATE_MOVE_FORWARD_2,
  STATE_WALL_ALIGN_2,          // Wall parallel alignment only (Phase 2)
  STATE_SIDE_DISTANCE_ALIGN,   // Side distance alignment (Phase 2)
  STATE_ALIGN_DISTANCE_2,      // Front distance adjustment (Phase 2)
  STATE_FINAL_APPROACH,        // Final approach with step-by-step
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
#define LIGHT_ERROR_PERCENT       5.0   // Max 3% difference between sensors
#define LIGHT_MIN_AVG_INTENSITY   80    // Min average intensity 80% (prevents false alignment)

#define WALL_PARALLEL_TOL     1
#define WALL_APPROACH_DIST    33
#define TARGET_28CM           26        // Target distance 28cm for color detection position
#define COLOR_TURN_DIST       24
#define FINAL_STOP_DIST       8             // Final stop distance: 8cm
#define TARGET_WALL_DIST_PHASE1  31         // Target distance from wall after Phase 1 alignment
#define TARGET_WALL_DIST      20            // Target distance from wall after Phase 2 alignment
#define SIDE_TARGET_DIST      24            // Target distance from side wall (Phase 2)
#define MIN_SAFE_DIST         15            // Minimum safe distance (don't hit wall)
#define MAX_WALL_DIST         35            // Maximum distance to consider valid
#define ROTATE_TARGET_ANGLE   80.0

// Step-by-step approach distances
#define SLOW_APPROACH_DIST_PHASE1  40       // Start step-by-step approach at 40cm (Phase 1)
#define SLOW_APPROACH_DIST_PHASE2  15       // Start step-by-step approach at 15cm (Phase 2)

const unsigned long ALIGN_SAMPLE_MS = 300;
const int ALIGN_PULSE_MS = 80;
const int DISTANCE_ADJUST_PULSE_MS = 100;
const int FORWARD_STEP_MS = 100;
const int FORWARD_MEASURE_PAUSE_MS = 20;
const int STEP_APPROACH_MS = 60;            // Step duration for step-by-step approach

//------------------------
// While Loop Safety Limits
//------------------------
const int MAX_STEPS_PHASE1 = 150;
const int MAX_STEPS_PHASE2 = 100;
const int MAX_ALIGN_LOOP = 50;           // Max iterations for combined alignment
const int MAX_DISTANCE_ADJUST = 20;      // Max iterations for distance adjustment
const int MAX_SIDE_ALIGN_LOOP = 30;      // Max iterations for side distance alignment
const int MAX_STEP_APPROACH = 100;       // Max iterations for step-by-step approach
const int MAX_28CM_ADJUST = 30;          // Max iterations for 28cm adjustment

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
// Left wheels: A and C (based on BACKOFF in forward movement)
// Right wheels: B and D (based on FORWARD in forward movement)
void moveForward(int pwm) {
  int leftPwm = pwm + LeftWheel_Adjust;
  int rightPwm = pwm + RightWheel_Adjust;
  
  MOTORA_FORWARD(leftPwm);   // Left wheel A
  MOTORB_BACKOFF(rightPwm);  // Right wheel B
  MOTORC_FORWARD(leftPwm);   // Left wheel C
  MOTORD_BACKOFF(rightPwm);  // Right wheel D
}

void moveBackward(int pwm) {
  int leftPwm = pwm + LeftWheel_Adjust;
  int rightPwm = pwm + RightWheel_Adjust;
  
  MOTORA_BACKOFF(leftPwm);    // Left wheel A (reverse)
  MOTORB_FORWARD(rightPwm);   // Right wheel B (reverse)
  MOTORC_BACKOFF(leftPwm);    // Left wheel C (reverse)
  MOTORD_FORWARD(rightPwm);   // Right wheel D (reverse)
}

void moveLeft(int pwm) {
  MOTORA_BACKOFF(pwm);
  MOTORB_BACKOFF(pwm);
  MOTORC_FORWARD(pwm);
  MOTORD_FORWARD(pwm);
}

void moveRight(int pwm) {
  MOTORA_FORWARD(pwm);
  MOTORB_FORWARD(pwm);
  MOTORC_BACKOFF(pwm);
  MOTORD_BACKOFF(pwm);
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

void displayLight(int left, int right, float errorPercent, String action) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("L:"); display.print(left);
  display.print("% R:"); display.print(right); display.println("%");

  display.setCursor(0, 12);
  display.print("Err:"); display.print(errorPercent, 1); display.println("%");

  display.setCursor(0, 24);
  display.println(action);

  display.display();
}

void displayDistance(String title) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println(title);

  display.setCursor(0, 12);
  display.print("FL:"); display.print(distance_front_left);
  display.print(" FR:"); display.println(distance_front_right);

  display.setCursor(0, 24);
  display.print("Diff:");
  display.println(distance_front_left - distance_front_right);

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

void displayStepInfo(int stepNum, long fl, long fr, int ll, int lr, float errPercent) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("STEP:"); display.print(stepNum);
  display.print(" FL:"); display.print(fl);
  display.print(" FR:"); display.println(fr);

  display.setCursor(0, 12);
  display.print("L:"); display.print(ll);
  display.print(" R:"); display.print(lr);
  display.print(" E:"); display.print(errPercent, 1); display.println("%");

  display.setCursor(0, 24);
  display.print("Dist:"); display.print((fl + fr) / 2); display.println("cm");

  display.display();
}

void displayCombinedAlign(int left, int right, float lightErr, int avgIntensity,
                          long fl, long fr, int wallErr,
                          bool wallOK, bool lightOK, bool intensityOK, String action) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("W:"); display.print(wallOK ? "OK" : "NO");
  display.print(" L:"); display.print(lightOK ? "OK" : "NO");
  display.print(" I:"); display.println(intensityOK ? "OK" : "LOW");

  display.setCursor(0, 12);
  display.print("Err:"); display.print(lightErr, 1); display.print("%");
  display.print(" Avg:"); display.print(avgIntensity); display.println("%");

  display.setCursor(0, 24);
  display.print("FL:"); display.print(fl);
  display.print(" FR:"); display.print(fr);
  display.print(" "); display.println(action);

  display.display();
}

void displayWallAlign2(long fl, long fr, int wallErr, bool wallOK, String action) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("PHASE2 WALL");
  display.print(" "); display.println(wallOK ? "OK" : "ALIGN");

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

void displayDistanceAdjust(long avgDist, int targetDist, String action) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("DIST ADJUST");

  display.setCursor(0, 12);
  display.print("Current:"); display.print(avgDist); display.println("cm");
  display.print(" Target:"); display.print(targetDist); display.println("cm");

  display.setCursor(0, 24);
  display.println(action);

  display.display();
}

void displayStepApproach(long currentDist, int targetDist, int startDist, String phase) {
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
  display.print("Start:"); display.print(startDist); display.println("cm");

  display.display();
}

// Display for 28cm adjustment
void display28cmAdjust(long currentDist, int targetDist, String action) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("ADJUST TO 28CM");

  display.setCursor(0, 12);
  display.print("Current:"); display.print(currentDist); display.println("cm");
  display.print(" Target:"); display.print(targetDist); display.println("cm");

  display.setCursor(0, 24);
  display.println(action);

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

void sampleLightAverage(unsigned long sampleMs = ALIGN_SAMPLE_MS) {
  stopAll();

  unsigned long start = millis();
  long sumL = 0, sumR = 0;
  int count = 0;

  while (millis() - start < sampleMs) {
    readLightSensors();
    calculateIntensity();

    sumL += lightLeftIntensity;
    sumR += lightRightIntensity;
    count++;

    delay(5);
  }

  avgLightLeft = (count > 0) ? (sumL / count) : 0;
  avgLightRight = (count > 0) ? (sumR / count) : 0;

  lightLeftIntensity = avgLightLeft;
  lightRightIntensity = avgLightRight;
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

// 检查光强是否足够高（确保检测到真正的光源）
bool isLightIntensityHighEnough() {
  int avgIntensity = getAverageLightIntensity();
  return (avgIntensity >= LIGHT_MIN_AVG_INTENSITY);
}

// 检查光线是否对齐（误差小 + 强度足够）
bool isLightAlignedComplete() {
  float errorPercent = calculateLightErrorPercent();
  bool intensityOK = isLightIntensityHighEnough();
  return (intensityOK && errorPercent <= LIGHT_ERROR_PERCENT);
}

bool isWallDetected(long fl, long fr) {
  return ((fl != 999 && fl <= WALL_APPROACH_DIST) ||
          (fr != 999 && fr <= WALL_APPROACH_DIST));
}

bool isWallParallel() {
  return (abs(distance_front_left - distance_front_right) <= WALL_PARALLEL_TOL);
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

char detectColor() {
  readColorSensor();

  Serial.print("Color - R:"); Serial.print(redColor);
  Serial.print(" G:"); Serial.println(greenColor);

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
// COMBINED ALIGNMENT FUNCTIONS (Wall + Light in single loop) - Phase 1 Only
//============================================================

int alignLoopCount = 0;
int strafeSearchDirection = 1;  // 1 = right, -1 = left
int strafeSearchCount = 0;
const int MAX_STRAFE_SEARCH = 10;  // 最大搜索次数

void resetCombinedAlign() {
  alignLoopCount = 0;
  strafeSearchDirection = 1;
  strafeSearchCount = 0;
  resetWallPID();
}

//------------------------------------------------------------
// Combined Alignment Logic (Phase 1 with Light Sensor)
//------------------------------------------------------------
void doCombinedAlignment(RobotState nextState, const char* title) {
  
  alignLoopCount++;
  
  if (alignLoopCount > MAX_ALIGN_LOOP) {
    stopAll();
    displayState("ALIGN TIMEOUT", "Max loops reached", "Proceeding...");
    Serial.println(">>> Combined alignment timeout!");
    delay(300);
    resetCombinedAlign();
    currentState = nextState;
    return;
  }

  // Measure current state
  sampleFrontAverage(ALIGN_SAMPLE_MS);
  sampleLightAverage();

  int wallErr = distance_front_left - distance_front_right;
  float lightErr = calculateLightErrorPercent();
  int avgIntensity = getAverageLightIntensity();
  
  bool wallOK = abs(wallErr) <= WALL_PARALLEL_TOL;
  bool intensityOK = isLightIntensityHighEnough();  // avg >= 80%
  bool lightErrOK = lightErr <= LIGHT_ERROR_PERCENT;  // err <= 3%
  bool lightOK = intensityOK && lightErrOK;  // both conditions

  // Display current status
  displayCombinedAlign(lightLeftIntensity, lightRightIntensity, lightErr, avgIntensity,
                       distance_front_left, distance_front_right, wallErr,
                       wallOK, lightErrOK, intensityOK, "");

  Serial.print("Align Loop "); Serial.print(alignLoopCount);
  Serial.print(" | WallErr:"); Serial.print(wallErr);
  Serial.print(" LightErr:"); Serial.print(lightErr, 1); Serial.println("%");
  Serial.print(" | AvgIntensity:"); Serial.print(avgIntensity); Serial.print("%");
  Serial.print(" IntensityOK:"); Serial.print(intensityOK ? "YES" : "NO");
  Serial.print(" LightErrOK:"); Serial.print(lightErrOK ? "YES" : "NO");
  Serial.print(" WallOK:"); Serial.println(wallOK ? "YES" : "NO");

  // Check if ALL conditions are met (wall + light error + intensity)
  if (wallOK && lightOK) {
    stopAll();
    displayState("BOTH OK!", "Wall+Light+Inten", "");
    Serial.println(">>> All conditions met! Wall parallel, light aligned, intensity sufficient!");
    delay(300);
    resetCombinedAlign();
    currentState = nextState;
    return;
  }

  // Step 1: If wall not parallel, fix wall first
  if (!wallOK) {
    int pulseMs = computeWallPIDPulse(wallErr);
    
    Serial.print(">>> Adjusting wall, pulse: "); Serial.println(pulseMs);
    displayCombinedAlign(lightLeftIntensity, lightRightIntensity, lightErr, avgIntensity,
                         distance_front_left, distance_front_right, wallErr,
                         wallOK, lightErrOK, intensityOK, "ROTATE");

    if (wallErr > 0) {
      rotateRight(Motor_PWM_Rotate);
    } else {
      rotateLeft(Motor_PWM_Rotate);
    }
    
    delay(pulseMs);
    stopAll();
    delay(50);
    return;
  }

  // Step 2: Wall is parallel, check light error
  if (!lightErrOK) {
    Serial.println(">>> Wall OK, adjusting light error (strafe toward stronger side)");
    displayCombinedAlign(lightLeftIntensity, lightRightIntensity, lightErr, avgIntensity,
                         distance_front_left, distance_front_right, wallErr,
                         wallOK, lightErrOK, intensityOK, "ALIGN");

    if (lightLeftIntensity > lightRightIntensity) {
      moveLeft(Motor_PWM_Strafe);
    } else {
      moveRight(Motor_PWM_Strafe);
    }

    delay(ALIGN_PULSE_MS);
    stopAll();
    delay(50);
    
    resetWallPID();
    return;
  }

  // Step 3: Light error OK but intensity too low
  if (!intensityOK) {
    strafeSearchCount++;
    
    if (strafeSearchCount > MAX_STRAFE_SEARCH) {
      stopAll();
      displayState("LIGHT SEARCH", "No strong light", "Proceeding...");
      Serial.println(">>> Light search timeout, proceeding anyway");
      delay(500);
      resetCombinedAlign();
      currentState = nextState;
      return;
    }

    Serial.print(">>> Intensity too low ("); Serial.print(avgIntensity);
    Serial.print("%), searching for light source... Direction: ");
    Serial.println(strafeSearchDirection > 0 ? "RIGHT" : "LEFT");
    
    displayCombinedAlign(lightLeftIntensity, lightRightIntensity, lightErr, avgIntensity,
                         distance_front_left, distance_front_right, wallErr,
                         wallOK, lightErrOK, intensityOK, "SEARCH");

    if (strafeSearchDirection > 0) {
      moveRight(Motor_PWM_Strafe);
    } else {
      moveLeft(Motor_PWM_Strafe);
    }

    delay(ALIGN_PULSE_MS * 2);
    stopAll();
    delay(50);

    strafeSearchDirection *= -1;
    resetWallPID();
    return;
  }
}

//============================================================
// ADJUST TO 28CM FUNCTION (Step-by-step Forward/Backward)
//============================================================
int adjust28cmCount = 0;

void resetAdjust28cm() {
  adjust28cmCount = 0;
}

void doAdjustTo28cm(RobotState nextState) {
  
  adjust28cmCount++;
  
  if (adjust28cmCount > MAX_28CM_ADJUST) {
    stopAll();
    displayState("28CM TIMEOUT", "Max adjusts reached", "Proceeding...");
    Serial.println(">>> 28cm adjustment timeout!");
    delay(300);
    resetAdjust28cm();
    currentState = nextState;
    return;
  }

  // Measure current distance
  sampleFrontAverage(ALIGN_SAMPLE_MS);
  long avgDist = getAverageFrontDistance();

  if (avgDist == 999) {
    stopAll();
    display28cmAdjust(999, TARGET_28CM, "NO READING");
    delay(100);
    return;
  }

  int distErr = avgDist - TARGET_28CM;
  
  Serial.print("28cm Adjust | Step:"); Serial.print(adjust28cmCount);
  Serial.print(" Current:"); Serial.print(avgDist);
  Serial.print(" Target:"); Serial.print(TARGET_28CM);
  Serial.print(" Error:"); Serial.println(distErr);

  // Check if at target distance (±1cm tolerance)
  if (abs(distErr) <= 1) {
    stopAll();
    display28cmAdjust(avgDist, TARGET_28CM, "OK!");
    Serial.println(">>> Distance adjusted to 28cm!");
    delay(300);
    resetAdjust28cm();
    currentState = nextState;
    return;
  }

  // Step-by-step adjustment
  if (distErr > 0) {
    // Too far, need to move forward (step-by-step)
    display28cmAdjust(avgDist, TARGET_28CM, "FWD STEP");
    Serial.println(">>> Step forward to reach 28cm");
    moveForward(Motor_PWM_Forward);
    delay(STEP_APPROACH_MS);
    stopAll();
  } else {
    // Too close, need to move backward (step-by-step)
    display28cmAdjust(avgDist, TARGET_28CM, "BACK STEP");
    Serial.println(">>> Step backward to reach 28cm");
    moveBackward(Motor_PWM_Forward);
    delay(STEP_APPROACH_MS);
    stopAll();
  }

  delay(FORWARD_MEASURE_PAUSE_MS);
}

//============================================================
// WALL ONLY ALIGNMENT (Phase 2 - No Light Sensor)
//============================================================
int wallAlignLoopCount = 0;

void resetWallAlign() {
  wallAlignLoopCount = 0;
  resetWallPID();
}

void doWallAlignmentOnly(RobotState nextState, const char* title) {
  
  wallAlignLoopCount++;
  
  if (wallAlignLoopCount > MAX_ALIGN_LOOP) {
    stopAll();
    displayState("WALL TIMEOUT", "Max loops reached", "Proceeding...");
    Serial.println(">>> Wall alignment timeout!");
    delay(300);
    resetWallAlign();
    currentState = nextState;
    return;
  }

  sampleFrontAverage(ALIGN_SAMPLE_MS);

  int wallErr = distance_front_left - distance_front_right;
  bool wallOK = abs(wallErr) <= WALL_PARALLEL_TOL;

  displayWallAlign2(distance_front_left, distance_front_right, wallErr, wallOK, "");

  Serial.print("Wall Align Loop "); Serial.print(wallAlignLoopCount);
  Serial.print(" | FL:"); Serial.print(distance_front_left);
  Serial.print(" FR:"); Serial.print(distance_front_right);
  Serial.print(" Err:"); Serial.print(wallErr);
  Serial.print(" OK:"); Serial.println(wallOK ? "YES" : "NO");

  if (wallOK) {
    stopAll();
    displayWallAlign2(distance_front_left, distance_front_right, wallErr, wallOK, "OK!");
    Serial.println(">>> Wall parallel!");
    delay(300);
    resetWallAlign();
    currentState = nextState;
    return;
  }

  int pulseMs = computeWallPIDPulse(wallErr);
  
  Serial.print(">>> Adjusting wall, pulse: "); Serial.println(pulseMs);
  displayWallAlign2(distance_front_left, distance_front_right, wallErr, wallOK, "ROTATE");

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
    displayState("SIDE TIMEOUT", "Max loops reached", "Proceeding...");
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

//------------------------------------------------------------
// Step-by-step approach for Phase 2 (Final Approach)
// Start at SLOW_APPROACH_DIST_PHASE2 (15cm), stop at FINAL_STOP_DIST (8cm)
//------------------------------------------------------------
void doStepByStepApproachPhase2(RobotState nextState) {
  
  static int stepCount = 0;
  
  if (stepCount == 0) {
    stepCount = 1;
    Serial.println("=== Phase 2 Step-by-Step Approach (15cm -> 8cm) ===");
  }
  
  if (stepCount > MAX_STEP_APPROACH) {
    stopAll();
    displayState("APPROACH TIMEOUT", "Max steps reached", "COMPLETE!");
    Serial.println(">>> Final approach timeout!");
    delay(300);
    stepCount = 0;
    currentState = nextState;
    return;
  }

  // Measure current distance
  updateDistances();
  long avgDist = getAverageFrontDistance();

  if (avgDist == 999) {
    stopAll();
    displayStepApproach(999, FINAL_STOP_DIST, SLOW_APPROACH_DIST_PHASE2, "P2");
    delay(100);
    return;
  }

  Serial.print("Step Approach P2 | Step:"); Serial.print(stepCount);
  Serial.print(" Dist:"); Serial.print(avgDist);
  Serial.print(" Target:"); Serial.println(FINAL_STOP_DIST);

  displayStepApproach(avgDist, FINAL_STOP_DIST, SLOW_APPROACH_DIST_PHASE2, "P2");

  // Check if reached target (±1cm tolerance)
  if (avgDist <= FINAL_STOP_DIST) {
    stopAll();
    displayStepApproach(avgDist, FINAL_STOP_DIST, SLOW_APPROACH_DIST_PHASE2, "OK!");
    Serial.println(">>> Phase 2 target reached!");
    delay(300);
    stepCount = 0;
    currentState = nextState;
    return;
  }

  // Only move if distance > target
  if (avgDist > FINAL_STOP_DIST) {
    // Move forward a small step
    moveForward(Motor_PWM_Forward);
    delay(STEP_APPROACH_MS);
    stopAll();
    delay(FORWARD_MEASURE_PAUSE_MS);
  }

  stepCount++;
}

//============================================================
// DISTANCE ADJUSTMENT FUNCTION (for Phase 2 front distance 20cm)
//============================================================
void doDistanceAdjustment(RobotState nextState, const char* title, int targetDist) {
  
  static int distanceAdjustCount = 0;
  
  if (distanceAdjustCount == 0) {
    distanceAdjustCount = 1;
  }
  
  if (distanceAdjustCount > MAX_DISTANCE_ADJUST) {
    stopAll();
    displayState("DIST TIMEOUT", "Max adjusts reached", "Proceeding...");
    Serial.println(">>> Distance adjustment timeout!");
    delay(300);
    distanceAdjustCount = 0;
    currentState = nextState;
    return;
  }

  sampleFrontAverage(ALIGN_SAMPLE_MS);
  long avgDist = getAverageFrontDistance();

  if (avgDist == 999) {
    stopAll();
    displayState("DIST ERROR", "No valid reading", "");
    delay(100);
    return;
  }

  Serial.print("Dist Adjust | Current:"); Serial.print(avgDist);
  Serial.print(" Target:"); Serial.println(targetDist);

  // Check if at target distance (±1cm tolerance)
  if (abs(avgDist - targetDist) <= 1) {
    stopAll();
    displayDistanceAdjust(avgDist, targetDist, "OK!");
    Serial.println(">>> Distance adjusted!");
    delay(300);
    distanceAdjustCount = 0;
    currentState = nextState;
    return;
  }

  // Adjust distance
  if (avgDist > targetDist) {
    // Too far, move forward
    if (avgDist <= MIN_SAFE_DIST) {
      stopAll();
      displayDistanceAdjust(avgDist, targetDist, "TOO CLOSE!");
      Serial.println(">>> Already at safe minimum!");
      delay(300);
      distanceAdjustCount = 0;
      currentState = nextState;
      return;
    }
    
    displayDistanceAdjust(avgDist, targetDist, "FORWARD");
    moveForward(Motor_PWM_Forward);
    delay(DISTANCE_ADJUST_PULSE_MS);
    stopAll();
  } else {
    // Too close, move backward
    displayDistanceAdjust(avgDist, targetDist, "BACKWARD");
    moveBackward(Motor_PWM_Forward);
    delay(DISTANCE_ADJUST_PULSE_MS);
    stopAll();
  }

  distanceAdjustCount++;
  delay(50);
}

//============================================================
// AUTONOMOUS ROBOT LOGIC
//============================================================
void autonomousControl() {
  if (currentState == STATE_FINAL_APPROACH || currentState == STATE_MOVE_TO_TURN_POINT ||
      currentState == STATE_ADJUST_TO_28CM || currentState == STATE_ALIGN_DISTANCE_2 ||
      currentState == STATE_SIDE_DISTANCE_ALIGN) {
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

    case STATE_CALIBRATING:
      break;

    case STATE_READY:
      stopAll();
      if (stateChanged) {
        displayState("READY!", "Press switch", "to START");
      }
      break;

    //============================================================
    // 第一段前进
    //============================================================
    case STATE_MOVE_FORWARD:
      {
        if (stateChanged) {
          displayState("MOVE FWD 1", "While loop", "Dist only");
          Serial.println("=== Phase 1: Move Forward ===");
        }

        bool keepMoving = true;
        int stepCount = 0;
        String stopReason = "";

        while (keepMoving && stepCount < MAX_STEPS_PHASE1) {
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

          readLightSensors();
          calculateIntensity();

          float errorPercent = calculateLightErrorPercent();
          stepCount++;

          displayStepInfo(stepCount, fl, fr, lightLeftIntensity, lightRightIntensity, errorPercent);

          Serial.print("Step "); Serial.print(stepCount);
          Serial.print(" | FL:"); Serial.print(fl);
          Serial.print(" FR:"); Serial.print(fr);
          Serial.print(" | L:"); Serial.print(lightLeftIntensity);
          Serial.print(" R:"); Serial.print(lightRightIntensity);
          Serial.print(" | Err:"); Serial.print(errorPercent, 1); Serial.println("%");

          // 停止条件1: 检测到墙壁
          if (keepMoving && isWallDetected(fl, fr)) {
            keepMoving = false;
            stopReason = "WALL";
            Serial.println(">>> Wall detected!");
            displayState("WALL DETECT", "Dist FL:" + String(fl), "FR:" + String(fr));
            delay(300);
          }

          // 停止条件2: 太靠近墙壁
          if (keepMoving) {
            long avgDist = getAverageFrontDistance();
            if (avgDist != 999 && avgDist <= WALL_APPROACH_DIST) {
              keepMoving = false;
              stopReason = "CLOSE";
              Serial.println(">>> Too close to wall!");
            }
          }
        }

        stopAll();
        Serial.print("Phase 1 complete: "); Serial.print(stepCount);
        Serial.print(" steps, reason: "); Serial.println(stopReason);

        currentState = STATE_ALIGN_COMBINED;
      }
      break;

    case STATE_ALIGN_COMBINED:
      if (stateChanged) {
        resetCombinedAlign();
        displayState("ALIGN PHASE1", "Wall+Light loop", "Need Int>=80%");
        Serial.println("=== Combined Alignment Phase 1 ===");
      }
      // After alignment, go to 28cm adjustment
      doCombinedAlignment(STATE_ADJUST_TO_28CM, "ALIGN 1");
      break;

    //============================================================
    // Adjust to 28cm (Step-by-step Forward/Backward)
    //============================================================
    case STATE_ADJUST_TO_28CM:
      if (stateChanged) {
        resetAdjust28cm();
        displayState("ADJUST 28CM", "Step FWD/BACK", "Target: 28cm");
        Serial.println("=== Adjust Distance to 28cm (Step-by-step) ===");
      }
      // After 28cm adjustment, go directly to color detection
      doAdjustTo28cm(STATE_DETECT_COLOR);
      break;

    case STATE_DETECT_COLOR:
      {
        if (stateChanged) {
          displayState("DETECT COLOR", "Reading...", "");
        }

        detectedColor = detectColor();

        if (detectedColor != 'N') {
          String colorStr = (detectedColor == 'R') ? "RED->Right" : "GREEN->Left";
          displayState("COLOR:", colorStr, "Go to 24cm");
          delay(1000);
          currentState = STATE_MOVE_TO_TURN_POINT;
        } else {
          displayState("NO COLOR", "Waiting...", "");
          delay(200);
        }
      }
      break;

    case STATE_MOVE_TO_TURN_POINT:
      {
        if (stateChanged) {
          displayState("AFTER COLOR", "Forward to 24cm", "");
          Serial.println("=== Move to turn point ===");
        }

        updateDistances();
        long avgDist = getAverageFrontDistance();

        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("MOVE TO 24CM");
        display.setCursor(0, 12);
        display.print("FL:"); display.print(distance_front_left);
        display.print(" FR:"); display.println(distance_front_right);
        display.setCursor(0, 24);
        display.print("AVG:"); display.print(avgDist); display.println("cm");
        display.display();

        if (distance_front_left == 999 && distance_front_right == 999) {
          stopAll();
          delay(50);
          break;
        }

        if (avgDist != 999 && avgDist <= COLOR_TURN_DIST) {
          stopAll();
          displayState("TURN POINT", "Reached 24cm", "Rotate now");
          Serial.println(">>> Reached 24cm, start turning");
          delay(300);
          currentState = STATE_ROTATING_90;
        } else {
          moveForward(Motor_PWM_Forward);
        }

        delay(50);
      }
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
    // 第二段前进 (Phase 2 - No Light Sensor)
    //============================================================
    case STATE_MOVE_FORWARD_2:
      {
        if (stateChanged) {
          displayState("MOVE FWD 2", "While loop", "Dist only");
          Serial.println("=== Phase 2: Move Forward ===");
        }

        bool keepMoving = true;
        int stepCount2 = 0;
        String stopReason = "";

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

          readLightSensors();
          calculateIntensity();

          float errorPercent = calculateLightErrorPercent();
          stepCount2++;

          displayStepInfo(stepCount2, fl, fr, lightLeftIntensity, lightRightIntensity, errorPercent);

          Serial.print("Step2 "); Serial.print(stepCount2);
          Serial.print(" | FL:"); Serial.print(fl);
          Serial.print(" FR:"); Serial.print(fr);
          Serial.print(" | L:"); Serial.print(lightLeftIntensity);
          Serial.print(" R:"); Serial.print(lightRightIntensity);
          Serial.print(" | Err:"); Serial.print(errorPercent, 1); Serial.println("%");

          // 停止条件1: 检测到墙壁
          if (keepMoving && isWallDetected(fl, fr)) {
            keepMoving = false;
            stopReason = "WALL";
            Serial.println(">>> Wall detected in phase 2!");
          }

          // 停止条件2: 到达最终距离
          if (keepMoving) {
            long avgDist = getAverageFrontDistance();
            if (avgDist != 999 && avgDist <= SLOW_APPROACH_DIST_PHASE2) {
              keepMoving = false;
              stopReason = "SLOW";
              Serial.println(">>> Reached slow approach zone!");
            }
          }
        }

        stopAll();
        Serial.print("Phase 2 complete: "); Serial.print(stepCount2);
        Serial.print(" steps, reason: "); Serial.println(stopReason);

        if (stopReason == "SLOW") {
          currentState = STATE_WALL_ALIGN_2;
        } else {
          currentState = STATE_WALL_ALIGN_2;
        }
      }
      break;

    case STATE_WALL_ALIGN_2:
      if (stateChanged) {
        resetWallAlign();
        displayState("WALL ALIGN 2", "Parallel only", "No light sensor");
        Serial.println("=== Wall Alignment Phase 2 (No Light Sensor) ===");
      }
      doWallAlignmentOnly(STATE_SIDE_DISTANCE_ALIGN, "WALL 2");
      break;

    case STATE_SIDE_DISTANCE_ALIGN:
      if (stateChanged) {
        resetSideAlign();
        String turnDir = (detectedColor == 'G') ? "L->RIGHT sens" : "R->LEFT sens";
        displayState("SIDE ALIGN", turnDir, "Target: 24cm");
        Serial.print("=== Side Distance Alignment (Turn: ");
        Serial.print(detectedColor == 'G' ? "LEFT, RIGHT sensor" : "RIGHT, LEFT sensor");
        Serial.println(") ===");
      }
      doSideDistanceAlignment(STATE_ALIGN_DISTANCE_2);
      break;

    case STATE_ALIGN_DISTANCE_2:
      if (stateChanged) {
        displayState("DIST ADJUST 2", "Target: 20cm", "");
        Serial.println("=== Distance Adjustment Phase 2 (Target: 20cm) ===");
      }
      doDistanceAdjustment(STATE_FINAL_APPROACH, "DIST 2", TARGET_WALL_DIST);  // 20cm
      break;

    case STATE_FINAL_APPROACH:
      if (stateChanged) {
        displayState("FINAL APPROACH", "15cm -> 8cm", "Step by step");
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
  Serial.println("Light Tracking Robot v2.16");
  Serial.println("Simplified: Direct 28cm adjust");
  Serial.println("================================");

  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 failed"));
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("v2.16");
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

  digitalWrite(S0, LOW);
  digitalWrite(S1, HIGH);

  pinMode(LIGHT_LEFT, INPUT);
  pinMode(LIGHT_RIGHT, INPUT);

  pinMode(START_SWITCH_PIN, INPUT_PULLUP);

  displayState("IDLE", "Press switch", "to calibrate");
  Serial.println("Setup complete!");
  Serial.print("Motor PWM - Forward:"); Serial.print(Motor_PWM_Forward);
  Serial.print(" Rotate:"); Serial.print(Motor_PWM_Rotate);
  Serial.print(" Strafe:"); Serial.println(Motor_PWM_Strafe);
  Serial.print("Wheel Adjust - Left:"); Serial.print(LeftWheel_Adjust);
  Serial.print(" Right:+"); Serial.println(RightWheel_Adjust);
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

      if (currentState == STATE_IDLE) {
        currentState = STATE_CALIBRATING;
        calibrateStep = 0;
        displayState("CALIBRATION", "Press for base", "");
      }
      else if (currentState == STATE_CALIBRATING) {
        if (calibrateStep == 0) {
          calibrateLightAmbient();
          calibrateStep = 1;
          displayState("CAL STEP 2", "Press for dark", "");
        } else if (calibrateStep == 1) {
          calibrateLightDark();
          currentState = STATE_READY;
        }
      }
      else if (currentState == STATE_READY) {
        currentState = STATE_MOVE_FORWARD;
      }
    }
  }

  if (digitalRead(START_SWITCH_PIN) == HIGH) {
    switchPressed = false;
  }

  if (currentState == STATE_CALIBRATING) {
    // wait for button presses
  } else {
    autonomousControl();
  }

  delay(10);
}
