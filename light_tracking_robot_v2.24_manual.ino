/*****************************************************
Light Tracking Robot with Color Detection + PID Wall Align
Version: 2.24 (Manual Light Values - No Calibration)
Date: 2026-03-27

Changes from v2.23:
- Removed light sensor calibration
- Manual input for light sensor base/min values
- Direct start from IDLE -> READY -> MOVE_FORWARD

Phase 1 strafe logic:
- First 10 loops: Light sensor guides strafe direction (fixed pulse)
- After 10 loops: Side distance difference guides direction (fixed pulse)
  * side_left > side_right → strafe RIGHT
  * side_left < side_right → strafe LEFT
- NO PID control for strafe, just simple step movement

Phase 1 exit conditions:
1. Parallel to wall (front left/right diff <= 1)
2. Front distance = 26cm ±2
3. Side left/right distance equal within tolerance 3

Phase 2 flow:
1. Initial wall parallel alignment
2. Loop until front distance <= 15cm:
   - Step forward
   - Wall parallel alignment (PID)
   - Side distance alignment to 24cm (simple step strafe)
3. Final approach: step-by-step to 8cm

Flow:
1. Move forward (while loop with conditions)
2. Stop when wall detected
3. Combined alignment loop (Phase 1)
4. Detect color
5. Move forward until front average distance = 24cm
6. Rotate 90° (gyro)
7. Move forward (while loop)
8. Phase 2 alignment:
   - Initial wall parallel alignment
   - Loop: Step → Wall → Side (until dist <= 15cm)
   - Final approach to 8cm
9. Complete
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
#define S0 40
#define S1 28
#define S2 41
#define S3 47
#define sensorOut 48

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
// MANUAL LIGHT SENSOR VALUES - EDIT THESE!
//============================================================
// Base values (ambient light - sensor reading in bright/open area)
// Higher value = more light detected
int lightLeftBase = 440;   // Left sensor ambient base value
int lightRightBase = 465;  // Right sensor ambient base value

// Min values (dark - sensor reading when covered)
// Lower value = less light detected
int lightLeftMin = 410;    // Left sensor dark/minimum value
int lightRightMin = 440;    // Right sensor dark/minimum value

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
long redFrequency = 0;
long greenFrequency = 0;

//------------------------
// Light Sensor Variables
//------------------------
int lightLeftRaw = 0;
int lightRightRaw = 0;

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
// Motor Speed
//------------------------
int Motor_PWM_Forward = 60;
int Motor_PWM_Rotate = 70;
int Motor_PWM_Strafe = 100;

//------------------------
// Motor Calibration
//------------------------
int LeftWheel_Adjust = 8;
int RightWheel_Adjust = 0;

//============================================================
// ROBOT STATE MACHINE
//============================================================
enum RobotState {
  STATE_IDLE,
  STATE_READY,
  STATE_MOVE_FORWARD,
  STATE_ALIGN_COMBINED,
  STATE_DETECT_COLOR,
  STATE_MOVE_TO_TURN_POINT,
  STATE_ROTATING_90,
  STATE_MOVE_FORWARD_2,
  STATE_WALL_ALIGN_2,
  STATE_PHASE2_LOOP,
  STATE_FINAL_APPROACH,
  STATE_COMPLETE
};

RobotState currentState = STATE_IDLE;
RobotState previousState = STATE_IDLE;
char detectedColor = 'N';

//------------------------
// Tolerances & Distances
//------------------------
#define LIGHT_EQUAL_TOL           15
#define WALL_PARALLEL_TOL         1
#define FRONT_DIST_TOL            2
#define SIDE_EQUAL_TOL            3

#define WALL_APPROACH_DIST        33
#define TARGET_28CM               26
#define COLOR_TURN_DIST           24
#define FINAL_STOP_DIST           6
#define TARGET_WALL_DIST_PHASE1   31
#define TARGET_WALL_DIST          20
#define SIDE_TARGET_DIST          22
#define MIN_SAFE_DIST             15
#define MAX_WALL_DIST             35
#define ROTATE_TARGET_ANGLE       55.0

#define SLOW_APPROACH_DIST_PHASE1 40
#define SLOW_APPROACH_DIST_PHASE2 15

const unsigned long ALIGN_SAMPLE_MS = 300;
const int ALIGN_PULSE_MS = 80;
const int DISTANCE_ADJUST_PULSE_MS = 100;
const int FORWARD_STEP_MS = 100;
const int FORWARD_MEASURE_PAUSE_MS = 20;
const int STEP_APPROACH_MS = 60;
const int SIDE_STRAFE_PULSE_MS = 80;  // Fixed pulse for side distance strafe

//------------------------
// Safety Limits
//------------------------
const int MAX_STEPS_PHASE1 = 300;
const int MAX_STEPS_PHASE2 = 200;
const int MAX_ALIGN_LOOP = 100;
const int MAX_DISTANCE_ADJUST = 40;
const int MAX_SIDE_ALIGN_LOOP = 60;
const int MAX_STEP_APPROACH = 200;
const int MAX_28CM_ADJUST = 60;
const int MAX_PHASE2_LOOP = 200;

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

//------------------------
// PID for Strafe Alignment (Phase 1 only)
//------------------------
float strafeError = 0;
float strafePrevError = 0;
float strafeIntegral = 0;
unsigned long strafePrevTime = 0;

float strafeKp = 25.0;
float strafeKi = 0.5;
float strafeKd = 8.0;

int strafePidMinPulse = 40;
int strafePidMaxPulse = 200;
float strafeIntegralLimit = 50.0;

const int LIGHT_GUIDE_LOOPS = 3;

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
// MOTOR MOVEMENT FUNCTIONS
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

void displayColorFreq(long redFreq, long greenFreq, long diff, char result) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("COLOR DETECT v2.24");

  display.setCursor(0, 12);
  display.print("R:"); display.print(redFreq);
  display.print(" G:"); display.println(greenFreq);

  display.setCursor(0, 24);
  display.print("Diff:"); display.print(diff);
  display.print(" ");
  if (result == 'R') display.println("RED");
  else if (result == 'G') display.println("GREEN");
  else display.println("?");

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

void displayCombinedAlignStatus(long fl, long fr, long avgDist, int wallErr,
                                long sideL, long sideR, int sideDiff, 
                                int strafePulse, String guideMode, String action) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("FL:"); display.print(fl);
  display.print(" FR:"); display.println(fr);

  display.setCursor(0, 12);
  display.print("SL:"); display.print(sideL);
  display.print(" SR:"); display.print(sideR);
  display.print(" D:"); display.println(sideDiff);

  display.setCursor(0, 24);
  display.print(guideMode);
  display.print(" P:"); display.print(strafePulse);
  display.print(" "); display.println(action);

  display.display();
}

void displayPhase2Loop(int loopCount, long frontDist, long sideDist, String step) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("P2 LOOP "); display.print(loopCount);
  display.print(" "); display.println(step);

  display.setCursor(0, 12);
  display.print("Front:"); display.print(frontDist);
  display.print("cm Side:"); display.println(sideDist);

  display.setCursor(0, 24);
  display.print("Target:"); display.print(SIDE_TARGET_DIST);
  display.print("cm End:"); display.print(SLOW_APPROACH_DIST_PHASE2); display.println("cm");

  display.display();
}

void displayPhase2Wall(long fl, long fr, int wallErr, bool wallOK) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("P2 WALL ");
  display.println(wallOK ? "OK" : "ADJ");

  display.setCursor(0, 12);
  display.print("FL:"); display.print(fl);
  display.print(" FR:"); display.print(fr);
  display.print(" Err:"); display.println(wallErr);

  display.setCursor(0, 24);
  display.println(wallOK ? "Continue..." : "Rotating...");

  display.display();
}

void displayPhase2Side(long sideDist, int targetDist, String sensorName, bool sideOK) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("P2 SIDE ");
  display.print(sensorName);
  display.println(sideOK ? " OK" : " ADJ");

  display.setCursor(0, 12);
  display.print("Dist:"); display.print(sideDist);
  display.print("cm Tgt:"); display.println(targetDist);

  display.setCursor(0, 24);
  display.println(sideOK ? "Continue..." : "Strafing...");

  display.display();
}

void displayFinalApproach(long currentDist, int targetDist, int stepCount) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("FINAL APPROACH #"); display.println(stepCount);

  display.setCursor(0, 12);
  display.print("Now:"); display.print(currentDist);
  display.print("cm Tgt:"); display.print(targetDist); display.println("cm");

  display.setCursor(0, 24);
  display.println(currentDist <= targetDist ? "DONE!" : "Moving...");

  display.display();
}

void displayManualValues() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("LB:"); display.print(lightLeftBase);
  display.print(" RB:"); display.println(lightRightBase);

  display.setCursor(0, 12);
  display.print("LMin:"); display.print(lightLeftMin);
  display.print(" RMin:"); display.println(lightRightMin);

  display.setCursor(0, 24);
  display.println("MANUAL MODE");

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

void sampleSideDistances(unsigned long sampleMs = ALIGN_SAMPLE_MS) {
  stopAll();

  unsigned long start = millis();
  long sumSL = 0, sumSR = 0;
  int countSL = 0, countSR = 0;

  while (millis() - start < sampleMs) {
    long sl = measureDistance(TRIG_SIDE_LEFT, ECHO_SIDE_LEFT);
    delay(5);
    long sr = measureDistance(TRIG_SIDE_RIGHT, ECHO_SIDE_RIGHT);
    delay(5);

    if (sl != 999) {
      sumSL += sl;
      countSL++;
    }

    if (sr != 999) {
      sumSR += sr;
      countSR++;
    }
  }

  distance_side_left = (countSL > 0) ? (sumSL / countSL) : 999;
  distance_side_right = (countSR > 0) ? (sumSR / countSR) : 999;
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

//============================================================
// LIGHT/WALL CHECK FUNCTIONS
//============================================================
float calculateLightErrorPercent() {
  int diff = abs(lightLeftIntensity - lightRightIntensity);
  int maxIntensity = max(lightLeftIntensity, lightRightIntensity);

  if (maxIntensity == 0) return 100.0;
  return (float)diff * 100.0 / (float)maxIntensity;
}

bool isWallDetected(long fl, long fr) {
  return ((fl != 999 && fl <= WALL_APPROACH_DIST) ||
          (fr != 999 && fr <= WALL_APPROACH_DIST));
}

//============================================================
// COLOR SENSOR FUNCTIONS
//============================================================
void readColorSensor() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  delay(10);
  redFrequency = pulseIn(sensorOut, LOW, 100000);
  if (redFrequency == 0) redFrequency = 99999;

  delay(30);

  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  delay(10);
  greenFrequency = pulseIn(sensorOut, LOW, 100000);
  if (greenFrequency == 0) greenFrequency = 99999;
}

char detectColor() {
  readColorSensor();

  long freqDiff = redFrequency - greenFrequency;

  Serial.print("Color Sensor - R_freq:"); Serial.print(redFrequency);
  Serial.print(" G_freq:"); Serial.print(greenFrequency);
  Serial.print(" Diff:"); Serial.println(freqDiff);

  char result = 'N';

  if (freqDiff > 30000) {
    Serial.println(">>> GREEN detected!");
    result = 'G';
  }
  else if (freqDiff < 20000) {
    Serial.println(">>> RED detected!");
    result = 'R';
  }
  else {
    Serial.println(">>> UNKNOWN color!");
    result = 'N';
  }

  displayColorFreq(redFrequency, greenFrequency, freqDiff, result);
  return result;
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
// PID STRAFE ALIGN FUNCTIONS (Phase 1 only)
//============================================================
void resetStrafePID() {
  strafeError = 0;
  strafePrevError = 0;
  strafeIntegral = 0;
  strafePrevTime = 0;
}

int computeStrafePIDPulse(float error) {
  unsigned long now = millis();
  float dt = (strafePrevTime == 0) ? 0.05 : (now - strafePrevTime) / 1000.0;

  if (dt <= 0) dt = 0.05;
  if (dt > 0.3) dt = 0.3;

  strafeIntegral += error * dt;
  strafeIntegral = constrain(strafeIntegral, -strafeIntegralLimit, strafeIntegralLimit);

  float derivative = (error - strafePrevError) / dt;
  float output = strafeKp * error + strafeKi * strafeIntegral + strafeKd * derivative;

  strafePrevError = error;
  strafePrevTime = now;

  int pulse = (int)abs(output);
  pulse = constrain(pulse, strafePidMinPulse, strafePidMaxPulse);

  return pulse;
}

//============================================================
// COMBINED ALIGNMENT FUNCTIONS - PHASE 1
//============================================================
int alignLoopCount = 0;
int strafeLoopCount = 0;

void resetCombinedAlign() {
  alignLoopCount = 0;
  strafeLoopCount = 0;
  resetWallPID();
  resetStrafePID();
}

void doCombinedAlignment(RobotState nextState, const char* title) {
  alignLoopCount++;

  if (alignLoopCount > MAX_ALIGN_LOOP) {
    stopAll();
    displayState("ALIGN TIMEOUT", "Max loops reached", "Proceeding...");
    Serial.println(">>> Combined alignment timeout!");
    delay(200);
    resetCombinedAlign();
    currentState = nextState;
    return;
  }

  sampleFrontAverage(ALIGN_SAMPLE_MS);
  sampleSideDistances(ALIGN_SAMPLE_MS);
  sampleLightAverage();

  int wallErr = distance_front_left - distance_front_right;
  long avgDist = getAverageFrontDistance();
  int distErr = (avgDist == 999) ? 999 : (avgDist - TARGET_28CM);

  int sideDiffSigned = distance_side_left - distance_side_right;
  int sideDiffAbs = abs(sideDiffSigned);
  int lightDiffSigned = lightLeftIntensity - lightRightIntensity;

  bool wallOK = abs(wallErr) <= WALL_PARALLEL_TOL;
  bool distOK = (avgDist != 999 && abs(distErr) <= FRONT_DIST_TOL);
  bool sideOK = (distance_side_left != 999 && distance_side_right != 999 && 
                 sideDiffAbs <= SIDE_EQUAL_TOL);

  Serial.print("Align Loop "); Serial.print(alignLoopCount);
  Serial.print(" | WallErr:"); Serial.print(wallErr);
  Serial.print(" AvgDist:"); Serial.print(avgDist);
  Serial.print(" | SideDiff:"); Serial.println(sideDiffSigned);

  if (wallOK && distOK && sideOK) {
    stopAll();
    displayState("PH1 OK!", "Wall+26cm+SideEq", "");
    Serial.println(">>> Phase 1 complete!");
    delay(200);
    resetCombinedAlign();
    currentState = nextState;
    return;
  }

  if (!wallOK) {
    int pulseMs = computeWallPIDPulse(wallErr);
    Serial.print(">>> Wall adjust, pulse: "); Serial.println(pulseMs);

    displayCombinedAlignStatus(distance_front_left, distance_front_right, avgDist, wallErr,
                               distance_side_left, distance_side_right, sideDiffSigned,
                               pulseMs, "WALL", "ROT");

    if (wallErr > 0) rotateRight(Motor_PWM_Rotate);
    else rotateLeft(Motor_PWM_Rotate);

    delay(pulseMs);
    stopAll();
    delay(30);
    return;
  }

  if (!distOK) {
    if (avgDist == 999) {
      stopAll();
      displayState("PH1 DIST", "No reading", "");
      delay(100);
      return;
    }

    displayCombinedAlignStatus(distance_front_left, distance_front_right, avgDist, wallErr,
                               distance_side_left, distance_side_right, sideDiffSigned,
                               0, "DIST", distErr > 0 ? "FWD" : "BACK");

    if (distErr > 0) moveForward(Motor_PWM_Forward);
    else moveBackward(Motor_PWM_Forward);

    delay(STEP_APPROACH_MS);
    stopAll();
    delay(FORWARD_MEASURE_PAUSE_MS);
    resetWallPID();
    resetStrafePID();
    return;
  }

  if (!sideOK) {
    if (distance_side_left == 999 || distance_side_right == 999) {
      stopAll();
      displayState("PH1 SIDE", "No side reading", "");
      delay(100);
      return;
    }

    strafeLoopCount++;

    bool strafeRight;
    String guideMode;

    // All strafe use fixed pulse, no PID
    if (strafeLoopCount <= LIGHT_GUIDE_LOOPS) {
      // First 10 loops: Use LIGHT SENSOR to guide strafe direction
      guideMode = "LIGHT";
      
      if (lightDiffSigned > 0) {
        // Left light > Right light -> strafe LEFT
        strafeRight = false;
      } else {
        // Right light >= Left light -> strafe RIGHT
        strafeRight = true;
      }
    } else {
      // After 10 loops: Use SIDE DISTANCE difference to guide direction
      guideMode = "SIDE";
      
      // Direction based on side distance difference
      // side_left > side_right -> need to strafe RIGHT to balance
      // side_left < side_right -> need to strafe LEFT to balance
      if (sideDiffSigned > 0) {
        strafeRight = false;
      } else {
        strafeRight = true;
      }
    }

    Serial.print(">>> Strafe "); Serial.print(strafeRight ? "RIGHT" : "LEFT");
    Serial.print(" ("); Serial.print(guideMode); Serial.println(")");

    displayCombinedAlignStatus(distance_front_left, distance_front_right, avgDist, wallErr,
                               distance_side_left, distance_side_right, sideDiffSigned,
                               ALIGN_PULSE_MS, guideMode, strafeRight ? "R" : "L");

    if (strafeRight) moveRight(Motor_PWM_Strafe);
    else moveLeft(Motor_PWM_Strafe);

    delay(ALIGN_PULSE_MS);  // Fixed pulse for all strafe
    stopAll();
    delay(30);
    resetWallPID();
    return;
  }
}

//============================================================
// WALL ALIGNMENT (Phase 2 Initial)
//============================================================
int wallAlignLoopCount = 0;

void resetWallAlign() {
  wallAlignLoopCount = 0;
  resetWallPID();
}

bool doWallAlignPhase2() {
  wallAlignLoopCount++;

  if (wallAlignLoopCount > MAX_ALIGN_LOOP) {
    Serial.println(">>> Wall align timeout!");
    return true;
  }

  sampleFrontAverage(ALIGN_SAMPLE_MS);

  int wallErr = distance_front_left - distance_front_right;
  bool wallOK = abs(wallErr) <= WALL_PARALLEL_TOL;

  displayPhase2Wall(distance_front_left, distance_front_right, wallErr, wallOK);

  Serial.print("P2 Wall | FL:"); Serial.print(distance_front_left);
  Serial.print(" FR:"); Serial.print(distance_front_right);
  Serial.print(" Err:"); Serial.print(wallErr);
  Serial.print(" OK:"); Serial.println(wallOK ? "YES" : "NO");

  if (wallOK) {
    Serial.println(">>> Wall aligned!");
    return true;
  }

  int pulseMs = computeWallPIDPulse(wallErr);

  if (wallErr > 0) rotateRight(Motor_PWM_Rotate);
  else rotateLeft(Motor_PWM_Rotate);

  delay(pulseMs);
  stopAll();
  delay(30);

  return false;
}

//============================================================
// SIDE DISTANCE ALIGNMENT (Phase 2 - Simple Step Strafe)
//============================================================
int sideAlignLoopCount = 0;

void resetSideAlign() {
  sideAlignLoopCount = 0;
}

bool doSideAlignPhase2() {
  sideAlignLoopCount++;

  if (sideAlignLoopCount > MAX_SIDE_ALIGN_LOOP) {
    Serial.println(">>> Side align timeout!");
    return true;
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
    Serial.println(">>> Side sensor no reading!");
    return false;
  }

  int sideErr = sideDist - SIDE_TARGET_DIST;
  bool sideOK = abs(sideErr) <= 1;

  displayPhase2Side(sideDist, SIDE_TARGET_DIST, sensorName, sideOK);

  Serial.print("P2 Side | Sensor:"); Serial.print(sensorName);
  Serial.print(" Dist:"); Serial.print(sideDist);
  Serial.print(" Target:"); Serial.print(SIDE_TARGET_DIST);
  Serial.print(" Err:"); Serial.print(sideErr);
  Serial.print(" OK:"); Serial.println(sideOK ? "YES" : "NO");

  if (sideOK) {
    Serial.println(">>> Side distance aligned!");
    return true;
  }

  // Simple step strafe - fixed pulse, no PID
  // Determine strafe direction based on color turn
  if (detectedColor == 'G') {
    if (sideErr > 0) moveRight(Motor_PWM_Strafe);
    else moveLeft(Motor_PWM_Strafe);
  } else {
    if (sideErr > 0) moveLeft(Motor_PWM_Strafe);
    else moveRight(Motor_PWM_Strafe);
  }

  delay(SIDE_STRAFE_PULSE_MS);  // Fixed pulse
  stopAll();
  delay(30);

  return false;
}

//============================================================
// PHASE 2 LOOP (Step + Wall + Side until distance <= 15cm)
//============================================================
int phase2LoopCount = 0;
enum Phase2SubState {
  P2_STEP,
  P2_WALL,
  P2_SIDE
};
Phase2SubState p2SubState = P2_STEP;

void resetPhase2Loop() {
  phase2LoopCount = 0;
  p2SubState = P2_STEP;
  resetWallPID();
  resetStrafePID();
}

bool doPhase2Loop() {
  sampleFrontAverage(ALIGN_SAMPLE_MS);
  long frontDist = getAverageFrontDistance();

  // Check if we should exit to final approach
  if (frontDist != 999 && frontDist <= SLOW_APPROACH_DIST_PHASE2) {
    stopAll();
    displayState("P2 LOOP DONE", "Front <= 15cm", "Final approach");
    Serial.print(">>> Phase 2 loop complete! Front: "); Serial.print(frontDist); Serial.println("cm");
    delay(200);
    return true;
  }

  if (phase2LoopCount > MAX_PHASE2_LOOP) {
    stopAll();
    displayState("P2 TIMEOUT", "Max loops", "Final approach");
    Serial.println(">>> Phase 2 loop timeout!");
    delay(200);
    return true;
  }

  switch (p2SubState) {
    case P2_STEP:
      {
        phase2LoopCount++;
        Serial.print("=== P2 Loop #"); Serial.print(phase2LoopCount);
        Serial.print(" Front:"); Serial.print(frontDist); Serial.println("cm ===");

        displayPhase2Loop(phase2LoopCount, frontDist, 0, "STEP");

        moveForward(Motor_PWM_Forward);
        delay(STEP_APPROACH_MS);
        stopAll();
        delay(FORWARD_MEASURE_PAUSE_MS);

        p2SubState = P2_WALL;
        resetWallAlign();
        return false;
      }

    case P2_WALL:
      {
        sampleSideDistances(100);
        long sideDist = (detectedColor == 'G') ? distance_side_right : distance_side_left;
        displayPhase2Loop(phase2LoopCount, frontDist, sideDist, "WALL");

        if (doWallAlignPhase2()) {
          p2SubState = P2_SIDE;
          resetSideAlign();
        }
        return false;
      }

    case P2_SIDE:
      {
        displayPhase2Loop(phase2LoopCount, frontDist, 0, "SIDE");

        if (doSideAlignPhase2()) {
          p2SubState = P2_STEP;
        }
        return false;
      }

    default:
      p2SubState = P2_STEP;
      return false;
  }
}

//============================================================
// FINAL APPROACH
//============================================================
void doFinalApproach(RobotState nextState) {
  static int stepCount = 0;

  if (stepCount == 0) {
    stepCount = 1;
    Serial.println("=== Final Approach (to 8cm) ===");
  }

  if (stepCount > MAX_STEP_APPROACH) {
    stopAll();
    displayFinalApproach(0, FINAL_STOP_DIST, stepCount);
    Serial.println(">>> Final approach timeout!");
    delay(200);
    stepCount = 0;
    currentState = nextState;
    return;
  }

  sampleFrontAverage(ALIGN_SAMPLE_MS);
  long avgDist = getAverageFrontDistance();

  if (avgDist == 999) {
    stopAll();
    delay(100);
    return;
  }

  Serial.print("Final Step "); Serial.print(stepCount);
  Serial.print(" | Dist:"); Serial.print(avgDist);
  Serial.print(" Target:"); Serial.println(FINAL_STOP_DIST);

  displayFinalApproach(avgDist, FINAL_STOP_DIST, stepCount);

  if (avgDist <= FINAL_STOP_DIST) {
    stopAll();
    displayFinalApproach(avgDist, FINAL_STOP_DIST, stepCount);
    Serial.println(">>> Final target reached!");
    delay(200);
    stepCount = 0;
    currentState = nextState;
    return;
  }

  moveForward(Motor_PWM_Forward);
  delay(STEP_APPROACH_MS);
  stopAll();
  delay(FORWARD_MEASURE_PAUSE_MS);

  stepCount++;
}

//============================================================
// AUTONOMOUS ROBOT LOGIC
//============================================================
void autonomousControl() {
  if (currentState == STATE_FINAL_APPROACH || currentState == STATE_MOVE_TO_TURN_POINT ||
      currentState == STATE_PHASE2_LOOP) {
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
        displayManualValues();
      }
      break;

    case STATE_READY:
      stopAll();
      if (stateChanged) {
        displayState("READY!", "Press switch", "to START");
      }
      break;

    case STATE_MOVE_FORWARD:
      {
        if (stateChanged) {
          displayState("MOVE FWD 1", "While loop", "Dist only");
          Serial.println("=== Phase 1: Move Forward ===");
        }

        bool keepMoving = true;
        int stepCount = 0;

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

          if (keepMoving && isWallDetected(fl, fr)) {
            keepMoving = false;
            Serial.println(">>> Wall detected!");
            displayState("WALL DETECT", "Dist FL:" + String(fl), "FR:" + String(fr));
            delay(200);
          }

          if (keepMoving) {
            long avgDist = getAverageFrontDistance();
            if (avgDist != 999 && avgDist <= WALL_APPROACH_DIST) {
              keepMoving = false;
              Serial.println(">>> Too close to wall!");
            }
          }
        }

        stopAll();
        currentState = STATE_ALIGN_COMBINED;
      }
      break;

    case STATE_ALIGN_COMBINED:
      if (stateChanged) {
        resetCombinedAlign();
        displayState("ALIGN PHASE1", "Wall+26+SideEq", "Light->Side");
        Serial.println("=== Phase 1 Combined Alignment ===");
      }
      doCombinedAlignment(STATE_DETECT_COLOR, "ALIGN 1");
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

        if (distance_front_left == 999 && distance_front_right == 999) {
          stopAll();
          delay(30);
          break;
        }

        if (avgDist != 999 && avgDist <= COLOR_TURN_DIST) {
          stopAll();
          displayState("TURN POINT", "Reached 24cm", "Rotate now");
          Serial.println(">>> Reached 24cm, start turning");
          delay(200);
          currentState = STATE_ROTATING_90;
        } else {
          moveForward(Motor_PWM_Forward);
        }

        delay(30);
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
          if (detectedColor == 'R') rotateRight(Motor_PWM_Rotate);
          else rotateLeft(Motor_PWM_Rotate);
        }
      }
      break;

    case STATE_MOVE_FORWARD_2:
      {
        if (stateChanged) {
          displayState("MOVE FWD 2", "While loop", "Dist only");
          Serial.println("=== Phase 2: Move Forward ===");
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

          if (isWallDetected(fl, fr)) {
            keepMoving = false;
            Serial.println(">>> Wall detected!");
          }

          if (keepMoving) {
            long avgDist = getAverageFrontDistance();
            if (avgDist != 999 && avgDist <= SLOW_APPROACH_DIST_PHASE2) {
              keepMoving = false;
              Serial.println(">>> Reached 15cm zone!");
            }
          }
        }

        stopAll();
        currentState = STATE_WALL_ALIGN_2;
      }
      break;

    case STATE_WALL_ALIGN_2:
      if (stateChanged) {
        resetWallAlign();
        displayState("WALL ALIGN 2", "Initial parallel", "");
        Serial.println("=== Phase 2 Initial Wall Alignment ===");
      }

      if (doWallAlignPhase2()) {
        resetPhase2Loop();
        currentState = STATE_PHASE2_LOOP;
      }
      break;

    case STATE_PHASE2_LOOP:
      if (stateChanged) {
        displayState("P2 LOOP", "Step+Wall+Side", "Until <=15cm");
        Serial.println("=== Phase 2 Loop: Step + Wall + Side ===");
      }

      if (doPhase2Loop()) {
        currentState = STATE_FINAL_APPROACH;
      }
      break;

    case STATE_FINAL_APPROACH:
      if (stateChanged) {
        displayState("FINAL", "Approach 8cm", "");
        Serial.println("=== Final Approach ===");
      }
      doFinalApproach(STATE_COMPLETE);
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
  Serial.println("Light Tracking Robot v2.24");
  Serial.println("Manual Light Values - No Calibration");
  Serial.println("================================");

  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 failed"));
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("v2.24");
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

  // Print manual light sensor values
  Serial.println("");
  Serial.println("=== MANUAL LIGHT SENSOR VALUES ===");
  Serial.print("Left Base:  "); Serial.println(lightLeftBase);
  Serial.print("Right Base: "); Serial.println(lightRightBase);
  Serial.print("Left Min:   "); Serial.println(lightLeftMin);
  Serial.print("Right Min:  "); Serial.println(lightRightMin);
  Serial.println("==================================");
  Serial.println("");

  displayManualValues();
  delay(2000);

  displayState("IDLE", "Press switch", "to START");
  Serial.println("Setup complete!");
  Serial.println("");
  Serial.println("Flow: IDLE -> READY -> MOVE_FORWARD");
  Serial.println("Phase 1 strafe: Simple step strafe (no PID)");
  Serial.print("  - First "); Serial.print(LIGHT_GUIDE_LOOPS); Serial.println(" loops: Light sensor guides direction");
  Serial.println("  - After: Side distance diff guides direction");
  Serial.print("  - Fixed pulse: "); Serial.print(ALIGN_PULSE_MS); Serial.println("ms");
  Serial.println("");
  Serial.println("Phase 2 side alignment: Simple step strafe (fixed pulse)");
  Serial.print("  - Fixed pulse: "); Serial.print(SIDE_STRAFE_PULSE_MS); Serial.println("ms");
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
        // Skip calibration, go directly to READY
        currentState = STATE_READY;
        Serial.println(">>> Switch to READY state");
      }
      else if (currentState == STATE_READY) {
        currentState = STATE_MOVE_FORWARD;
        Serial.println(">>> START! Moving forward...");
      }
    }
  }

  if (digitalRead(START_SWITCH_PIN) == HIGH) {
    switchPressed = false;
  }

  autonomousControl();

  delay(10);
}
