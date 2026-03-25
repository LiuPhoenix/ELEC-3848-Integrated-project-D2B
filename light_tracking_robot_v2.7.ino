/*****************************************************
Light Tracking Robot with Color Detection + PID Wall Align
Version: 2.7 (Color -> Go to 24cm -> Turn)
Date: 2026-03-25

Flow:
1. Move forward (while loop with conditions)
2. Stop when wall detected
3. Align wall (PID rotate)
4. Align light (strafe)
5. Detect color
6. Move forward until front average distance = 24cm
7. Rotate 90° (gyro)
8. Move forward (while loop)
9. Align wall
10. Align light
11. Final approach
12. Complete

Changes from v2.6:
- Removed "light target found" stop condition from forward movement
- Light intensity > 95% now ONLY used for light alignment phase
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

int redMin = 850, redMax = 950;
int greenMin = 1000, greenMax = 1100;

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
// Motor Speed
//------------------------
int Motor_PWM_Normal = 80;
int Motor_PWM_Slow = 50;

//============================================================
// ROBOT STATE MACHINE
//============================================================
enum RobotState {
  STATE_IDLE,
  STATE_CALIBRATING,
  STATE_READY,
  STATE_MOVE_FORWARD,
  STATE_ALIGN_WALL,
  STATE_ALIGN_LIGHT,
  STATE_DETECT_COLOR,
  STATE_MOVE_TO_TURN_POINT,
  STATE_ROTATING_90,
  STATE_MOVE_FORWARD_2,
  STATE_ALIGN_WALL_2,
  STATE_ALIGN_LIGHT_2,
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
//----------------------------
// REMOVED: LIGHT_INTENSITY_MIN - no longer used for stopping
// Light intensity > 95% is now ONLY used in light alignment phase
//----------------------------
#define LIGHT_TOLERANCE       5
#define LIGHT_ERROR_PERCENT   5.0
#define WALL_PARALLEL_TOL     2
#define WALL_APPROACH_DIST    30
#define COLOR_TURN_DIST       24
#define FINAL_STOP_DIST       10
#define ROTATE_TARGET_ANGLE   90.0

const unsigned long ALIGN_SAMPLE_MS = 300;
const int ALIGN_PULSE_MS = 80;
const int FORWARD_STEP_MS = 100;
const int FORWARD_MEASURE_PAUSE_MS = 20;

//------------------------
// While Loop Safety Limits
//------------------------
const int MAX_STEPS_PHASE1 = 150;
const int MAX_STEPS_PHASE2 = 100;

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
// MOTOR MOVEMENT FUNCTIONS
//============================================================
void moveForward(int pwm) {
  MOTORA_FORWARD(pwm);
  MOTORB_BACKOFF(pwm);
  MOTORC_FORWARD(pwm);
  MOTORD_BACKOFF(pwm);
}

void moveBackward(int pwm) {
  MOTORA_BACKOFF(pwm);
  MOTORB_FORWARD(pwm);
  MOTORC_BACKOFF(pwm);
  MOTORD_FORWARD(pwm);
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

void displayLight(int left, int right, int diff, String action) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("L:"); display.print(left);
  display.print(" R:"); display.println(right);

  display.setCursor(0, 12);
  display.print("Diff:"); display.print(diff);

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

// REMOVED: isLightTargetFound() function - no longer needed for stopping
// Light intensity > 95% check is now ONLY used in light alignment phase

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

void doWallAlignPID(RobotState nextState, const char* title) {
  sampleFrontAverage(ALIGN_SAMPLE_MS);

  if (distance_front_left == 999 || distance_front_right == 999) {
    stopAll();
    displayState("WALL ALIGN", "Echo invalid", "");
    delay(50);
    return;
  }

  wallError = distance_front_left - distance_front_right;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(title);
  display.setCursor(0, 12);
  display.print("FL:"); display.print(distance_front_left);
  display.print(" FR:"); display.println(distance_front_right);
  display.setCursor(0, 24);
  display.print("Err:"); display.println((int)wallError);
  display.display();

  if (abs(wallError) <= WALL_PARALLEL_TOL) {
    stopAll();
    resetWallPID();
    displayState("WALL OK", "Parallel!", "");
    delay(300);
    currentState = nextState;
    return;
  }

  int pulseMs = computeWallPIDPulse(wallError);

  // 哪边距离小，就往哪边转
  // wallError = FL - FR
  // 如果 FR 更小 => wallError > 0 => rotateRight()
  // 如果 FL 更小 => wallError < 0 => rotateLeft()
  if (wallError > 0) {
    rotateRight(Motor_PWM_Slow);
  } else {
    rotateLeft(Motor_PWM_Slow);
  }

  delay(pulseMs);
  stopAll();
  delay(30);
}

//============================================================
// LIGHT ALIGN FUNCTIONS
//============================================================
void doLightAlign(RobotState nextState, const char* title) {
  sampleLightAverage();

  int diff = lightLeftIntensity - lightRightIntensity;
  displayLight(lightLeftIntensity, lightRightIntensity, diff, String(title));

  if (abs(diff) <= LIGHT_TOLERANCE) {
    stopAll();
    displayState("LIGHT OK", "Aligned!", "");
    delay(300);
    currentState = nextState;
    return;
  }

  if (diff > LIGHT_TOLERANCE) {
    moveLeft(Motor_PWM_Slow);
  } else {
    moveRight(Motor_PWM_Slow);
  }

  delay(ALIGN_PULSE_MS);
  stopAll();
}

//============================================================
// AUTONOMOUS ROBOT LOGIC
//============================================================
void autonomousControl() {
  if (currentState == STATE_FINAL_APPROACH || currentState == STATE_MOVE_TO_TURN_POINT) {
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
    // 第一段前进 - MODIFIED: Removed light target found condition
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
          moveForward(Motor_PWM_Normal);
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

          // REMOVED: Light target found condition
          // Robot now ONLY stops based on distance/wall detection

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

        currentState = STATE_ALIGN_WALL;
      }
      break;

    case STATE_ALIGN_WALL:
      if (stateChanged) {
        resetWallPID();
        displayState("ALIGN WALL", "PID active", "");
      }
      doWallAlignPID(STATE_ALIGN_LIGHT, "ALIGN WALL");
      break;

    case STATE_ALIGN_LIGHT:
      if (stateChanged) {
        displayState("ALIGN LIGHT", "Sample 300ms", "");
      }
      doLightAlign(STATE_DETECT_COLOR, "STRAFE");
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
          moveForward(Motor_PWM_Slow);
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
            rotateRight(Motor_PWM_Slow);
          } else {
            rotateLeft(Motor_PWM_Slow);
          }
        }
      }
      break;

    //============================================================
    // 第二段前进 - MODIFIED: Removed light target found condition
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
          moveForward(Motor_PWM_Normal);
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

          // REMOVED: Light target found condition
          // Robot now ONLY stops based on distance/wall detection

          // 停止条件1: 检测到墙壁
          if (keepMoving && isWallDetected(fl, fr)) {
            keepMoving = false;
            stopReason = "WALL";
            Serial.println(">>> Wall detected in phase 2!");
          }

          // 停止条件2: 到达最终距离
          if (keepMoving) {
            long avgDist = getAverageFrontDistance();
            if (avgDist != 999 && avgDist <= FINAL_STOP_DIST) {
              keepMoving = false;
              stopReason = "FINAL";
              Serial.println(">>> Final distance reached!");
            }
          }
        }

        stopAll();
        Serial.print("Phase 2 complete: "); Serial.print(stepCount2);
        Serial.print(" steps, reason: "); Serial.println(stopReason);

        if (stopReason == "FINAL") {
          currentState = STATE_COMPLETE;
        } else {
          currentState = STATE_ALIGN_WALL_2;
        }
      }
      break;

    case STATE_ALIGN_WALL_2:
      if (stateChanged) {
        resetWallPID();
        displayState("ALIGN WALL 2", "PID active", "");
      }
      doWallAlignPID(STATE_ALIGN_LIGHT_2, "ALIGN WALL 2");
      break;

    case STATE_ALIGN_LIGHT_2:
      if (stateChanged) {
        displayState("ALIGN LIGHT 2", "Sample 300ms", "");
      }
      doLightAlign(STATE_FINAL_APPROACH, "STRAFE");
      break;

    case STATE_FINAL_APPROACH:
      {
        if (stateChanged) {
          displayState("FINAL", "Approaching...", "");
        }

        readLightSensors();
        calculateIntensity();

        long avgDist = getAverageFrontDistance();

        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("FINAL APPROACH");
        display.setCursor(0, 12);
        display.print("Dist:"); display.print(avgDist); display.println("cm");
        display.setCursor(0, 24);
        display.print("L:"); display.print(lightLeftIntensity);
        display.print(" R:"); display.println(lightRightIntensity);
        display.display();

        if (avgDist != 999 && avgDist <= FINAL_STOP_DIST) {
          stopAll();
          currentState = STATE_COMPLETE;
        } else {
          moveForward(Motor_PWM_Slow);
        }

        delay(50);
      }
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
  Serial.println("Light Tracking Robot v2.7");
  Serial.println("Color -> 24cm -> Turn");
  Serial.println("NO light stop in forward");
  Serial.println("================================");

  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 failed"));
  }

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("v2.7");
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
