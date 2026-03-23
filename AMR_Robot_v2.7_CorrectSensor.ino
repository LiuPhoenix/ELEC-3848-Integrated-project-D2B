/*****************************************************
Autonomous Mobile Robot (AMR) - Fixed Sensor Selection
Version: 2.7
Date: 2026-03-23

CORRECTED Sensor Selection Logic:
- RED detected → Turn RIGHT 90° → LEFT side faces wall → Use LEFT sensor
- GREEN detected → Turn LEFT 90° → RIGHT side faces wall → Use RIGHT sensor

*****************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//============================================================
//                    PIN DEFINITIONS
//============================================================

//------------------------
// OLED Display (I2C)
//------------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//------------------------
// Color Sensor (TCS3200)
//------------------------
#define S0 4
#define S1 6
#define S2 7
#define S3 10
#define sensorOut 11

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
#define START_SWITCH_PIN  25

//============================================================
//                    GLOBAL OBJECTS
//============================================================

MPU6050 mpu(Wire);

//============================================================
//                    GLOBAL VARIABLES
//============================================================

//------------------------
// Gyroscope Variables
//------------------------
float currentYaw = 0;
float rotationStartYaw = 0;
bool gyroCalibrated = false;

//------------------------
// Color Sensor Variables
//------------------------
int redFrequency = 0;
int greenFrequency = 0;
int redColor = 0;
int greenColor = 0;

int redMin = 110, redMax = 140;
int greenMin = 35, greenMax = 45;

//------------------------
// Ultrasonic Variables
//------------------------
long distance_front_left = 0;
long distance_front_right = 0;
long distance_side_left = 0;
long distance_side_right = 0;

//------------------------
// Motor Control Variables
//------------------------
int Motor_PWM = 200;
int Motor_PWM_Slow = 100;

//------------------------
// Robot State Machine
//------------------------
enum RobotState {
  STATE_IDLE,
  STATE_GYRO_CALIBRATING,
  STATE_GYRO_CALIBRATED,
  STATE_ALIGNING,
  STATE_APPROACHING_32CM,
  STATE_PARALLEL_MOVE,
  STATE_ROTATING_90,
  STATE_FOLLOWING_WALL_24CM,
  STATE_APPROACHING_5CM,
  STATE_COMPLETE
};

RobotState currentState = STATE_IDLE;
RobotState previousState = STATE_IDLE;
char detectedColor = 'N';

//------------------------
// Key Distances
//------------------------
#define DISTANCE_WALL_32CM   32
#define DISTANCE_WALL_24CM   24
#define DISTANCE_WALL_5CM    8
#define PARALLEL_TOLERANCE   2
#define ANGLE_TOLERANCE      3.0
#define SIDE_TOLERANCE       3

//============================================================
//                    MOTOR CONTROL MACROS
//============================================================

#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH);analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP()          do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,HIGH);analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP()          do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,HIGH);digitalWrite(DIRB2,LOW); analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH);analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP()          do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,HIGH);analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP()          do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,HIGH);digitalWrite(DIRD2,LOW); analogWrite(PWMD,pwm);}while(0)

//============================================================
//                    MOTOR MOVEMENT FUNCTIONS
//============================================================

void moveForward(int pwm) {
  MOTORA_FORWARD(pwm);
  MOTORB_BACKOFF(pwm);
  MOTORC_FORWARD(pwm);
  MOTORD_BACKOFF(pwm);
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

// 斜向移动
void moveForwardAdjustLeft(int pwm) {
  MOTORA_STOP();
  MOTORB_BACKOFF(pwm);
  MOTORC_FORWARD(pwm);
  MOTORD_STOP();
}

void moveForwardAdjustRight(int pwm) {
  MOTORA_FORWARD(pwm);
  MOTORB_STOP();
  MOTORC_STOP();
  MOTORD_BACKOFF(pwm);
}

//============================================================
//                    OLED DISPLAY FUNCTIONS
//============================================================

void displayState(String state, String info1, String info2) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0, 0);
  display.println(state);
  
  display.setCursor(0, 20);
  display.println(info1);
  
  display.setCursor(0, 40);
  display.println(info2);
  
  display.display();
}

void displayDistance(String title, int frontL, int frontR, int sideL, int sideR) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0, 0);
  display.println(title);
  
  display.setCursor(0, 20);
  display.print("FL:"); display.print(frontL);
  display.print(" FR:"); display.println(frontR);
  
  display.setCursor(0, 40);
  display.print("SL:"); display.print(sideL);
  display.print(" SR:"); display.println(sideR);
  
  display.display();
}

void displayColor(int r, int g, char result) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0, 0);
  display.println("COLOR DETECT");
  
  display.setCursor(0, 20);
  display.print("R:"); display.print(r);
  display.print(" G:"); display.println(g);
  
  display.setCursor(0, 40);
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
  display.println("ROTATING 90");
  
  display.setCursor(0, 20);
  display.print("Rotated: "); display.print((int)rotated); display.println(" deg");
  
  display.display();
}

void displayWallFollow(int sideDist, int frontDist, char sideLabel) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0, 0);
  display.print("WALL FOLLOW ");
  display.println(detectedColor == 'R' ? "(RED)" : "(GRN)");
  
  display.setCursor(0, 20);
  display.print("Side"); display.print(sideLabel); display.print(": ");
  display.print(sideDist); display.println("cm");
  
  display.setCursor(0, 40);
  display.print("Front: "); display.print(frontDist); display.println("cm");
  
  display.display();
}

//============================================================
//                    ULTRASONIC FUNCTIONS
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

bool isParallelToWall() {
  return abs(distance_front_left - distance_front_right) <= PARALLEL_TOLERANCE;
}

long getAverageFrontDistance() {
  return (distance_front_left + distance_front_right) / 2;
}

//============================================================
//                    GYROSCOPE FUNCTIONS
//============================================================

void updateGyro() {
  mpu.update();
  currentYaw = mpu.getAngleZ();
}

void calibrateGyro() {
  Serial.println("Calibrating Gyroscope... DO NOT MOVE!");
  displayState("CALIBRATING", "Do NOT move!", "");
  
  delay(1000);
  mpu.calcGyroOffsets();
  gyroCalibrated = true;
  
  Serial.println("Gyro calibration done!");
  displayState("CALIBRATED!", "Press switch", "to start");
}

float getRotatedAngle(float startAngle, float currentAngle) {
  float diff = currentAngle - startAngle;
  
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;
  
  return abs(diff);
}

//============================================================
//                    COLOR SENSOR FUNCTIONS
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
  
  Serial.print("R:"); Serial.print(redColor);
  Serial.print(" G:"); Serial.println(greenColor);
  
  displayColor(redColor, greenColor, 'N');
  
  if (redColor > greenColor) {
    Serial.println("RED detected!");
    displayColor(redColor, greenColor, 'R');
    return 'R';
  }
  else if (greenColor > redColor) {
    Serial.println("GREEN detected!");
    displayColor(redColor, greenColor, 'G');
    return 'G';
  }
  return 'N';
}

//============================================================
//                    AUTONOMOUS ROBOT LOGIC
//============================================================

void autonomousControl() {
  updateDistances();
  updateGyro();
  
  bool stateChanged = (currentState != previousState);
  if (stateChanged) {
    previousState = currentState;
  }
  
  //========================================
  // 正确的传感器选择逻辑：
  // 
  // RED → 右转90° → 左侧朝墙 → 用 LEFT sensor
  // GREEN → 左转90° → 右侧朝墙 → 用 RIGHT sensor
  //========================================
  long sideDistance = 0;
  char sideLabel = '?';
  
  if (detectedColor == 'R') {
    // 右转后，左侧朝墙
    sideDistance = distance_side_left;
    sideLabel = 'L';
  } else if (detectedColor == 'G') {
    // 左转后，右侧朝墙
    sideDistance = distance_side_right;
    sideLabel = 'R';
  }
  
  switch (currentState) {
    
    //========================================
    case STATE_IDLE:
    //========================================
      stopAll();
      if (stateChanged) {
        displayState("IDLE", "Press switch", "to calibrate");
        Serial.println("IDLE");
      }
      break;
    
    //========================================
    case STATE_GYRO_CALIBRATING:
    //========================================
      if (stateChanged) {
        calibrateGyro();
        currentState = STATE_GYRO_CALIBRATED;
      }
      break;
    
    //========================================
    case STATE_GYRO_CALIBRATED:
    //========================================
      stopAll();
      if (stateChanged) {
        displayState("READY!", "Press switch", "to START");
        Serial.println("READY!");
      }
      break;
    
    //========================================
    case STATE_ALIGNING:
    //========================================
      if (stateChanged) {
        displayState("ALIGNING", "Parallel to wall", "...");
        Serial.println("ALIGNING");
      }
      
      if (isParallelToWall()) {
        stopAll();
        displayState("ALIGNED!", "Ready", "");
        Serial.println("Aligned!");
        currentState = STATE_APPROACHING_32CM;
        delay(500);
      } else {
        displayDistance("ALIGN", distance_front_left, distance_front_right, 
                        distance_side_left, distance_side_right);
        if (distance_front_left > distance_front_right) {
          rotateLeft(Motor_PWM_Slow);
        } else {
          rotateRight(Motor_PWM_Slow);
        }
      }
      break;
    
    //========================================
    case STATE_APPROACHING_32CM:
    //========================================
      {
        long avgDist = getAverageFrontDistance();
        
        if (stateChanged) {
          displayState("APPROACH", "Target: 32cm", "");
          Serial.println("APPROACH 32cm");
        }
        
        displayDistance("-> 32cm", distance_front_left, distance_front_right,
                        distance_side_left, distance_side_right);
        
        if (avgDist <= DISTANCE_WALL_32CM) {
          stopAll();
          displayState("REACHED!", "32cm", "");
          Serial.println("Reached 32cm!");
          currentState = STATE_PARALLEL_MOVE;
          delay(500);
        } else {
          if (!isParallelToWall()) {
            if (distance_front_left > distance_front_right) {
              moveForwardAdjustLeft(Motor_PWM_Slow);
            } else {
              moveForwardAdjustRight(Motor_PWM_Slow);
            }
          } else {
            moveForward(Motor_PWM_Slow);
          }
        }
      }
      break;
    
    //========================================
    case STATE_PARALLEL_MOVE:
    //========================================
      if (stateChanged) {
        displayState("PARALLEL", "Finding color", "...");
        Serial.println("PARALLEL MOVE");
      }
      
      detectedColor = detectColor();
      
      if (detectedColor != 'N') {
        stopAll();
        String colorStr = (detectedColor == 'R') ? "RED -> Right" : "GREEN -> Left";
        displayState("COLOR!", colorStr, "");
        Serial.print("Color: "); Serial.println(detectedColor);
        delay(1000);
        currentState = STATE_ROTATING_90;
      } else {
        moveLeft(Motor_PWM_Slow);
      }
      break;
    
    //========================================
    case STATE_ROTATING_90:
    //========================================
      {
        if (stateChanged) {
          rotationStartYaw = currentYaw;
          String dirStr = (detectedColor == 'R') ? "Turn RIGHT" : "Turn LEFT";
          displayState("ROTATING", dirStr, "90 degrees");
          Serial.print("ROTATING - ");
          Serial.println(dirStr);
        }
        
        float rotated = getRotatedAngle(rotationStartYaw, currentYaw);
        displayRotation(rotated);
        Serial.print("Rotated: "); Serial.println(rotated);
        
        if (rotated >= 85.0) {
          stopAll();
          displayState("ROTATED!", "90 degrees", "complete");
          Serial.println("Rotation complete!");
          delay(500);
          currentState = STATE_FOLLOWING_WALL_24CM;
        } else {
          if (detectedColor == 'R') {
            rotateRight(Motor_PWM_Slow);
          } else {
            rotateLeft(Motor_PWM_Slow);
          }
        }
      }
      break;
    
    //========================================
    case STATE_FOLLOWING_WALL_24CM:
    //========================================
      {
        long frontDist = getAverageFrontDistance();
        
        if (stateChanged) {
          String sensorStr = (detectedColor == 'R') ? "Use LEFT sensor" : "Use RIGHT sensor";
          displayState("WALL FOLLOW", sensorStr, "Side: 24cm");
          Serial.print("WALL FOLLOW - ");
          Serial.println(sensorStr);
        }
        
        displayWallFollow(sideDistance, frontDist, sideLabel);
        
        Serial.print("Side"); Serial.print(sideLabel); Serial.print(": "); Serial.print(sideDistance);
        Serial.print(" Front: "); Serial.println(frontDist);
        
        // 到达前方5cm
        if (frontDist <= DISTANCE_WALL_5CM) {
          stopAll();
          displayState("REACHED!", "5cm from wall", "");
          Serial.println("Reached 5cm!");
          currentState = STATE_APPROACHING_5CM;
          return;
        }
        
        //========================================
        // 前进时保持侧面24cm
        // 
        // RED(右转): 左侧靠墙
        //   - 左侧距离 > 24cm → 左移靠近墙
        //   - 左侧距离 < 24cm → 右移远离墙
        // 
        // GREEN(左转): 右侧靠墙
        //   - 右侧距离 > 24cm → 右移靠近墙
        //   - 右侧距离 < 24cm → 左移远离墙
        //========================================
        
        if (sideDistance > DISTANCE_WALL_24CM + SIDE_TOLERANCE) {
          // 离墙太远，需要靠近墙
          Serial.println("Too far, move closer");
          
          if (detectedColor == 'R') {
            // 左侧靠墙，左移靠近
            moveForwardAdjustLeft(Motor_PWM_Slow);
          } else {
            // 右侧靠墙，右移靠近
            moveForwardAdjustRight(Motor_PWM_Slow);
          }
        } 
        else if (sideDistance < DISTANCE_WALL_24CM - SIDE_TOLERANCE) {
          // 离墙太近，需要远离墙
          Serial.println("Too close, move away");
          
          if (detectedColor == 'R') {
            // 左侧靠墙，右移远离
            moveForwardAdjustRight(Motor_PWM_Slow);
          } else {
            // 右侧靠墙，左移远离
            moveForwardAdjustLeft(Motor_PWM_Slow);
          }
        } 
        else {
          // 距离正好，直走
          Serial.println("OK, forward");
          moveForward(Motor_PWM_Slow);
        }
      }
      break;
    
    //========================================
    case STATE_APPROACHING_5CM:
    //========================================
      {
        long frontDist = getAverageFrontDistance();
        
        if (stateChanged) {
          displayState("FINAL", "Approach 5cm", "");
          Serial.println("FINAL 5cm");
        }
        
        displayState("FINAL", "Front:", String(frontDist) + " cm");
        
        if (frontDist <= DISTANCE_WALL_5CM) {
          stopAll();
          currentState = STATE_COMPLETE;
        } else {
          moveForward(Motor_PWM_Slow);
        }
      }
      break;
    
    //========================================
    case STATE_COMPLETE:
    //========================================
      stopAll();
      if (stateChanged) {
        displayState("COMPLETE!", "Task done!", ":)");
        Serial.println("========== COMPLETE! ==========");
      }
      break;
  }
}

//============================================================
//                    SETUP
//============================================================

void setup() {
  Serial.begin(115200);
  Serial.println("================================");
  Serial.println("AMR Robot v2.7 - Correct Sensor");
  Serial.println("================================");
  
  Wire.begin();
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 failed"));
  }
  
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("AMR v2.7");
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
  
  pinMode(START_SWITCH_PIN, INPUT_PULLUP);
  
  displayState("READY!", "Press switch", "to calibrate");
  Serial.println("Setup complete!");
}

//============================================================
//                    MAIN LOOP
//============================================================

void loop() {
  static bool switchPressed = false;
  static unsigned long lastPressTime = 0;
  
  if (digitalRead(START_SWITCH_PIN) == LOW && !switchPressed) {
    if (millis() - lastPressTime > 300) {
      switchPressed = true;
      lastPressTime = millis();
      
      if (currentState == STATE_IDLE) {
        currentState = STATE_GYRO_CALIBRATING;
      } else if (currentState == STATE_GYRO_CALIBRATED) {
        currentState = STATE_ALIGNING;
      }
    }
  }
  if (digitalRead(START_SWITCH_PIN) == HIGH) {
    switchPressed = false;
  }
  
  autonomousControl();
  
  delay(10);
}
