/*
  Color Sensor Calibration
  color-sensor-calib.ino
  Calibrate RGB Color Sensor output Pulse Widths
  Uses values obtained for RGB Sensor Demo sketch 

  DroneBot Workshop 2020
  https://dronebotworkshop.com
*/

// Define color sensor pins

#define S0 4 //PG5
#define S1 6 //PH3
#define S2 7 //PH4
#define S3 10 //PB4
#define sensorOut 11 //PB5

// Variables for Color Pulse Width Measurements

int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

// 儲存紅、藍、綠色的值
int redColor = 0;
int greenColor = 0;
int blueColor = 0;

void setup() {
  // 設定S0~S4為輸出
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  
  // 設定 sensorOut 為輸出
  pinMode(sensorOut, INPUT);
  
  // 設定頻率尺度(frequency scaling)為 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
  
  // 開始串列通訊
  Serial.begin(9600);
}

void loop() {
  // 設定光電二極體讀取濾過的紅色值
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  
  // 讀取輸出頻率
  redFrequency = pulseIn(sensorOut, LOW);
  
  // 重新對應讀取的紅色值範圍從 0 到 255
  // 以下的 min 及 max 是前一個程式讀取的最大跟最小值的範圍，需依感測模組而調整 
  // redColor = map(redFrequency, min, max, 255,0);
  redColor = map(redFrequency, 332, 350, 270,0);
  
  // 列出紅色值
  Serial.print("R = ");
  Serial.print(redColor);
  delay(100);
  
  // 設定光電二極體讀取濾過的綠色值
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  
  // 讀取輸出頻率
  greenFrequency = pulseIn(sensorOut, LOW);
  
  // 重新對應讀取的綠色值範圍從 0 到 255
  // 以下的 min 及 max 是前一個程式讀取的最大跟最小值的範圍，需依感測模組而調整 
  // greenColor = map(greenFrequency, min, max, 255, 0);
  greenColor = map(greenFrequency, 850, 150, 20, 0);
  
  // 列出綠色值 
  Serial.print(" G = ");
  Serial.print(greenColor);
  delay(100);
 
  // 設定光電二極體讀取濾過的藍色值
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);


  // 檢查目前測得的顏色值，並顯示物體顏色在串列 Console
  if(redColor > greenColor){
      Serial.println(" - RED detected!");
  }
  if(greenColor > redColor){
    Serial.println(" - GREEN detected!");
  }

}