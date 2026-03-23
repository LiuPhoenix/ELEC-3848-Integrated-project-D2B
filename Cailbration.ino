#define S0 4 //PG5
#define S1 6 //PH3
#define S2 7 //PH4
#define S3 10 //PB4
#define sensorOut 11 //PB5


// 儲存由光電二極體讀取的值
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

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
  
   // 列出紅色值
  Serial.print("R = ");
  Serial.print(redFrequency);
  delay(100);
  
  // 設定光電二極體讀取濾過的綠色值
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  
  // 讀取輸出頻率
  greenFrequency = pulseIn(sensorOut, LOW);
  
  // 列出綠色值 
  Serial.print(" G = ");
  Serial.print(greenFrequency);
  delay(100);
 
  // 設定光電二極體讀取濾過的藍色值
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  
  // 讀取輸出頻率
  blueFrequency = pulseIn(sensorOut, LOW);
  
  // 列出藍色值，並換行
  Serial.print(" B = ");
  Serial.println(blueFrequency);
  delay(100);
}