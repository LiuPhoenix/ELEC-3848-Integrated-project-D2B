#define echoPin1 13 //PB7
#define trigPin1 46 //PL3
#define echoPin2 45 //PL4
#define trigPin2 44 //PL5
#define echoPin3 29 //PA7
#define trigPin3 30 //PC7
#define echoPin4 32 //PC5
#define trigPin4 33 //PC4

long distance_in_cm;

void setup() {
  Serial.begin (9600);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin1, OUTPUT);
}

void loop() {
  measure_distance();
  Serial.print(distance_in_cm);
  Serial.println("");
  delay(1000);
}

void measure_distance() {
  long duration;
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
 
  digitalWrite(trigPin1, LOW); 
  delayMicroseconds(2); 
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin1, LOW);
 
  duration = pulseIn(echoPin1, HIGH);
  distance_in_cm = (duration/2.0) / 29.1;
}

