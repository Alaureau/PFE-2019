const int SIG = 2;
const int s0 = 6;
const int s1 = 5;
const int s2 = 4;
const int s3 = 3;

void setup() {
  // put your setup code here, to run once:
  pinMode(SIG, INPUT);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  //digitalWrite(SIG, LOW);
  Serial.begin(9600);

}

void loop() {

//sensor1

  long duration1, distance1;
  digitalWrite(s0, HIGH);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  delayMicroseconds(2);
  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  delayMicroseconds(10); 
  digitalWrite(s0, HIGH);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);

  duration1 = pulseIn(SIG, HIGH);
  distance1 = (duration1/2) / 29.1; 
  
  Serial.print("1_");
  Serial.println(distance1);

  delay(500);

 //sensor 2
  
  long duration2, distance2;
  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  delayMicroseconds(2);
  digitalWrite(s0, LOW);
  digitalWrite(s1, HIGH);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  delayMicroseconds(10); 
  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);

  duration2 = pulseIn(SIG, HIGH);
  distance2 = (duration2/2) / 29.1; 
  
  Serial.print("2_");
  Serial.println(distance2);
  
  
  delay(500);

 //sensor3

 long duration3, distance3;
  digitalWrite(s0, HIGH);
  digitalWrite(s1, LOW);
  digitalWrite(s2, HIGH);
  digitalWrite(s3, LOW);
  delayMicroseconds(2);
  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, HIGH);
  digitalWrite(s3, LOW);
  delayMicroseconds(10); 
  digitalWrite(s0, HIGH);
  digitalWrite(s1, LOW);
  digitalWrite(s2, HIGH);
  digitalWrite(s3, LOW);

  duration3 = pulseIn(SIG, HIGH);
  distance3 = (duration3/2) / 29.1; 
  
  Serial.print("3_");
  Serial.println(distance3);

  delay(500);

  //sensor 4
  
  long duration4, distance4;
  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);
  digitalWrite(s2, HIGH);
  digitalWrite(s3, LOW);
  delayMicroseconds(2);
  digitalWrite(s0, LOW);
  digitalWrite(s1, HIGH);
  digitalWrite(s2, HIGH);
  digitalWrite(s3, LOW);
  delayMicroseconds(10); 
  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);
  digitalWrite(s2, HIGH);
  digitalWrite(s3, LOW);

  duration4 = pulseIn(SIG, HIGH);
  distance4 = (duration4/2) / 29.1; 
  
  Serial.print("4_");
  Serial.println(distance4);
  
  
  delay(500);

  //sensor 5
  
  long duration5, distance5;
  digitalWrite(s0, HIGH);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH);
  delayMicroseconds(2);
  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH);
  delayMicroseconds(10); 
  digitalWrite(s0, HIGH);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH);

  duration5 = pulseIn(SIG, HIGH);
  distance5 = (duration5/2) / 29.1; 
  
  Serial.print("5_");
  Serial.println(distance5);
  
  
  delay(500);

  //sensor 6
  
  long duration6, distance6;
  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);
  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH);
  delayMicroseconds(2);
  digitalWrite(s0, LOW);
  digitalWrite(s1, HIGH);
  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH);
  delayMicroseconds(10); 
  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);
  digitalWrite(s2, LOW);
  digitalWrite(s3, HIGH);

  duration6 = pulseIn(SIG, HIGH);
  distance6 = (duration6/2) / 29.1; 
  
  Serial.print("6_");
  Serial.println(distance6);
  
  
  delay(500);

  //sensor 7
  
  long duration7, distance7;
  digitalWrite(s0, HIGH);
  digitalWrite(s1, LOW);
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);
  delayMicroseconds(2);
  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);
  delayMicroseconds(10); 
  digitalWrite(s0, HIGH);
  digitalWrite(s1, LOW);
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);

  duration7 = pulseIn(SIG, HIGH);
  distance7 = (duration7/2) / 29.1; 
  
  Serial.print("7_");
  Serial.println(distance7);
  
  
  delay(500);

  //sensor 8
  
  long duration8, distance8;
  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);
  delayMicroseconds(2);
  digitalWrite(s0, LOW);
  digitalWrite(s1, HIGH);
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);
  delayMicroseconds(10); 
  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);
  digitalWrite(s2, HIGH);
  digitalWrite(s3, HIGH);

  duration8 = pulseIn(SIG, HIGH);
  distance8 = (duration8/2) / 29.1; 
  
  Serial.print("8_");
  Serial.println(distance8);
    
  delay(500);
  
}
