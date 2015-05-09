
boolean DEBUG = 1;

int potHPin = A0;    // select the input pin for the potentiometer
int potLPin = A12;
int pwmHPin = 8;      // select the pin for the LED
int pwmLPin = 9; 
int potHValue = 0;  // variable to store the value coming from the sensor
int potLValue = 0;
int dirPin = 5;

int pwmHValue = 0;
int pwmLValue = 0;

boolean DIR=0;//0-forward, 1-backward
boolean STOPGO=0; //0- stop, 1-go

int CSPin = A1;
int sensorValue = 0;
int current = 0;

int FF1 = 2;
int FF2 = 3;
int faultflag = 0; //0- OK, 1-short, 2-overheat, 3-undervoltage

int inByte = 0;         // incoming serial byte

void setup() {
  Serial.begin(9600);
  // declare the ledPin as an OUTPUT:
  pinMode(pwmHPin, OUTPUT); 
  pinMode(pwmLPin, OUTPUT);
  pinMode(dirPin, OUTPUT); 
  pinMode(FF1, INPUT); 
  pinMode(FF2, INPUT); 
  attachInterrupt(0, checkfault, HIGH);
  attachInterrupt(1, checkfault, HIGH);
}

void loop() {
  if (Serial.available() > 0) {
    inByte = Serial.parseInt();
    Serial.print("sent byte: ");
    Serial.println(inByte);
    if (inByte==1) DIR=1;
    else if (inByte==0) DIR=0;
    else if (inByte==2) STOPGO=0;
    else if (inByte==3) STOPGO=1;
  }
  // read the value from the pot and update PWM
  potHValue = analogRead(potHPin);
  potLValue = analogRead(potLPin);
  pwmHValue = map(potHValue, 0, 1023, 0, 254);
  pwmLValue = map(potLValue, 0, 1023, 0, 254);
  float pwmHPer = pwmHValue * (100.0/254.0);
  float pwmLPer = pwmLValue * (100.0/254.0);
   
  movemotor(DIR, pwmHValue,pwmLValue);
  //current sensor
  sensorValue = analogRead(CSPin);  
  //current = sensorValue * (5.0 / 1023.0);
  //if (sensorValue < 512) current = -current;
  current = map(sensorValue, 0, 1023, -3000, 3000);
  if (DEBUG){
    Serial.print(" PWMH: ");
    Serial.print(pwmHPer);
    Serial.print(" PWML: ");
    Serial.print(pwmLPer);
    Serial.print(" % current: ");
    Serial.print(current);
    Serial.print(" : ");
    Serial.print(digitalRead(FF1));
    Serial.print(" ");
    Serial.print(digitalRead(FF2));
    //Serial.print(" mA ");
    Serial.print(" Direction ");
    Serial.println(DIR);
 }
  if (faultflag){//0- OK, 1-short, 2-overheat, 3-undervoltage
   switch (faultflag){
     case 1:
     Serial.println("short circuit fault - motor disconnected - needs RESET");
     break;
     case 2:
     Serial.println("overheat fault");
     break;
     case 3:
     Serial.println("undervoltage fault - motor disconnected");
     break;
  }
 }
}
void checkfault(){
  if (digitalRead(FF1) && digitalRead(FF2)) faultflag=3;
  else if (!digitalRead(FF1) && digitalRead(FF2)) faultflag=2;
  else if (digitalRead(FF1) && !digitalRead(FF2)) faultflag=1;
  else if (!digitalRead(FF1) && !digitalRead(FF2)) faultflag=0;
}

void movemotor(boolean DIR, int pwmH, int pwmL){
  analogWrite(pwmHPin, pwmH);
  analogWrite(pwmLPin, pwmL);
  digitalWrite(dirPin,DIR);
  
}
