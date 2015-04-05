//DEGUG MODE
const boolean DEBUG = 1;

//Digital Pins
const int FF1 = 51;
const int FF2 = 53;
const int RESET=49;
const int pwmHPin = 46;
const int pwmLPin = 20; 
const int dirPin =43 ;

const int MSpin = 14; //pin to determine whether the unit is in master or slave mode
boolean MS=0; //default status 0 - arduino in slave mode (1 for master)
const int SENSOR=21;
const int ledPin=13;
const int BigLed=52;

//Analog Pins
const int PSpin=A12;
const int CSPin = A13;
const int potHPin = A14;
const int potLPin = A15;




void setup() {
  Serial.begin(9600);
  pinMode(ledPin,OUTPUT);
  pinMode(pwmHPin, OUTPUT); 
  pinMode(pwmLPin, OUTPUT);
  pinMode(dirPin, OUTPUT); 
  pinMode(FF1, INPUT); 
  pinMode(FF2, INPUT); 
  pinMode(SENSOR,INPUT);
  pinMode(MSpin,INPUT);
  
  pinMode(BigLed,OUTPUT);
 // digitalWrite(BigLed,1);
  
  //CHECK MASTER/SLAVE MODE
  if(digitalRead(MSpin)==HIGH){MS=1;}
}

void loop() {
if(MS==0){ //SLAVE MODE
  if (Serial.available() > 0) {
    int inByte = Serial.parseInt();
    digitalWrite(BigLed,inByte);
  }
  }
  if(MS==1){//MASTER MODE
    digitalWrite(BigLed,0);
    Serial.println(0);
    delay(2000);
    digitalWrite(BigLed,1);
    Serial.println(1);
    delay(2000);
  }
}

