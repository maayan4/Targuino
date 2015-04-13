//DEGUG MODE
const boolean DEBUG = 1;

//Pins
#define ledPin A2
#define pot A0
#define MSpin 14 //pin to determine whether the unit is in master or slave mode

boolean MS=0; //default status 0 - arduino in slave mode (1 for master)
int brightness = 0;    // how bright the LED is
int fadeAmount = 3;    // how many points to fade the LED by
int time = 1000;


void setup() {
  Serial.begin(57600);
  Serial2.begin(9600);
  pinMode(ledPin,OUTPUT);
  pinMode(MSpin,INPUT);
  //CHECK MASTER/SLAVE MODE
  if(digitalRead(MSpin)==HIGH){MS=1;}
}

void loop() {
if(MS==0){ //SLAVE MODE
  if (Serial.available() > 0) {
   time = Serial.parseInt();
   Serial2.println(time);
  }
}
Serial2.println(Serial.available());

if(MS == 1){
  Serial.println(analogRead(pot));
  Serial2.println(analogRead(pot));
}

  analogWrite(ledPin, brightness);

  // change the brightness for next time through the loop:
  brightness = 255 - brightness;

  delay(time);
}


