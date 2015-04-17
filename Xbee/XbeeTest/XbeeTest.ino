//DEGUG MODE
const boolean DEBUG = 1;

//Pins
#define ledPin 19
#define pot A2
#define MSpin 14 //pin to determine whether the unit is in master or slave mode

boolean MS=0; //default status 0 - arduino in slave mode (1 for master)
int brightness = 0;    // how bright the LED is
int fadeAmount = 3;    // how many points to fade the LED by
int DS = 150;

//XBEE
#define XBEE_DATA_RATE 57600
String strIn(100);

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
   DS = Serial.parseInt()*255/1023;
   Serial2.println(DS);
  }
}
Serial2.println(Serial.available());

if(MS == 1){
  Serial.println(analogRead(pot));
  Serial2.println(analogRead(pot));
}

  digitalWrite(ledPin, DS);

  // change the brightness for next time through the loop:

}

boolean beginXbee(){
  Serial.begin(XBEE_DATA_RATE);
  Serial.print("+++");
  if(Serial.available
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    strIn += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n\r') {
      stringComplete = true;
    }
  }
}
 
 
