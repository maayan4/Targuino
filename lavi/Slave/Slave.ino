////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int mainmotorspeed = 75;           // speed of the mastermotor!!!!!!!!!!!!!!  <<< change here!!!!!!!!!!!!1
int brakemoterspeed = 50;            // intensity of braking
int calibrationbreakspeed = 30;      // intensity of calibration main motor breaking
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


char startcommand = 'Y';             // send to slave to start motor

char stopcommand = 'N';              // send to slave to stop motor

char acnowledgecommand = 'R';        // send and receive for acnowledge

char calibrationbreak = 'b';         // send calibration break

char normalbreak = 'B';              // send normal operation break


int tripcurrent = 650;

int MotorPWMPin=9;                                      // pwm fo engine
int OverCurrentCount=5;                                                    //over current number>3
int resetpin=22;                                  //modul for the driver
int ff1=26;                                               //LOW	Fault flag 1 indicator: FF1 goes high when certain faults have occurred. See table below for details.
int ff2=28;                                               //LOW	Fault flag 2 indicator: FF2 goes high when certain faults have occurred. See table below for details.
int startButton=48;                                       //start button

int MotorSpeed = 70; //between 0-255;
int overcurrentprotectiondelay = 500; // in milliseconds delay to block overcurrent while accelerating

char incomingByte;                    //Communication

int Moving = 0 ; // 0 = stop, 1 = moving
int AtBase = 0 ; // 0 = not at base, 1 = at base;

void setup() {
  pinMode(ff1, INPUT); 
  pinMode(ff2, INPUT);
  pinMode(resetpin, OUTPUT);
  pinMode(startButton, INPUT_PULLUP);   
  pinMode(MotorPWMPin, OUTPUT);

  Serial.begin(57600);

  digitalWrite(resetpin, HIGH);
  analogWrite(MotorPWMPin,0);

  while(Serial.available())
    Serial.read();
}

void loop() 
{

  if (Serial.available() > 0) 
  {
    incomingByte = Serial.read();
  }
  else
  {
    incomingByte == 0;
  }
  if(incomingByte == 'Y')
  {
    analogWrite(MotorPWMPin, mainmotorspeed);
  }
  if(incomingByte == 'N')
  {
    analogWrite(MotorPWMPin, 0);
  }
  if(incomingByte == 'b')
  {
    analogWrite(MotorPWMPin, calibrationbreakspeed);
  }
  if(incomingByte == 'B')
  {
    analogWrite(MotorPWMPin, brakemoterspeed);
  }



}




















