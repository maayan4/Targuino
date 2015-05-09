#include <PID_v1.h>

//DEGUG MODE
#define DEBUG      1

//Digital Pins
#define FF1            51
#define FF2            53
#define RESET       49
#define pwmHPin  46
#define pwmLPin   44 
#define dirPin         43
#define resetPin     41

#define SENSOR    21
#define ledPin         13

//Analog Pins
#define PSpin        A12
#define CSPin        A13
#define potHPin    A14
#define potLPin     A15

//Temporary variables for the loop function
boolean DIR          = 0; // 1 - clockwise, 0 - counter-clockwise
boolean STOPGO       = 0; //0- stop, 1-go

double i=0; int j=0; //for the modeling "for" loop inside loop()

//Other Constants
#define pi                                                   3.14159
#define radius                                           6.5 //[cm] radius of the wheel. represents velocity at the end point
#define ropeRadius                                  2.23 // [cm] represent velocity where the rope is
#define numOfMagnets                           2 //number of magnets available
#define VMEASURE_SAMPLE_TIME    500

//Kalman Filter default values
#define Pdef      1
#define Qdef      1E-2
#define Rdef      1E-1 //here R is actually R^2
#define Kdef      0

//Speed measurment variables
unsigned long PreviousInterruptTime = 0;
volatile unsigned int Revolutions   = 0; //global variable needs to be defined as volatile in order to be used by interrupts
unsigned int deltaT                 = 0; 
double vAngular                     = 0;
double vNoisy                         = 0;
double vKalman                      = 0;
unsigned long CurrentTime = 0;
float maxVelocity    = 3.5; //[m/s] maximum velocity. Initial value was empirically guessed

//Kalman Filter properties
float P = 0;
float Q = 0;
float R = 0; //here R is actually R^2
float K = 0;

//PID and system Variables
double measuredIn = 0; //[m/s]measured velocity
double desiredIn  = 0; // [m/s] this variable stores the desired velocity
double sysIn      = 0; //the calculated input to the driver
double Kp         = 0.0603;	
double Ki         = 3.316;
double Kd         = 0.02872;
PID myPID(&measuredIn, &sysIn, &desiredIn, Kp, Ki, Kd, DIRECT);

//Other variables - delete if unnecessary
int faultflag     = 0; //0- OK, 1-short, 2-overheat, 3-undervoltage
int sensorValue   = 0;
float current     = 0;
float CS          = 0;

void setup(){
  attachInterrupt(2,IncrRevolution,RISING); //interrupt 2 is for pin number 21 - magnet sensor input
  interrupts();
  
  Serial.begin(    9600);
  pinMode(ledPin,                  OUTPUT);
  pinMode(pwmHPin,                 OUTPUT); 
  pinMode(pwmLPin,                 OUTPUT);
  pinMode(dirPin,                  OUTPUT); 
  pinMode(FF1,                     INPUT); 
  pinMode(FF2,                     INPUT); 
  pinMode(SENSOR,                  INPUT);
  pinMode(resetPin,                OUTPUT);
  digitalWrite(resetPin,           0);
  
  //KalmanFilterReset(Pdef,Qdef,Rdef,Kdef);
  
  PreviousInterruptTime=millis();
  //MeasureMaxVelocity();
  
  myPID.SetOutputLimits(0,255);   //tell the PID to range between 0 and 255 (PWM range)    // some arbitrary value I've decided of.
  myPID.SetMode(AUTOMATIC); //turn the PID on
  myPID.SetSampleTime(200);
  
}

void loop() {
 
 //System Control
 //////////////////////////
  if (Serial.available() > 0) {
    int inByte = Serial.parseInt();
    switch(inByte){
      case 99: DIR=0; break;
      case 88: DIR=1; break;
      case 22: STOPGO=0; break;
      case 33: STOPGO=1; break;
      default: desiredIn=inByte/100; 
                    //measuredIn=inByte;
                    break; 
  }
  }

  //feedback - measure speed
  //////////////////////////////
  if(checkTime(VMEASURE_SAMPLE_TIME)){
    MeasureVelocity(VMEASURE_SAMPLE_TIME);
  }
 
  j++;
  
  if(j>100){
    desiredIn = 100;
    //KalmanFilterClear();
  }
 if(j>180){
   desiredIn =0;
  //KalmanFilterReset();
  j=0;
 } 
 
 myPID.Compute();
  movemotor(DIR,  sysIn , 255, 0); //velocity is translated into analog_output value [0-255] //*255/maxVelocity
  
  //current sensor
  sensorValue = analogRead(CSPin);  
  current = sensorValue * (5.0 / 1023.0);
  
  if (DEBUG){
    //Serial.print("T");
    Serial.print(millis());
    Serial.print(" ");
    //Serial.print("Des");
    Serial.print(desiredIn);
    Serial.print(" ");
    //Serial.print("Mea");
    Serial.print(measuredIn);
    Serial.print(" ");
    Serial.print("AV");
    Serial.print(vAngular);
    Serial.print(" ");
    Serial.print("R");
    Serial.print(Revolutions);
    Serial.print(" ");
    //Serial.print("CU");
    Serial.print(current);
    Serial.print(" ");
    //Serial.print("PS");
    Serial.print(analogRead(PSpin));
    Serial.print(" ");
    //Serial.print("DI");
    Serial.print(DIR);
    Serial.print(" ");
    //Serial.print("K");
    Serial.print(K*1E3);
    Serial.print(" ");
    //Serial.print("R");
    Serial.print(" ");
    Serial.print(R);
    Serial.print(" ");
    //Serial.print("Q");
    Serial.print(Q*1E6);
    Serial.print(" ");
    //Serial.print("MV");
    Serial.print(sysIn);
    Serial.print(" ");
    //Serial.print("vN");
    Serial.print(vNoisy);
    Serial.println("");
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

boolean movemotor(boolean DIR, float pwmH, float pwmL, boolean isRealV){
  
  if(isRealV){
    pwmH = map(pwmH, 0, maxVelocity, 0, 255);
    pwmL  = map(pwmL, 0, maxVelocity, 0, 255); 
  }
  else if(pwmH > 255 || pwmL >255){
    return 1;
  }
  
  digitalWrite(dirPin,DIR);
  analogWrite(pwmHPin, pwmH);
  analogWrite(pwmLPin, pwmL);
  
  return 0;
  
}

void IncrRevolution(){
  Revolutions++;
}

void MeasureVelocity(int deltaT){ //returns kalman filtered linear speed 
  
  /*if(Revolutions == 0){ //with sample time (deltaT) equals to 500 ms, this condition represents velocity slower than 0.14 [m/s] 
    vNoisy = 0;
  }
  else if(Revolutions == 1){
    vNoisy = vNoisy;  //keep the last value
  }
  else{*/ 
    float vAngularOld = vAngular;
    vAngular=2*pi*1000*Revolutions/(numOfMagnets*deltaT); //multiply 1/period by 1000 to get the time in seconds and the velocity in Hz
    vNoisy=vAngular*ropeRadius/100; //divide by 100 to get [m/s]   
  //}
  
  KalmanFilter();
  
  //measuredIn = vNoisy; // vKalman; //* 255 / maxVelocity;
  
  measuredIn = ( vAngular + 0.5 * abs ( vAngular - vAngularOld) ) * 255 / 157.07; //convert vAngular to PWM values
  
  Revolutions=0;
  return; 
}

void KalmanFilter(){
  P = P + Q;
  K    = P / (P + R);
  vKalman = vKalman + K * (vNoisy - vKalman);   // = (1-K)*vKalman + K*vNoisy. K is the weight of the measurement
  P    = (1 - K) * P;
  return;
}

void KalmanFilterReset(float p, float q, float r, float k){ //reset kalman filter properties to initial values 
  P = p; 
  Q = q;
  R = r;
  K = k;
  return;
}

void MeasureMaxVelocity(){
  
  movemotor(DIR, 255 , 255, 0);
  
  int nTimes = 0;
  
  while(nTimes<2){
    nTimes = nTimes + checkTime(10*VMEASURE_SAMPLE_TIME);
  }
  
  MeasureVelocity(5*VMEASURE_SAMPLE_TIME);
  maxVelocity = vKalman / nTimes;
  return;
  
}

boolean checkTime(int sampleTime){ //returns 1 if more than  SAMPLE_TIME has past
   
   CurrentTime = millis();
   deltaT = CurrentTime - PreviousInterruptTime; //current time - previous time
   
   if(deltaT>sampleTime){
   PreviousInterruptTime = CurrentTime;
   return 1;
 }
 else{
   return 0;
    }
}
  
  
