//DEGUG MODE
const boolean DEBUG = 1;

//Digital Pins
const int FF1 = 51;
const int FF2 = 53;
const int RESET=49;
const int pwmHPin = 46;
const int pwmLPin = 44; 
const int dirPin =43 ;

const int SENSOR=21;
const int ledPin=13;

//Analog Pins
const int PSpin=A12;
const int CSPin = A13;
const int potHPin = A14;
const int potLPin = A15;

//Temporary variables for the loop function
int potHValue = 0;  // variable to store the value coming from the sensor
int potLValue = 1023;
int pwmHValue = 60;
int pwmLValue = 0;
boolean DIR=0; // 1 - clockwise, 0 - counter-clockwise
boolean STOPGO=0; //0- stop, 1-go

int i=0; int j=0; //for the modeling "for" loop inside loop()

//Other Constants
const double pi = 3.14159;
const double radius = 6.5; //[cm] radius of the wheel. represents velocity at the end point
const double ropeRadius=2.23; // [cm] represent velocity where the rope is
const int numOfMagnets=2; //number of magnets available
const double maxV = 4;

//Kalman Filter properties
double vKalman = 0; //initial guess
//double XhatMinus = 0;
double P = 1; //initial guess
//double PMinus = 0;
double Q = 1*10^(-6);
double R = 0.01;
double K = 0;

//Speed measurment variables
unsigned long PreviousInterruptTime=0;
volatile unsigned int Revolutions=0; //global variable needs to be defined as volatile in order to be used by interrupts
unsigned int deltaT=0; 
float vAngular=0;
float vLinear=0;

//Other variables - delete if unnecessary
int faultflag = 0; //0- OK, 1-short, 2-overheat, 3-undervoltage
int sensorValue = 0;
float current = 0;
float CS=0;

void setup() {
  attachInterrupt(2,IncrRevolution,RISING); //interrupt 2 is for pin number 21
  interrupts();
  PreviousInterruptTime=millis();
  Serial.begin(9600);
  pinMode(ledPin,OUTPUT);
  pinMode(pwmHPin, OUTPUT); 
  pinMode(pwmLPin, OUTPUT);
  pinMode(dirPin, OUTPUT); 
  pinMode(FF1, INPUT); 
  pinMode(FF2, INPUT); 
  pinMode(SENSOR,INPUT);
}

void loop() {
  if (Serial.available() > 0) {
    int inByte = Serial.parseInt();
    //DEBUG - send message back
    //Serial.print("sent byte: ");
    //Serial.println(inByte);
    switch(inByte){
      case 9: DIR=0; break;
      case 8: DIR=1; break;
      case 2: STOPGO=0; break;
      case 3: STOPGO=1; break;
      default: pwmHValue=float(inByte/100); break; 
  }
  }
  
  //                                                      _______
  //------input---(+-)--(input-PID)->|motor |----------->output
  //                        |                              ---------         |
  //                        |         ____                                  |
  //                        -------|PID|---------------------------
  //                                  -----
  //first read the feedback (speed measurment) and then decide on the input
  
  //feedback - measure speed
 if(Revolutions>1){
    MeasureVelocity();
}
  
  // read the value from the potentiometer and update PWM
  //potHValue = analogRead(potHPin);
  //potLValue = analogRead(potLPin);
  
  pwmLValue=254;
  pwmHValue=i;
  j++;
  if(j>20){
    i++;
    j=0;
  }
 if(i>254){
  i=0;
 } 
  
  //pwmHValue = map(potHValue, 0, 1023, 0, 254); //just to convert potentiometer input range [0-1023] to PWM range [0-254]  
  
  float pwmHPer = pwmHValue * (100.0/254.0);
  float pwmLPer = pwmLValue * (100.0/254.0);
   
  movemotor(DIR, pwmHValue,pwmLValue);
  
  //current sensor
  sensorValue = analogRead(CSPin);  
  current = sensorValue * (5.0 / 1023.0);
  //if (sensorValue < 512) current = -current;
 
   //current = map(sensorValue, 0, 1023, -3000, 3000);
  
  if (DEBUG){
    //Serial.print("T");
    Serial.print(millis());
    Serial.print(" ");
    //Serial.print("PH");
    Serial.print(pwmHValue*maxV/254);
    Serial.print(" ");
    //Serial.print("PL");
    Serial.print(pwmLPer);
    Serial.print(" ");
    //Serial.print("R");
    Serial.print(Revolutions);
    Serial.print(" ");
    //Serial.print("DT");
    Serial.print(deltaT);
    Serial.print(" ");
    //Serial.print("LV");
    Serial.print(vLinear);
    Serial.print(" ");
    //Serial.print("AV");
    Serial.print(vKalman);
    Serial.print(" ");
    //Serial.print("CU");
    Serial.print(current);
    Serial.print(" ");
    //Serial.print("FA");
    Serial.print(digitalRead(FF1));
    Serial.print(" ");
    //Serial.print("FB");
    Serial.print(digitalRead(FF2));
    Serial.print(" ");
    //Serial.print("PS");
    Serial.print(analogRead(PSpin));
    //Serial.print(" ");
    //Serial.print("DI");
    //Serial.print(DIR);
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

void movemotor(boolean DIR, int pwmH, int pwmL){
  analogWrite(pwmHPin, pwmH);
  analogWrite(pwmLPin, pwmL);
  digitalWrite(dirPin,DIR);
  
}

void IncrRevolution(){
  Revolutions++;
}

void MeasureVelocity(){ //returns angular speed omega
  
  unsigned long CurrentTime=millis();
  
  deltaT=CurrentTime-PreviousInterruptTime; 
  PreviousInterruptTime=CurrentTime;
  
//  if (Revolutions == 0){ // check slow speed case or an entire revolution was not completed yet
//    if(pwmHValue==0 || pwmLValue==0){
//    return 0;
//    }
//    if(deltaT>1000){ // 800ms equals to speed lower than 0.5[m/s]
//    return 999; //speed is too slow for the magnetic sensor to catch it. assuming there is only one magnet on the wheel
//    }
//    else{
//    return 990; //return some arbitrary number 
//  }
//}
  
  vAngular=2*pi*1000*Revolutions/(numOfMagnets*deltaT);
  vLinear=vAngular*ropeRadius/100; //divide by 100 to get [m/s]
  
  KalmanFilter();
 
  Revolutions=0;
  return; //multiply 1/period by 1000 to get the time in seconds and the velocity in Hz
}
  
void KalmanFilter(){
  P = P + Q;
  K    = P / (P + R);
  vKalman = vKalman + K * (vLinear - vKalman);   
  P    = (1 - K) * P;
}
  
  
