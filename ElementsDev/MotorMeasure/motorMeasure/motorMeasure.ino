#include <PID_v1.h>
#include <SPI.h>
#include <Ethernet.h>
#include <SD.h>
#include <stdio.h>
#include <string.h>

//STABLE>>>

#define BUILD_NUMBER      115

#define DEBUG             0

//Digital Pins
#define FF1pin            41 //fault flag 1
#define FF2pin            39 //fault flag 2
#define motorResetPin     49 //motor driver reset pin
#define pwmHPin           46 //motor driver PWM high pin
#define pwmLPin           44 //motor driver PWM low pin
#define dirPin            43 //motor driver direction pin
#define resetPin          30 //system reset. connected to "reset" pin on the arduino. high value resets the entire system
#define MSpin             14 //pin to determine whether the unit is in master or slave mode
#define MagnetSensorPin   19 //pin to magnet hall sensor


//Analog Pins
#define PSpin             A12 //motor driver power supply pin
#define CSPin             A13 //motor driver current sensor pin
#define RANDOM_NOISE_PIN  A3
//#define potPin            A2 //potentiometer pin (optional)

//LEDS
#define LED1        35
#define LED2        37    

//State machine
#define numOfStates         8
#define ZERO_STATE          0x01
#define WAIT_FOR_RESPONSE   0x02
#define PULL_STATE          0x03
#define PULL_AND_COUNT      0x04      
#define PULL_SLOW           0x05

//System modes
#define MS_IDLE         0xAA
#define MS_RUNNING      0xAB
#define MS_CALIBRATION  0xAC
#define SL_IDLE         0xBA
#define SL_RUNNING      0xBB
#define SL_CALIBRATION  0xBC

//PID
#define Kp                0.01255
#define Ki                0.96221
#define Kd                0.0

//Other Constants
#define pi                      3.14159
#define radius                  6.5 //[cm] radius of the wheel. represents velocity at the end point
#define ropeRadius              2.23 // [cm] represent velocity where the rope is
#define numOfMagnets            2 //number of magnets available on the motor
#define maximumMotorVelocity    25 //maximum motor velocity 157.08[rad/second] = 2*pi*1500[RPM]/60[seconds] or 3.5[m/s]=roperadius(0.0223[m])*1500[RPM]/60[sec] or 1500/60 = 25 [RPS]
#define VMEASURE_SAMPLE_TIME    500 //what is the timeframe for velocity measurement. [ms]
#define SLOW_VELOCITY           0.9 //the motor use this velocity when moveing slowly from one side to the other. [m/second]
#define MAX_PWM_VALUE           255
#define NOMINAL_TRACK_LENGTH    2500 //nominal length of the track (distance between motors) [cm]
#define SAFETY_DISTANCE         10 //safety distance from the edges in numOfmagnets
#define DELAY_BETWEEN_RUNNINGS  2000 //time to wait between runnings in [ms]
#define MAXIMUM_REQUESTED_VELOCITY 3.5 //the maximum velocity that can be requested on the webpage
#define MAXIMUM_ALLOWED_CURRENT_OUTPUT  900
#define MINIMUM_ALLOWED_CURRENT_OUTPUT  90

//XBEE
#define XBEE_DATA_RATE 9600

//XBEE messages
#define START_BYTE          0xC8
#define STOP_BYTE           0xD6
#define CLEAR_BUFFER        0xDD

//Master->slave
#define GOTO_CALIBRATION    0x6B
#define GOTO_RUN            0x61
#define GOTO_IDLE           0x62
#define SWITCH_TO_COAST     0x63
#define KEEP_ALIVE          0x64
#define LTR_VELOCITY_UPDATE 0x65
#define LTR_DELAY_UPDATE    0x66
#define PULL_SLOWLY         0x67
#define STOP_MOTOR          0x68
#define START_PULLING       0x69
#define TRACK_LENGTH_UPDATE 0x6A

//Slave->master
#define ACK_CALIB           0x70
#define ACK_RUN             0x71
#define ACK_IDLE            0x72
#define ACK_COAST           0x73
#define DONE_PULLING        0x74
#define ACK_KA              0x75
#define ACK_VEL             0x76
#define ACK_PULL            0x77
#define ACK_STOP            0x78
#define ACK_DELAY           0x79
#define ACK_TRACK           0x7A

//HTTP messages parsing cases:
#define UPDATE_VELOCITY_RTL       0x50
#define UPDATE_VELOCITY_LTR       0x51
#define UPDATE_BEGIN_CALIBRATION  0x52
#define UPDATE_BEGIN_RUNNING      0x53
#define UPDATE_STOP               0x54
#define UPDATE_STATUS             0x55
#define UPDATE_DELAY_RTL          0x5B
#define UPDATE_DELAY_LTR          0x5C
#define OPEN_FILE 			       	  0x56

//WhatToSend states - these flags set the message that will be sent next on idle mode
#define SEND_KA                   0xF1
#define SEND_DELAY                0xF2
#define SEND_VELOCITY             0xF3

//HTTP message strings
#define MSG_RTL_VEL         "RTL_vel="
#define MSG_LTR_VEL         "LTR_vel="
#define MSG_RTL_DELAY       "RTL_delay="
#define MSG_LTR_DELAY       "LTR_delay="
#define MSG_CAL             "BeginCalibration"
#define MSG_RUN             "BeginRunning"
#define MSG_STOP            "Stop"
#define MSG_STATUS          "status"

//Ethernet
#define REQ_BUF_SZ            60 // size of buffer used to capture HTTP requests
#define WEBSITE_FILENAME      "targuino.htm" //name of the main html file on the SD card
#define JAVASCRIPT_FILENAME   "targuino.js" //javascript filename
#define JS_UISLIDER_FILENAME  "nuslider.js"
#define CSS_UISLIDER_FILENAME "nuslider.css"
#define JQUERY_FILENAME       "jquery.js"
#define CSS_FILENAME          "targuino.css" //css filename
#define JPG_FILENAME          "targuino.jpg" //jpg filename
#define XML_FILENAME          "targuino.xml"
#define ICO_FILENAME          "favicon.png"

//Watchdogs and timers
#define DELAY_BETWEEN_KA          3000 //time between keep-alive messages
#define KA_WATCHDOG               3 //maximum keepalive messages to wait for SL response before error message appears
#define MAXIMUM_NUM_OF_RESENDS    3 //timeout for a response from the other side
#define RESEND_TIMOUT             5000 //timout to resend message
#define FINITE_WAITING_TIMEOUT     40000 //finite timeout. after this time the system goes back to idle

//Motor State description variables
boolean DIR                         = 1; // 1 - clockwise, 0 - counter-clockwise
volatile unsigned int Revolutions   = 0; //number of revolutions. global variable needs to be defined as volatile in order to be used by interrupts
volatile unsigned int RunLength     = 0; //this variable counts revolutions during a run and compare it to lTrack [Revolutions]. It also delivers the length to lTrack during calibration
unsigned int lTrack                 = 0; //stores the length of the track (= distance between motors). needs to be written to the EEPROM in the future
float vAngular                      = 0;
float vLinear                       = 0;
double measuredIn                   = 0; //[m/s]measured velocity
float K                             = 0.5; //averaging constant between very different velocity measurements
float kk                            = 0;
int distance_from_edge              = 0; //this variable contains the calculated distance from the opposite edge 

//Timer and watchdog variables
unsigned long PreviousInterruptTime       = 0;
unsigned long CurrentTime                 = 0;
unsigned int deltaT                       = 0;
unsigned int tBeginMovement               = 0; 
unsigned int timeGuard                    = 0;
unsigned int maximumRunTime               = 0; //this is them maximum time the motor can run before watchdog is up
unsigned long PrevMsgSent                 = 0; //stores the last time KEEP-ALIVE msg was sent
int numOfKAmsgs                           = 0; //stores the number of keealive msgs that were sent
int numOfresends                          = 0; //stores the number of times a message was resent
int numOfCycUnknown                       = 0; //stores the number of times slave got unknown messages
int nDroppedMsgs                          = 0;
unsigned long PreviousTimeForGeneralTasks = 0; //this is a timer in loop() for general tasks
boolean hasStopBeenClickedOnce            = false;

/* PID Variables*/
double desiredIn  = 0; // this variable stores the desired velocity in PWM range (0-255)
double sysIn      = 0; //the calculated input to the driver
PID myPID(&measuredIn, &sysIn, &desiredIn, Kp, Ki, Kd, DIRECT);

//system state variables
boolean MS              = 0; //is arduino in Master or Slave mode (0 for slave mode, 1 for master)
byte FSM_State          = ZERO_STATE; //system's currect state in the state machine. default value is ZERO_STATE - initial state (see #define)
byte sysMode            = 0; //this byte indicate two thing: 1) is this motor MS/SL   2) is the system in idle, calibration or run mode
float velLTR            = 0.0; //the velocity the user has requested for the run from left to right
float velRTL            = 0.0; //the velocity the user has requested for the run from right to left
String TarMsg           = " "; //string that contains system status
boolean stopSignal      = false; //a general flag which uses to control whether the system goes to the next stage
int LTR_delay           = 0; //delay before slave starts to pull. in [sec]
int RTL_delay           = 0; //delay before master starts to pull. in [sec]
boolean delayHasPassed  = false; //a flag states whether the delay time has passed
boolean isCloseToEdge   = false;

//Ethernet
byte mac[] 										    	   = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress 				 				  			       ip(192, 168, 0, 177);
EthernetServer   			  				   			   server(80);
char HTTP_req[REQ_BUF_SZ] 						    	   = {0};  	// 0 = null in ascii
char req_index   			 			  			       = 0;       // index into HTTP_req buffer
typedef struct parsedResult {byte msgType; float value; String fileToOpen; char httpResponse;} parsedResult;

//XBEE variables
byte msgToSend[10]              = {START_BYTE, 0, 0, 0, 0, 0, 0, 0, 0, STOP_BYTE};
byte recBuff[7]                = {0, 0, 0, 0, 0, 0, 0};
byte previousTxMsg              = 0xFF; 
byte previousRxMsg             = 0;
String XBEEstatusMsg           = " ";
byte MsgCounter                = 0; //counter to be added to the messages
byte msgIndex                  = 0;
boolean isSlaveConnected       = 0;
byte whatToSend                = 0xF1;   // a flag states what should the master send to the slave: KA, delay or velocity
byte inMsgCount                = 0;
byte MsgCounterAtDest          = 0; //stores MsgCounter as it is on the other side 
byte tempByte                  = 0;
boolean isMsg                  = false;
byte msgIndDiff                = 0; //distance between message indices

//SD card
File TServerFile;
File xmlFile;

//Other variables - delete if unnecessary
int faultflag           = 0; //0- OK, 1-short, 2-overheat, 3-undervoltage
int sensorValue         = 0;
float current           = 0;
float CS                = 0;
boolean OneTimeFlag   = true;
int tempAcc             = 0;
boolean M              = false;
float vel = 0;
float delta = 0;
byte desiredRev = 0;
float velocity = 1;
bool safe = true; 

//=====================SETUP=============================================
void setup(){
  // disable Ethernet chip
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  
  //initialize XBEE
  Serial.begin(             XBEE_DATA_RATE); 
  Serial2.begin(             XBEE_DATA_RATE); //for debug purposesuffer

  Serial2.print("Targuino version 0.8 build "); Serial2.println(BUILD_NUMBER);
  


  //initialize digital pins. Analog pins don't require initialize
  pinMode(pwmHPin,          OUTPUT); 
  pinMode(pwmLPin,          OUTPUT);
  pinMode(dirPin,           OUTPUT); 
  pinMode(FF1pin,           INPUT); 
  pinMode(FF2pin,           INPUT); 
  pinMode(motorResetPin,    INPUT); 
  pinMode(MagnetSensorPin,  INPUT);
  pinMode(resetPin,         OUTPUT);
  pinMode(MSpin,            INPUT);

  //init leds
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  //initialize reset pin
  digitalWrite(resetPin,           HIGH); //switching to LOW will reset the arduino

  Serial2.println("Pins initialized");

  //CHECK MASTER/SLAVE MODE
  if(digitalRead(MSpin) ==  HIGH){  
    MS = 1;
    Serial2.println("This is the master motor");
    
  }
  else {
    sysMode = SL_IDLE;
    Serial2.println("This is the slave motor");
  }
  
  myPID.SetOutputLimits(0,255);  //tell the PID to range between 0 and 255 (PWM range)    // some arbitrary value I've decided of. 255 is too much?
  myPID.SetMode(    AUTOMATIC); //turn the PID on
  myPID.SetSampleTime(    VMEASURE_SAMPLE_TIME);

  maximumRunTime = float(NOMINAL_TRACK_LENGTH / SLOW_VELOCITY) * 1000;
  
  Serial2.print("Maximum runtime is [ms]: "); Serial2.println(maximumRunTime);

  //setup interrupt for the magnet sensor
  attachInterrupt(4,IncrRevolution,RISING); //interrupt 4 is for pin number 19 - magnet sensor input

  PreviousInterruptTime = millis(); //start timer

  randomSeed(analogRead(RANDOM_NOISE_PIN));

}

//=====================MAIN=============================================
void loop(){



  //Do some general tasks
  CurrentTime = millis(); //main timer for all system purposes
  
  if(CurrentTime - PreviousTimeForGeneralTasks > 20000){ 
    PreviousTimeForGeneralTasks = CurrentTime;
    M = !M ;
    safe = true;
  }

  if(M){
    vel = 0.4;
    desiredIn = (double) convertMagnetsToPWM(convertVelToMagnets(vel));
  }
  else{ 
    coastMotor();
    desiredIn = 0;
    sysIn = 0;
    myPID.Compute();
  }
  
  //current sensor
  sensorValue = analogRead(CSPin);  
  //current = ( map(sensorValue, 0, 1023, 0, 5) - 2.5) / 0.066;

  if(checkTime(VMEASURE_SAMPLE_TIME)){  
    MeasureVelocityInPWM(VMEASURE_SAMPLE_TIME);
    
    myPID.Compute();
    //  sysIn = desiredIn;
      
      if(safe){
      movemotor(DIR,sysIn,MAX_PWM_VALUE);
    }
    else{
      Serial2.println("SAFETY MECHANISM HAS BEEN TURNED ON");
    }
      SerialProcess();
    
    }



}//end loop

//=====================FUNCTIONS=============================================

void SerialProcess(){

  Serial2.print(float(CurrentTime / 1000));

  //Serial2.print(" In: ");
  /*Serial2.print(" ");
  Serial2.print(desiredIn);

  //Serial2.print(" vAngular: ");
  Serial2.print(" ");
  Serial2.print(vAngular);

  //Serial2.print(" vLinear ");
  Serial2.print(" ");
  Serial2.print(vLinear);
*/
  Serial2.print(" ");
  //Serial2.print(" sysIn: ");
   Serial2.print(sysIn);   /*

  //Serial2.print(" measuredIn: ");
  Serial2.print(" ");
  Serial2.print(measuredIn);*/ 

  //Serial2.print(" Sensor: ");
  Serial2.print(" ");
  Serial2.print(sensorValue); 

  //Serial2.print(" current: ");
  //Serial2.print(" ");
  //Serial2.print(current);

  Serial2.println();
}

boolean movemotor(boolean DIR, float pwmH, float pwmL){ //set motor velocity and direction
  
  if(pwmH > 255 || pwmL > 255){  
    pwmH = 0;
    return 1;
    } //illigal values
  
  byte pwmHround = mround(pwmH);
  byte pwmLround = mround(pwmL);
  
  //Serial2.print(pwmHround); Serial2.print(" "); Serial2.println(pwmHround); 

  digitalWrite(dirPin,DIR); //maybe *255?
  analogWrite(pwmHPin, pwmHround);
  analogWrite(pwmLPin, pwmLround);

  //check current sensor - safety check
  sensorValue = analogRead(CSPin);  

  if(sensorValue > MAXIMUM_ALLOWED_CURRENT_OUTPUT || sensorValue < MINIMUM_ALLOWED_CURRENT_OUTPUT){
    coastMotor();
    safe = false;
    FSM_State = ZERO_STATE;
      OneTimeFlag = true;
      if(MS){ 
        sysMode = MS_IDLE;
        //sendXBEEmsg(STOP_MOTOR);
      }
      else{ sysMode = SL_IDLE;}
    
    Serial2.print(float(CurrentTime / 1000)); Serial2.print(" SAFETY WARNING: Stopping due to sensor value = "); Serial2.println(sensorValue);
    return 1;
  }
  return 0;
}

void IncrRevolution(){ //ISR function - measure revolutions

  Revolutions++;
  RunLength++;

}

void MeasureVelocityInPWM(int deltaT){ 

  measuredIn = (double) convertMagnetsToPWM(Revolutions);

  Revolutions = 0;

  return; 
}

boolean checkTime(int sampleTime){ //returns 1 if more than  SAMPLE_TIME has past

 CurrentTime = millis();
   deltaT = CurrentTime - PreviousInterruptTime; //current time - previous time

   if(deltaT > sampleTime){
    PreviousInterruptTime = CurrentTime;
    return 1;
  }
  else{
   return 0;
 }
}

void stopMotor(){
  movemotor(DIR,0,MAX_PWM_VALUE);
}


boolean areVelocitiesLegal(){ //check wether the values for the velocities are correct
  if(velRTL > MAXIMUM_REQUESTED_VELOCITY || velLTR > MAXIMUM_REQUESTED_VELOCITY){ return 1;}
  else if(velRTL < 0.1 || velLTR < 0.1){ return 1;}
  else{ return 0;}
}

////////NEW
byte convertVelToMagnets(float inVel){

  float preciseCountedMagnets =  inVel * numOfMagnets * VMEASURE_SAMPLE_TIME * 100 / ( 2 * pi * ropeRadius * 1000 );
  
  return mround(preciseCountedMagnets);
}

byte convertMagnetsToPWM(byte inMagnets){

  float PWMout = float(inMagnets) * 255 * 1000 / ( VMEASURE_SAMPLE_TIME * numOfMagnets * maximumMotorVelocity);

  return mround(PWMout); 
}

byte convertVelToPWM(float vel){

  float precisePWMvalue = vel * 255 * 100 / ( maximumMotorVelocity * 2 * pi * ropeRadius );

  return mround(precisePWMvalue);
}

byte mround(float InValue){ //round a float number to the nearest integer by the first decimal.

  int integerPart = int(InValue);
  float remainPart = InValue - integerPart;

  if(remainPart > 0.5){ 
    return (byte) integerPart + 1 ;
  }
  else{
    return (byte) integerPart;
  }

}

void coastMotor(){
  digitalWrite(pwmLPin,0);
  digitalWrite(pwmHPin,0);
}

void resetArduino(){
  digitalWrite(resetPin, LOW);
}
