#include <PID_v1.h>
#include <SPI.h>
#include <Ethernet.h>
#include <SD.h>
#include <stdio.h>
#include <string.h>

#define BUILD_NUMBER      105

#define DEBUG             0

//Digital Pins
#define FF1pin            41 //fault flag 1
#define FF2pin            39 //fault flag 2
#define motorResetPin     49 //motor driver reset pin
#define pwmHPin           46 //motor driver PWM high pin
#define pwmLPin           44 //motor driver PWM low pin
#define dirPin            43 //motor driver direction pin
#define resetPin          41 //system reset. connected to "reset" pin on the arduino. high value resets the entire system
#define MSpin             14 //pin to determine whether the unit is in master or slave mode
#define MagnetSensorPin   19 //pin to magnet hall sensor

//Analog Pins
#define PSpin            A12 //motor driver power supply pin
#define CSPin            A13 //motor driver current sensor pin
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
#define PULL_SLOW_VELOCITY  0x05

//System modes
#define MS_IDLE         0xAA
#define MS_RUNNING      0xAB
#define MS_CALIBRATION  0xAC
#define SL_IDLE         0xBA
#define SL_RUNNING      0xBB
#define SL_CALIBRATION  0xBC

//PID
#define Kp                1.0603 
#define Ki                3.316
#define Kd                0.02872

//Other Constants
#define pi                      3.14159
#define radius                  6.5 //[cm] radius of the wheel. represents velocity at the end point
#define ropeRadius              2.23 // [cm] represent velocity where the rope is
#define numOfMagnets            2 //number of magnets available on the motor
#define maximumMotorVelocity    5.575 //maximum motor velocity 157.08[rad/second] = 2*pi*1500[RPM]/60[seconds] or 5.575[m/s]=roperadius(0.223[m])*1500[RPM]/60[sec]
#define VMEASURE_SAMPLE_TIME    500 //what is the timeframe for velocity measurement. [ms]
#define SLOW_VELOCITY           0.4 //the motor use this velocity when moveing slowly from one side to the other. [m/second]
#define MAX_PWM_VALUE           255
#define NOMINAL_TRACK_LENGTH    3000 //nominal length of the track (distance between motors) [cm]
#define SAFETY_DISTANCE         10 //safety distance from the edges in numOfmagnets
#define DELAY_BETWEEN_RUNNINGS  2000 //time to wait between runnings in [ms]

//XBEE
#define XBEE_DATA_RATE 57600

//XBEE messages
//Master->slave
#define GOTO_CALIBRATION    0x60
#define GOTO_RUN            0x61
#define GOTO_IDLE           0x62
#define SWITCH_TO_COAST     0x63
#define KEEP_ALIVE          0x64
#define LTR_VELOCITY_UPDATE 0x65
#define LTR_DELAY_UPDATE    0x66
#define PULL_SLOWLY         0x67
#define STOP_MOTOR          0x68
#define START_PULLING       0x69

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

//HTTP messages parsing cases:
#define UPDATE_VELOCITY_RTL       0x50
#define UPDATE_VELOCITY_LTR       0x51
#define UPDATE_BEGIN_CALIBRATION  0x52
#define UPDATE_BEGIN_RUNNING      0x53
#define UPDATE_STOP               0x54
#define UPDATE_STATUS             0x55
#define UPDATE_CSS			       	  0x56
#define UPDATE_JPG			       	  0x57
#define UPDATE_JS		         		  0x58
#define UPDATE_DELAY_RTL          0x59
#define UPDATE_DELAY_LTR          0x60

//WhatToSend states
#define SEND_KA                   0xF1
#define SEND_DELAY                0xF2
#define SEND_VELOCITY             0xF3

//HTTP message strings
#define MSG_RTL_VEL         "RTL_vel="
#define MSG_LTR_VEL         "LTR_vel="
#define MSG_CAL             "BeginCalibration"
#define MSG_RUN             "BeginRunning"
#define MSG_STOP            "Stop"
#define MSG_STATUS          "status"
#define MSG_CSS             ".css"
#define MSG_JS              ".js"
#define MSG_JPG             ".jpg"
#define MSG_RTL_DELAY       "RTL_delay="
#define MSG_LTR_DELAY       "LTR_delay="

//Ethernet
#define REQ_BUF_SZ          60 // size of buffer used to capture HTTP requests
#define WEBSITE_FILENAME    "targuino.htm" //name of the main html file on the SD card
#define JAVASCRIPT_FILENAME "targuino.js" //javascript filename
#define CSS_FILENAME        "targuino.css" //css filename
#define JPG_FILENAME        "targuino.jpg" //jpg filename
#define XML_FILENAME        "targuino.xml"

//Watchdogs and timers
#define DELAY_BETWEEN_KA    5000 //time between keep-alive messages
#define KA_WATCHDOG         3 //maximum keepalive messages to wait for SL response before error message appears
#define RESPONSE_TIMEOUT    6000 //timeout for a response from the other side

//Motor State description variables
boolean DIR                         = 0; // 1 - clockwise, 0 - counter-clockwise
volatile unsigned int Revolutions   = 0; //number of revolutions. global variable needs to be defined as volatile in order to be used by interrupts
volatile unsigned int RunLength     = 0; //this variable counts revolutions during a run and compare it to lTrack [Revolutions]. It also delivers the length to lTrack during calibration
unsigned int lTrack                 = 0; //stores the length of the track (= distance between motors). needs to be written to the EEPROM in the future
float vAngular                      = 0;
double measuredIn                   = 0; //[m/s]measured velocity
float K                             = 0.5; //averaging constant between very different velocity measurements
float kk                            = 0;
int distance_from_edge              = 0; //this variable contains the calculated distance from the opposite edge 

//Timer and watchdog variables
unsigned long PreviousInterruptTime = 0;
unsigned long CurrentTime           = 0;
unsigned int deltaT                 = 0;
unsigned int tBeginMovement         = 0; 
unsigned int timeGuard              = 0;
unsigned int maximumRunTime         = 0; //this is them maximum time the motor can run before watchdog is up
unsigned long PrevMsgSent          = 0; //stores the last time KEEP-ALIVE msg was sent
int numOfKAmsgs            = 0; //stores the number of keealive msgs that were sent

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
String TarMsg        = " "; //string that contains system status
boolean stopSignal      = false; //a general flag which uses to control whether the system goes to the next stage
int LTR_delay          = 0; //delay before slave starts to pull. in [sec]
int RTL_delay          = 0; //delay before master starts to pull. in [sec]
boolean delayHasPassed = false; //a flag states whether the delay time has passed
byte whatToSend        = 0xF1;   // a flag states what should the master send to the slave: KA, delay or velocity

//Ethernet
byte mac[] 										    	   = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress 				 				  			       ip(192, 168, 0, 177);
EthernetServer   			  				   			   server(80);
char HTTP_req[REQ_BUF_SZ] 						    	   = {0};  	// 0 = null in ascii
char req_index   			 			  			       = 0;       // index into HTTP_req buffer
typedef struct parsedResult {byte msgType; float value;} parsedResult;

//XBEE variables
byte msgToSend[3]              = {0, 0, 0};
byte receivedMsg[3]            = {0, 0, 0};
byte previousMsg               = 0; 
String XBEEstatusMsg         = " ";
//SD card
File TServerFile;
File xmlFile;

//Other variables - delete if unnecessary
int faultflag           = 0; //0- OK, 1-short, 2-overheat, 3-undervoltage
int sensorValue         = 0;
float current           = 0;
float CS                = 0;
boolean zeroStateFlag   = true;
int tempAcc             = 0;
unsigned int statTmp    = 0;
unsigned int sumsum     = 0;

//=====================SETUP=============================================
void setup(){
  // disable Ethernet chip
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  
  //initialize XBEE
  Serial.begin(             XBEE_DATA_RATE); 
  Serial2.begin(             XBEE_DATA_RATE); //for debug purposes

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
    
    //initialize default velocities and delays
    velLTR = 2; // [m/s] , default values
    LTR_delay = 1; // [m/s]
    velRTL = 2;
    RTL_delay = 1;

    sysMode = MS_IDLE;  
    //initialize Ethernet server
    Ethernet.begin(mac, ip);
    server.begin();
    initSD();
  }
  else {
    sysMode = SL_IDLE;
    Serial2.println("This is the slave motor");
  }
  
  myPID.SetOutputLimits(0,200);  //tell the PID to range between 0 and 255 (PWM range)    // some arbitrary value I've decided of. 255 is too much?
  myPID.SetMode(    AUTOMATIC); //turn the PID on
  myPID.SetSampleTime(    200);

  maximumRunTime = float(NOMINAL_TRACK_LENGTH / SLOW_VELOCITY) * 1000;
  
  Serial2.print("Maximum runtime is [ms]: "); Serial2.println(maximumRunTime);

  //setup interrupt for the magnet sensor
  attachInterrupt(4,IncrRevolution,RISING); //interrupt 4 is for pin number 19 - magnet sensor input

  PreviousInterruptTime = millis(); //start timer

}

//=====================MAIN=============================================
void loop(){
  switch (FSM_State) {
   case ZERO_STATE:
    if(zeroStateFlag){
      Serial2.println("i'm in state: zero! waiting for instructions");
      TarMsg = "System in idle mode. Waiting for instructions";
      zeroStateFlag = false;
    }
    switch(sysMode){
      case MS_IDLE:
        if(CurrentTime - PrevMsgSent > DELAY_BETWEEN_KA){
          PrevMsgSent = CurrentTime;
          switch(whatToSend){
            case SEND_KA:
              sendXBEEmsg(KEEP_ALIVE);
              delay(100);
              break;
            case SEND_VELOCITY:
              sendXBEEmsg(LTR_VELOCITY_UPDATE);
              delay(100);
              sendXBEEmsg(velLTR*10); //multiply by 10 to make translation to hex easier
              delay(100);
              whatToSend = SEND_KA;
              break;
            case SEND_DELAY:
              sendXBEEmsg(LTR_DELAY_UPDATE);
              delay(100);
              sendXBEEmsg(LTR_delay);
              delay(100);
              whatToSend = SEND_KA;
              break;
          }
        numOfKAmsgs++;    //this number is zeroed if we get KA-ACK (this is handled in SerialEvent)
        }
        if(numOfKAmsgs > KA_WATCHDOG){ 
          Serial2.print(float(CurrentTime / 1000)); Serial2.println(" ERROR - XBEE Slave error: no response from slave");
          XBEEstatusMsg = "No connection";
          numOfKAmsgs = 0;
          }
        break;

      case MS_CALIBRATION:
        sendXBEEmsg(GOTO_CALIBRATION);
        Serial2.println("GO_TO_CALIB msg sent. waiting for response...");
        FSM_State = WAIT_FOR_RESPONSE;
        break;

      case SL_CALIBRATION:
        sendXBEEmsg(ACK_CALIB);
        Serial2.println("ACK_CAL msg sent. waiting for response...");
        FSM_State = WAIT_FOR_RESPONSE;
        break;
      case MS_RUNNING:
        if(areVelocitiesLegal()){
          Serial2.println("illigal velocity values. aborting...");
          sysMode = MS_IDLE;
          break;    
        }
        Serial2.println("GO_TO_RUN msg sent. waiting for response...");
        sendXBEEmsg(GOTO_RUN);
        FSM_State = WAIT_FOR_RESPONSE;
        break;
      case SL_RUNNING:
        sendXBEEmsg(ACK_RUN);
        Serial2.println("ACK_RUN msg sent. waiting for instructions...");
        break;
      default:
        break; //do nothing
    }
    break;
   
   case WAIT_FOR_RESPONSE:
    //In case of response SerialEvent would update FSM_STATE we'll move on. We're having a watchdog just in case
    timeGuard = CurrentTime - PrevMsgSent;
    if(timeGuard > RESPONSE_TIMEOUT || stopSignal){
      coastMotor();
      sendXBEEmsg(STOP_MOTOR);
      FSM_State = ZERO_STATE;
      if(MS){ sysMode = MS_IDLE;}
      else{ sysMode = SL_IDLE;}
    }
    break;

   case PULL_AND_COUNT:
    
    timeGuard = CurrentTime - tBeginMovement;
    
    if(timeGuard > maximumRunTime || stopSignal){ // target probably arrived to the other edge of the rope 
      Serial2.print("i'm at the end! Distance is now: "); Serial2.print(RunLength); Serial2.println(" revolutions. stopping...");
      stopMotor();
      Serial2.print("saving data! length of track is: "); Serial2.print(RunLength); Serial2.println(" revolutions. Saving...");
      lTrack = RunLength; //save lTrack. In the future - maybe save to EEPROM
      sendXBEEmsg(GOTO_IDLE);
      FSM_State = WAIT_FOR_RESPONSE;
      DIR = 0;
      stopSignal = false; //reset stopSignal for the future
      break;
    }
    
    if(checkTime(VMEASURE_SAMPLE_TIME)){  
      MeasureVelocity(VMEASURE_SAMPLE_TIME);
      myPID.Compute();
      movemotor(DIR,sysIn,MAX_PWM_VALUE);
      Serial2.print("DesiredIn: "); Serial2.print(desiredIn); Serial2.print(" MeasuredIn: "); Serial2.print(measuredIn); Serial2.print("StopSignal = "); Serial2.println(stopSignal);

     }
    break;


   case PULL_SLOW_VELOCITY:
    
    TarMsg = "Starting to pull slowly. Press Stop when target is near the end";

    Serial2.println(TarMsg);

    if(stopSignal){ //if target arrived to the other side 
      coastMotor();
      sendXBEEmsg(START_PULLING);
      Serial2.println("Coasting. slave motor should be pulling now. waiting for donePulling");
      FSM_State = WAIT_FOR_RESPONSE;
      stopSignal = false; //reset stopSignal for future runs
      delayHasPassed = false; //reset delayhaspassed for future runs
      break;
    }

    if(delayHasPassed && checkTime(VMEASURE_SAMPLE_TIME)){  
      MeasureVelocity(VMEASURE_SAMPLE_TIME);
      myPID.Compute();
      movemotor(DIR,sysIn,MAX_PWM_VALUE);
     }
    break;


   case PULL_STATE:
    Serial2.print("i'm in state: starting a run! distance target has gone is: "); Serial2.print(RunLength); Serial2.print("/"); Serial2.print(lTrack); Serial2.println(" revolutions. running...");
    
    //next 3 lines implement the time delay
    timeGuard = CurrentTime - tBeginMovement;  
    if(MS && timeGuard > RTL_delay * 1000){ delayHasPassed = true; }  //if RTL_Delay has passed (for master) - raise the flag to start pulling
    else if(timeGuard > LTR_delay * 1000){ delayHasPassed = true; }    //if LTR_Delay has passed (for slave) - raise the flag to start pulling


    distance_from_edge = lTrack - RunLength;

    if(distance_from_edge < (2 * SAFETY_DISTANCE)){
      Serial2.println("getting closer to the edge...");
      desiredIn = 0.7 * desiredIn; //reducing velocity towards the edge
      //blinkLEDS(500, 2, 1);
     }

    if(distance_from_edge < SAFETY_DISTANCE || stopSignal){ //if target arrived to the other side 
      Serial2.print("i'm too close to the edge! Distance is now: "); Serial2.print(RunLength); Serial2.println(" revolutions. stopping...");
      coastMotor();
      if(MS){ 
        Serial2.println("Coasting. slave motor should be pulling now. waiting for donePulling");
        sendXBEEmsg(START_PULLING);
      }
      else { 
        sendXBEEmsg(DONE_PULLING);
        Serial2.println("Coasting. slave: donePulling");
      }
      FSM_State = WAIT_FOR_RESPONSE;
      stopSignal = false; //reset stopSignal for future runs
      delayHasPassed = false; //reset delayhaspassed for future runs
      break;
    }

    if(delayHasPassed && checkTime(VMEASURE_SAMPLE_TIME)){  
      MeasureVelocity(VMEASURE_SAMPLE_TIME);
      myPID.Compute();
      movemotor(DIR,sysIn,MAX_PWM_VALUE);
     }
    break;
       
   default:
    stopMotor(); //just in case something happens...
    Serial2.print("Got into default mode. FSM_state = "); Serial2.println(FSM_State);
    FSM_State = ZERO_STATE;
    break;
	} 
  
  CurrentTime = millis(); //main timer for all system purposes

  //current sensor
  sensorValue = analogRead(CSPin);  
  current = ( map(sensorValue, 0, 1023, 0, 5) - 2.5) / 0.066;
  
  //Ethernet communication handling
  if(MS){
    EthernetClient client = server.available();  // try to get client
    if(client){
    	//Serial2.println("client found!");
      boolean currentLineIsBlank = true;
  	  while(client.connected()){ 
        if(client.available()){   // client data available to read
			    char c = client.read(); // read 1 byte (character) from client
          //Serial2.print(c);
          //this if and elseif are due to (apparent) lack of memory. REQ_BUF_SZ > 100 causes arduino to reset. So we decided to take only the first 80 bytes
           if(req_index < REQ_BUF_SZ - 1){  /* limit the size of the stored received HTTP request.  buffer first part of HTTP request in HTTP_req. leave last element in array as 0 to null terminate string (REQ_BUF_SZ - 1)*/
              HTTP_req[req_index] = c;          // save HTTP request character
              req_index++;
          	}
          // last line of client request is blank and ends with \n
          // respond to client only after last line received
          if (c == '\n' && currentLineIsBlank) {
              
              parsedResult out = parseHTTPmsg(HTTP_req);

              switch(out.msgType){
                case UPDATE_VELOCITY_RTL:
                  velRTL = out.value;
                  Serial2.print("RTL velocity has been updated to: "); Serial2.println(velRTL);
                  break;

                case UPDATE_VELOCITY_LTR:
                  velLTR = out.value;
                  Serial2.print("LTR velocity has been updated to: "); Serial2.println(velLTR);
                  whatToSend = SEND_VELOCITY;
                  break;

                case UPDATE_BEGIN_CALIBRATION:
                  sysMode = MS_CALIBRATION;
                  sendXBEEmsg(GOTO_CALIBRATION);
                  FSM_State = WAIT_FOR_RESPONSE;
                  desiredIn = convertVelToInput(SLOW_VELOCITY);
                  Serial2.print(float(CurrentTime / 1000)); Serial2.println(" BeginCalibration has been received");
                  break;

                case UPDATE_BEGIN_RUNNING:
                  sysMode = MS_RUNNING;
                  sendXBEEmsg(GOTO_RUN);
                  FSM_State = WAIT_FOR_RESPONSE;
                  Serial2.print(float(CurrentTime / 1000)); Serial2.println(" BeginRunning has been received");
                  break;

                case UPDATE_STOP:
                  if(sysMode == MS_CALIBRATION && FSM_State == PULL_AND_COUNT){ //if master is pulling in calibration mode
                    stopSignal = true;
                    Serial2.print(float(CurrentTime / 1000)); Serial2.print(" StopSignal acknowledged");
                    break;
                  }
                  else if(sysMode == MS_CALIBRATION && FSM_State == WAIT_FOR_RESPONSE){ //if slave is pulling in calibration mode
                    sendXBEEmsg(STOP_MOTOR);
                    Serial2.println("stop message has been sent");
                    break;
                  }
                  else if(sysMode == MS_RUNNING && FSM_State == PULL_STATE){ //if master is pulling in running mode
                    stopSignal = true;
                    Serial2.println("StopSignal acknowledged");
                    break;
                  }
                  else if(sysMode == MS_RUNNING && FSM_State == WAIT_FOR_RESPONSE){ //if slave is pulling in running mode
                    stopSignal = true;
                    sendXBEEmsg(STOP_MOTOR);
                    Serial2.println("StopSignal acknowledged");
                    break;
                  }
                  else if(sysMode == MS_IDLE){
                    Serial2.println("StopSignal acknowledged but sysmode is idle so no action was taken");
                    break; 
                  }
                  break;

                case UPDATE_STATUS:
                  //Serial2.println("status update");
                  sendHTTPResponse('x',client);
                  printXML(client);
                  break;
                
                case UPDATE_CSS:
                  Serial2.println("sending css...");
                  sendHTTPResponse('h',client);
                  sendFile(CSS_FILENAME, client);                   
                  break;

                case UPDATE_JS:
                  Serial2.println("sending js...");
                  sendHTTPResponse('h',client);
                  sendFile(JAVASCRIPT_FILENAME, client);   
                  break;
                
                case UPDATE_JPG:
                  Serial2.println("sending jpeg...");
                  sendHTTPResponse('h',client);
                  // send file
                  sendFile(JPG_FILENAME, client);  
                  break;

                case UPDATE_DELAY_RTL:
                  RTL_delay = out.value;
                  Serial2.print("RTL delay has been updated to: "); Serial2.println(RTL_delay);
                  break;

                case UPDATE_DELAY_LTR:
                  LTR_delay = out.value;
                  Serial2.print("RTL delay has been updated to: "); Serial2.println(RTL_delay);
                  whatToSend = SEND_DELAY;
                  break;

                default:
                  // send rest of HTTP header          
                  Serial2.print("default message: "); Serial2.println(out.msgType);
                  sendHTTPResponse('h',client);
                  // send web page
                  sendFile(WEBSITE_FILENAME, client);
                  break;

              }

              req_index = 0;// reset buffer index and all buffer elements to 0
              StrClear(HTTP_req, REQ_BUF_SZ);
              break;
            }

            // every line of text received from the client ends with \r\n
            if (c == '\n'){ 		currentLineIsBlank = true; } // last character on line of received text, starting new line with next character read
            else if (c != '\r'){   currentLineIsBlank = false; }// a text character was received from client
        } // end if (client.available())
      } // end while (client.connected())
      delay(1);      // give the web browser time to receive the data
      client.stop(); // close the connection	
      //Serial2.println("Connection closed");
  	} //end if(client)
  }

  sumsum = CurrentTime - statTmp;
  if(sumsum > 4000){
    Serial2.print(float(CurrentTime / 1000)); Serial2.print(" FSM_State: "); Serial2.print(FSM_State, HEX); Serial2.print(" sysMode: "); Serial2.print(sysMode,HEX); Serial2.print(" LTR_delay: "); Serial2.print(LTR_delay); Serial2.print(" LTR_Velocity: "); Serial2.print(velLTR); Serial2.print(" RTL_delay: "); Serial2.print(RTL_delay); Serial2.print(" RTL_Velocity: "); Serial2.print(velRTL); Serial2.print(" numOfKAmsgs: "); Serial2.println(numOfKAmsgs);   
    statTmp = CurrentTime;
  }

}//end loop
  
//=====================FUNCTIONS=============================================
boolean movemotor(boolean DIR, int pwmH, int pwmL){ //set motor velocity and direction
  
  if(pwmH > 255 || pwmL >255){  
    pwmH = 0;
    } //illigal values
  
  digitalWrite(dirPin,DIR); //maybe *255?
  analogWrite(pwmHPin, pwmH);
  analogWrite(pwmLPin, pwmL);
  return 0;
}

void IncrRevolution(){ //ISR function - measure revolutions
  
  Revolutions++;
  RunLength++;

}

void MeasureVelocity(int deltaT){ //returns kalman filtered linear speed 
  
  float vAngularOld = vAngular;
  vAngular= 1000 * Revolutions / (numOfMagnets * deltaT); //multiply 1/period by 1000 to get the time in seconds and the velocity in Hz
    
  float delta = vAngular - vAngularOld; 
  
  if(delta > 0){        kk = K;  }
  else if(delta < 0){ kk = -K; }
  else{               kk = 0;}    
  
  measuredIn = ( vAngular + kk * abs (delta ) ) * 255 / ( 100 * maximumMotorVelocity / ropeRadius); //convert vAngular to PWM values
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

void serialEvent() {
  byte temp_msg = 0;
  byte processedMSG = 0;
  
  if(Serial.available() > 2){ //if there are 3 bytes or more in the buffer
    processedMSG = procRecMsg();
    clrXBEErecBuff();
    //if(!MS){ Serial2.print(float(CurrentTime / 1000)); Serial2.print(" Proccessed message is: "); Serial2.println(processedMSG);}
  }

  switch(processedMSG){
    case ACK_KA: //keep alive ack
      numOfKAmsgs = 0;
      XBEEstatusMsg = "Live!";
      Serial2.print(float(CurrentTime / 1000)); Serial2.println(" Keep alive received");
      break;

    case ACK_VEL:
      numOfKAmsgs = 0;
      XBEEstatusMsg = "Live!";
      Serial2.print(float(CurrentTime / 1000)); Serial2.println(" LTR velocity ack received");
      break;

    case ACK_DELAY:
      numOfKAmsgs = 0;
      XBEEstatusMsg = "Live!";
      Serial2.print(float(CurrentTime / 1000)); Serial2.println(" LTR delay ack received");
      break;

    case GOTO_CALIBRATION:
      sysMode = SL_CALIBRATION;
      sendXBEEmsg(ACK_CALIB);
      FSM_State = WAIT_FOR_RESPONSE;
      Serial2.print(float(CurrentTime / 1000)); Serial2.println(" goto calibration received");
      break;

    case GOTO_RUN:
      sysMode = SL_RUNNING;
      sendXBEEmsg(ACK_RUN);
      FSM_State = WAIT_FOR_RESPONSE;    
      Serial2.print(float(CurrentTime / 1000)); Serial2.println(" goto run received");  
      break;

    case GOTO_IDLE:
      sysMode = SL_IDLE;
      FSM_State = ZERO_STATE;
      zeroStateFlag = true;
      sendXBEEmsg(ACK_IDLE);
      Serial2.print(float(CurrentTime / 1000)); Serial2.println(" goto idle received");  
      break;

    case SWITCH_TO_COAST:
      coastMotor();
      sendXBEEmsg(ACK_COAST);
      FSM_State = WAIT_FOR_RESPONSE;
      Serial2.print(float(CurrentTime / 1000)); Serial2.println(" switching to coast");  
      break;

    case KEEP_ALIVE:
      sendXBEEmsg(ACK_KA);
      Serial2.print(float(CurrentTime / 1000)); Serial2.println(" keep alive request received from master");  
      break;

    case ACK_CALIB:
      coastMotor();
      sendXBEEmsg(PULL_SLOWLY);
      FSM_State = WAIT_FOR_RESPONSE;
      Serial2.print(float(CurrentTime / 1000)); Serial2.println(" ack calib received"); 
      break;

    case ACK_RUN:
      coastMotor();
      sendXBEEmsg(PULL_SLOWLY);
      FSM_State = WAIT_FOR_RESPONSE;
      Serial2.print(float(CurrentTime / 1000)); Serial2.println(" ack run received, waiting for donePulling msg"); 
      break;

    case ACK_IDLE:
      sysMode = MS_IDLE;
      FSM_State = ZERO_STATE;
      zeroStateFlag = true;
      Serial2.print(float(CurrentTime / 1000)); Serial2.println(" ack idle received, going back to idle"); 
      break; 

    case ACK_COAST: //only master motor get ack_coast
      switch(sysMode){
        case MS_CALIBRATION:
          desiredIn = convertVelToInput(SLOW_VELOCITY);
          FSM_State = PULL_AND_COUNT;
          tBeginMovement = CurrentTime;
          Serial2.print(float(CurrentTime / 1000)); Serial2.println(" ack calib received in master calibration mode"); 
          break;
        case MS_RUNNING:
          desiredIn = convertVelToInput(velRTL);
          FSM_State = PULL_STATE;
          tBeginMovement = CurrentTime;
          Serial2.println("ack calib received in master running mode"); 
          break;
        default:
          Serial2.println("ERROR - ACK_COAST receievd but I don't know what to do with it");
          break; //do nothing
      }
      break;

    case DONE_PULLING:
      sendXBEEmsg(SWITCH_TO_COAST);
      FSM_State = WAIT_FOR_RESPONSE;
      Serial2.println("done pulling received, waiting for ack_coast"); 
      break;  

    case LTR_VELOCITY_UPDATE:
      previousMsg = LTR_VELOCITY_UPDATE;
      break;
      
    case PULL_SLOWLY:
      sendXBEEmsg(ACK_PULL);
      desiredIn = convertVelToInput(SLOW_VELOCITY);
      FSM_State = PULL_SLOW_VELOCITY;
      tBeginMovement = CurrentTime;
      break;

    case LTR_DELAY_UPDATE:
      previousMsg = LTR_DELAY_UPDATE;
      break;

    case STOP_MOTOR:
      stopMotor();
      FSM_State = ZERO_STATE;
      if(MS){ sysMode = MS_IDLE;}
      else{ sysMode = SL_IDLE;}
      break;

    default: //could be velocity value, delay value or just garbage
      if(processedMSG){ //if there is still a value that was received (0 is not a valid message)
        switch(previousMsg){
          case LTR_VELOCITY_UPDATE:
            velLTR = (float) processedMSG / 10;
            Serial2.print(float(CurrentTime / 1000)); Serial2.print(" LTR vel received: "); Serial2.println(velLTR);
            sendXBEEmsg(ACK_VEL);
            previousMsg = 999; 
            break;
          case LTR_DELAY_UPDATE:
            LTR_delay = processedMSG;
            Serial2.print(float(CurrentTime / 1000)); Serial2.print(" LTR delay received: "); Serial2.println(LTR_delay); 
            sendXBEEmsg(ACK_DELAY);
            previousMsg = 999;
            break;
          case 999:
            break;
          default:
            Serial2.print(float(CurrentTime / 1000)); Serial2.print(" Unknown character received: "); Serial2.println(processedMSG, HEX);
            break;
        }
      }
      break;
  }
}
  
void stopMotor(){
  movemotor(DIR,0,MAX_PWM_VALUE);
}

boolean initSD(){ //initialize SD card
    if (!SD.begin(4)) {
        Serial2.println("ERROR - SD card initialization failed!");
        return 1;    // init failed
    }
    Serial2.println("SUCCESS - SD card initialized.");
    // check for index.htm file
    if (!SD.exists(WEBSITE_FILENAME)) {
        Serial2.println("ERROR - Can't find targuino.htm file!");
        return 1;  // can't find index file
    }
    Serial2.println("SUCCESS - Found htm file");
    return 0;
}

boolean sendFile(char *Tfilename, EthernetClient cl){
  TServerFile = SD.open(Tfilename);        // open web page file
  if(TServerFile) {
    while(TServerFile.available()){
       cl.write(TServerFile.read()); // send web page to client
      }
    TServerFile.close();
    return 0;
    }
  else{
    Serial2.print("ERROR - Can't open "); Serial2.println(Tfilename);
    return 1;
  }
  }

void StrClear(char *str, char str_length){ // sets every element of str to 0 (clears array)
    for (int i = 0; i < str_length; i++){ str[i] = 0;}
}


unsigned int StrContains(char *str, char sfind[]){ // searches for the string sfind in the string str. returns the index of the last letter if string found, returns 0 if string not found
  char found = 0;
  char index = 0;
  char len;
  unsigned int iEnd = 0;

  len = strlen(str);
    
  if (strlen(sfind) > len){ 
    return 0; //any other value can be considered as an index value, so return 0/1 convention is not adequate here. 
  }

  while (index < len) {
      if (str[index] == sfind[found]){ 
        found++;
        iEnd = index;
        if (strlen(sfind) == found){ return iEnd;}
      }
      else{ found = 0;}
      index++;
  }
  return 0;
}

boolean sendHTTPResponse(char type, EthernetClient cl){
  if(type == 'x'){//"x" for xml response
    cl.println("HTTP/1.1 200 OK");
    cl.println("Content-Type: text/xml");
    cl.println("Connection: closed"); //closed?
  }
  else if(type == 'h'){ //"h" for html response 
    cl.println("HTTP/1.1 200 OK");
    cl.println("Content-Type: text/html");
    cl.println("Connection: closed");
  }
  else if(type == 'f'){ //"f" for file request
    cl.println("HTTP/1.1 200 OK");
  }
  else{
    return 1;
  }
  cl.println();
  return 0;
}

boolean updateXML(){
  xmlFile = SD.open(XML_FILENAME,FILE_WRITE);
  if(xmlFile) {
    Serial2.println("XML file opened successfully");
    int fileSize = xmlFile.size();
    xmlFile.seek(0); //go to the beginning of the file
    //while(xmlFile.available()){
       //cl.write(TServerFile.read()); // send web page to client
      //}
    TServerFile.close();
    return 0;

    }
  else{
    Serial2.print("ERROR - Can't open ");
    return 1;
  }
}

void coastMotor(){
  digitalWrite(pwmLPin,0);
  digitalWrite(pwmHPin,0);
}

void clrXBEEsendBuff(){
  msgToSend[0] = 0;
  msgToSend[1] = 0;
  msgToSend[2] = 0;
}

void clrXBEErecBuff(){
  receivedMsg[0] = 0;
  receivedMsg[1] = 0;
  receivedMsg[2] = 0;
}

void sendXBEEmsg(byte buff){
  msgToSend[0] = buff;
  msgToSend[1] = buff;
  msgToSend[2] = buff;
  Serial.write(msgToSend, 3);
  Serial2.print("message sent: "); Serial2.print(buff, HEX); Serial2.println(" ");
}

byte procRecMsg(){
  Serial.readBytes(receivedMsg, 3);
  
  Serial2.print(float(CurrentTime / 1000)); Serial2.print(" received message: "); Serial2.print(receivedMsg[0], HEX); Serial2.print(" "); Serial2.print(receivedMsg[0], HEX); Serial2.print(" "); Serial2.print(receivedMsg[0], HEX); Serial2.println("");

  boolean cmpr1 = receivedMsg[0] ^ receivedMsg[1];
  boolean cmpr2 = receivedMsg[0] ^ receivedMsg[2];
  boolean cmpr3 = receivedMsg[1] ^ receivedMsg[2];

  if(cmpr1 && cmpr2 && cmpr3){ return 0; } //message is totally corrupted. all of the messages are different from each other. only here "return 0" is an error 
  else if(cmpr1 && cmpr2){ return receivedMsg[1]; } //msg 1 is corrupted. 2 and 3 are the same (I've chosen 2)
  else if(cmpr2 && cmpr3){ return receivedMsg[0]; } //msg 3 is corrupted. 1 and 2 are the same (I've chosen 1)
  else if(cmpr1 && cmpr3){ return receivedMsg[2]; } //msg 2 is corrupted. 1 and 3 are the same (I've chosen 3)
  else{ return receivedMsg[0]; } //all msgs are the same (I've chosen 1)
}

void blinkLEDS(int tBetween, int times, boolean bothLEDS){
  int ntimes = 0;
  while(ntimes < times){
                  digitalWrite(LED1, LOW);
    if(bothLEDS){ digitalWrite(LED2, LOW); }  
    delay(tBetween);
                  digitalWrite(LED1, HIGH); 
    if(bothLEDS){ digitalWrite(LED2, HIGH); }
  }
}

boolean areVelocitiesLegal(){ //check wether the values for the velocities are correct
  if(velRTL > maximumMotorVelocity * ropeRadius || velLTR > maximumMotorVelocity * ropeRadius){ return 1;}
  else{ return 0;}
}

float convertVelToInput(float vel){
  return vel * 255 / maximumMotorVelocity;
}

struct parsedResult parseHTTPmsg(char *msg){
  parsedResult output;
  char temp[10] = "";


  //look for RTL velocity
  unsigned int ind = StrContains(msg, MSG_RTL_VEL);
  if(ind){
    output.msgType = UPDATE_VELOCITY_RTL;
    
    unsigned int andInd = StrContains(msg, "&");

    strncpy( temp, &msg[ind+1], andInd - ind + 1); //take only 2 first digits (ignore the last 0)
    output.value = atof(temp) / 10000;

    //Serial2.print("memcpy is:"); Serial2.print(temp); Serial2.print(" result is: "); Serial2.println(output.value);
    return output;
  }

  //look for LTR velocity
  ind = StrContains(msg, MSG_LTR_VEL);
  if(ind){
    output.msgType = UPDATE_VELOCITY_LTR;
    
    unsigned int andInd = StrContains(msg, "&");

    strncpy( temp, &msg[ind+1], andInd - ind + 1); //take only 2 first digits (ignore the last 0)
    output.value = atof(temp) / 10000;

    //Serial2.print("memcpy is:"); Serial2.print(temp); Serial2.print(" result is: "); Serial2.println(output.value);

    return output;
  }

  //look for BeginCalibration
  ind = StrContains(msg, MSG_CAL);
  if(ind){
    output.msgType = UPDATE_BEGIN_CALIBRATION;
    output.value = 99.99;
    return output;
  }

  //look for BeginRun
  ind = StrContains(msg, MSG_RUN);
  if(ind){
    output.msgType = UPDATE_BEGIN_RUNNING;
    output.value = 99.99;
    return output;
  }

  //look for StopSignal
  ind = StrContains(msg, MSG_STOP);
  if(ind){
    output.msgType = UPDATE_STOP;
    output.value = 99.99;
    return output;
  }

 //look for status
  ind = StrContains(msg, MSG_STATUS);
  if(ind){
    output.msgType = UPDATE_STATUS;
    output.value = 99.99;
    return output;
  }

   //look for CSS file
  ind = StrContains(msg, MSG_CSS);
  if(ind){
    output.msgType = UPDATE_CSS;
    output.value = 99.99;
    return output;
  }

  //look for js file‎
  ind = StrContains(msg, MSG_JS);
  if(ind){
    output.msgType = UPDATE_JS;
    output.value = 99.99;
    return output;
  }
  
  //look for jpg file
  ind = StrContains(msg, MSG_JPG);
  if(ind){
    output.msgType = UPDATE_JPG;
    output.value = 99.99;
    return output;
  }

  //look for RTL delay time update
  ind = StrContains(msg, MSG_RTL_DELAY);
  if(ind){
    output.msgType = UPDATE_DELAY_RTL;
    
    unsigned int andInd = StrContains(msg, "&");

    strncpy( temp, &msg[ind+1], andInd - ind);
    output.value = atoi(temp) / 1000; //same trick like with velocity: multiply by 1000 so it would be easy to be parsed

    //Serial2.print("strncpy delay is:"); Serial2.print(temp); Serial2.print(" result is: "); Serial2.println(output.value);
    return output;
  }

  //look for LTR delay time update
  ind = StrContains(msg, MSG_LTR_DELAY);
  if(ind){
    output.msgType = UPDATE_DELAY_LTR;
    
    unsigned int andInd = StrContains(msg, "&");

    strncpy( temp, &msg[ind+1], andInd - ind);
    output.value = atoi(temp) / 1000; //same trick like with velocity: multiply by 1000 so it would be easy to be parsed

    //Serial2.print("strncpy delay is:"); Serial2.print(temp); Serial2.print(" result is: "); Serial2.println(output.value);
    return output;
  }
}

void printXML(EthernetClient cl){
  cl.println("<?xml version = \"1.0\" ?>");
  cl.println("<Targuino>");
  cl.print("<RTL_Velocity>"); cl.print(velRTL); cl.println("</RTL_Velocity>");
  cl.print("<RTL_Delay>"); cl.print(RTL_delay); cl.println("</RTL_Delay>");
  cl.print("<LTR_Velocity>"); cl.print(velLTR); cl.println("</LTR_Velocity>");
  cl.print("<LTR_Delay>"); cl.print(LTR_delay); cl.println("</LTR_Delay>");
  cl.print("<SysMode>"); cl.print(sysMode); cl.println("</SysMode>");
  cl.print("<Messages>"); cl.print(TarMsg); cl.println("</Messages>");
  cl.print("<XBEE_STATUS>"); cl.print(XBEEstatusMsg); cl.println("</XBEE_STATUS>");
  cl.print("<TrackLength>"); cl.print(lTrack * ropeRadius); cl.println("</TrackLength>");
  cl.print("</Targuino>");
}

