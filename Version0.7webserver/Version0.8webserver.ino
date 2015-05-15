#include <PID_v1.h>
#include <SPI.h>
#include <Ethernet.h>
#include <SD.h>
#include <stdio.h>
#include <string.h>

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

//State machine
#define numOfStates         8
#define ZERO_STATE          0x01
#define WAIT_FOR_RESPONSE   0x02
#define PULL_STATE          0x03
#define PULL_AND_COUNT      0x04

//System modes
#define MS_IDLE         0xAA
#define MS_RUNNING      0xAB
#define MS_CALIBRATION  0xAC
#define SL_IDLE         0xBA
#define SL_RUNNING      0xBB
#define SL_CALIBRATION  0xBC

//PID
#define Kp                0.0603 
#define Ki                3.316
#define Kd                0.02872

//Other Constants
#define pi                      3.14159
#define radius                  6.5 //[cm] radius of the wheel. represents velocity at the end point
#define ropeRadius              2.23 // [cm] represent velocity where the rope is
#define numOfMagnets            2 //number of magnets available on the motor
#define maximumMotorVelocity    157.08  //maximum motor velocity [rad/second] = 2*pi*1500[RPM]/60[seconds]   
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
#define GOTO_CALIBRATION    0x10
#define GOTO_RUN            0x11
#define GOTO_IDLE           0x12
#define SWITCH_TO_COAST     0x13
#define KEEP_ALIVE          0x14
#define LTR_VELOCITY_UPDATE 0x15
#define LTR_DELAY_UPDATE    0x16
#define PULL_SLOWLY         0x17
#define STOP_MOTOR          0x18
#define START_PULLING       0x19

//Slave->master
#define ACK_CALIB           0x20
#define ACK_RUN             0x21
#define ACK_IDLE            0x22
#define ACK_COAST           0x23
#define DONE_PULLING        0x24
#define ACK_KA              0x25
#define ACK_VEL             0x26
#define ACK_PULL            0x27
#define ACK_STOP            0x28
#define ACK_DELAY           0x29

//HTTP messages parsing cases:
#define UPDATE_VELOCITY_RTL       0x50
#define UPDATE_VELOCITY_LTR       0x51
#define UPDATE_BEGIN_CALIBRATION  0x52
#define UPDATE_BEGIN_RUNNING      0x53
#define UPDATE_STOP               0x54
#define UPDATE_STATUS             0x55

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

//Ethernet
#define REQ_BUF_SZ          100 // size of buffer used to capture HTTP requests
#define WEBSITE_FILENAME    "targuino.htm" //name of the main html file on the SD card
#define JAVASCRIPT_FILENAME "targuino.js" //javascript filename
#define CSS_FILENAME        "targuino.css" //css filename
#define JPG_FILENAME         "targuino.jpg" //jpg filename

//Watchdogs and timers
#define DELAY_BETWEEN_KA    3000 //time between keep-alive messages
#define KA_WATCHDOG         3 //maximum keepalive messages to wait for SL response before error message appears

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

unsigned int PrevKAmsgSent          = 0; //stores the last time KEEP-ALIVE msg was sent
int numOfKAmsgs            = 0; //stores the number of keealive msgs that were sent

//PID Variables
double desiredIn  = 0; // this variable stores the desired velocity in PWM range (0-255)
double sysIn      = 0; //the calculated input to the driver
PID myPID(&measuredIn, &sysIn, &desiredIn, Kp, Ki, Kd, DIRECT);

//system state variables
boolean MS              = 0; //is arduino in Master or Slave mode (0 for slave mode, 1 for master)
byte FSM_State          = 0; //system's currect state in the state machine. default value is 0 - initial state (see #define)
byte sysMode            = 0; //this byte indicate two thing: 1) is this motor MS/SL   2) is the system in idle, calibration or run mode
float velLTR            = 0; //the velocity the user has requested for the run from left to right
float velRTL            = 0; //the velocity the user has requested for the run from right to left
String TarStatus        = ""; //string that contains system status
boolean openGate        = false; //a general flag which uses to control whether the system goes to the next stage
byte LTR_delay          = 0; //delay before slave starts to pull. in [ms]
byte RTL_delay          = 0; //delay before master starts to pull. in [ms]

//Ethernet
byte mac[] 					  	    = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress 				          ip(192, 168, 0, 177);
EthernetServer   			      server(80);
char HTTP_req[REQ_BUF_SZ] 	= {0};  	// 0 = null in ascii
char req_index   			      = 0;       // index into HTTP_req buffer

//XBEE variables
byte msgToSend[]              = {0, 0, 0};
byte receivedMsg[]            = {0, 0, 0};
boolean isXbeeAlive           = 0;
boolean gotAck                = 0;

//SD card
File TServerFile;

//Other variables - delete if unnecessary
int faultflag           = 0; //0- OK, 1-short, 2-overheat, 3-undervoltage
int sensorValue         = 0;
float current           = 0;
float CS                = 0;
boolean tmpFlag         = true;

//=====================SETUP=============================================
void setup(){
  // disable Ethernet chip
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  
  //initialize XBEE
  Serial.begin(             XBEE_DATA_RATE); 
  Serial2.begin(             XBEE_DATA_RATE); //for debug purposes

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

  //init buttons and leds
  pinMode(runButton, INPUT);
  pinMode(calButton, INPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
 
  //initialize reset pin
  digitalWrite(resetPin,           HIGH); //switching to LOW will reset the arduino
  //CHECK MASTER/SLAVE MODE
  if(digitalRead(MSpin) ==  HIGH){  MS = 1; }
  
  if(MS){
    //determine initial system mode
    sysMode = MS_IDLE;  
    //initialize Ethernet server
    Ethernet.begin(mac, ip);
    server.begin();
  	initSD();
  }
  else if(!MS){
    sysMode = SL_IDLE;
    TarStatus += "going into state: zero slave &#13;&#10;";
    Serial2.println(TarStatus);
  }
  
  myPID.SetOutputLimits(0,200);  //tell the PID to range between 0 and 255 (PWM range)    // some arbitrary value I've decided of. 255 is too much?
  myPID.SetMode(    AUTOMATIC); //turn the PID on
  myPID.SetSampleTime(    200);

  maximumRunTime = float(NOMINAL_TRACK_LENGTH / SLOW_VELOCITY / ropeRadius) * 1000;
  
  Serial2.print("Maximum runtime is: "); Serial2.println(maximumRunTime);

  //setup interrupt for the magnet sensor
  attachInterrupt(4,IncrRevolution,RISING); //interrupt 4 is for pin number 19 - magnet sensor input

  PreviousInterruptTime = millis(); //start timer

}

//=====================MAIN=============================================
void loop(){
  switch (FSM_State) {
   case ZERO_STATE:
    if(tmpFlag){
      TarStatus += "i'm in state: zero! waiting for instructions &#13;&#10;";
      Serial2.println(TarStatus);
      tmpFlag = false;
      blinkLEDS(500 , 3, 1);
    }
    switch(sysMode){
      case MS_IDLE:
        CurrentTime = millis();
        if(CurrentTime - PrevKAmsgSent > DELAY_BETWEEN_KA){
          sendXBEEmsg(KEEP_ALIVE);
          delay(100);
          sendXBEEmsg(LTR_VELOCITY_UPDATE);
          delay(100);
          sendXBEEmsg(velLTR);
          delay(100);
          sendXBEEmsg(LTR_DELAY_UPDATE);
          delay(100);
          sendXBEEmsg(LTR_delay);
          PrevKAmsgSent = CurrentTime;
          numOfKAmsgs++; //this number is zeroed if we get KA-ACK (this is handled in SerialEvent)
        }
        if(numOfKAmsgs > KA_WATCHDOG){
          TarStatus += "ERROR - XBEE Slave error: no response from slave";
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
    //this is an empty state, where the controller just wait for response. In case of response SerialEvent would update FSM_STATE we'll move on.
    break;

   case PULL_AND_COUNT:
    Serial2.print("DesiredIn: "); Serial.print(desiredIn); Serial.print(" MeasuredIn: "); Serial.println(measuredIn);
    
    timeGuard = millis() - tBeginMovement;
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
     }
    break;

   case PULL_STATE:
    Serial2.print("i'm in state: starting a run! distance target has gone is: "); Serial2.print(RunLength); Serial2.print("/"); Serial2.print(lTrack); Serial2.println(" revolutions. running...");
     
    distance_from_edge = lTrack - RunLength;

    if(distance_from_edge < (2 * SAFETY_DISTANCE)){
      Serial2.println("getting closer to the edge...");
      desiredIn = 0.5 * desiredIn; //reducing velocity towards the edge
      blinkLEDS(500, 2, 1);
     }

    if(distance_from_edge < SAFETY_DISTANCE || stopSignal){ //if target arrived to the other side 
      Serial2.print("i'm too close to the edge! Distance is now: "); Serial2.print(RunLength); Serial2.println(" revolutions. stopping...");
      delay(LTR_delay);
      coastMotor();
      sendXBEEmsg(START_PULLING);
      FSM_State = WAIT_FOR_RESPONSE;
      stopSignal = false; //reset stopSignal for the future
      break;
    }
    if(checkTime(VMEASURE_SAMPLE_TIME)){  
      MeasureVelocity(VMEASURE_SAMPLE_TIME);
      myPID.Compute();
      movemotor(DIR,sysIn,MAX_PWM_VALUE);
     }
    break;
       
   case SLAVE_STATE:
    //do nothing and wait for instructions
    break;
       
   default:
    FSM_State = ZERO_STATE;
    stopMotor(); //just in case something happens...
    Serial2.println("Got into default mode");
    break;
	} 
  //current sensor
  sensorValue = analogRead(CSPin);  
  current = ( map(sensorValue, 0, 1023, 0, 5) - 2.5) / 0.066;

  //Ethernet communication handling
  if(MS){
    EthernetClient client = server.available();  // try to get client
    if(client){
    	boolean currentLineIsBlank = true;
  	  while(client.connected()){
        if(client.available()){   // client data available to read
			    char c = client.read(); // read 1 byte (character) from client
          if (req_index < (REQ_BUF_SZ - 1)){     // limit the size of the stored received HTTP request.  buffer first part of HTTP request in HTTP_req. leave last element in array as 0 to null terminate string (REQ_BUF_SZ - 1)
              HTTP_req[req_index] = c;          // save HTTP request character
              req_index++;
          	}

          // last line of client request is blank and ends with \n
          // respond to client only after last line received
          if (c == '\n' && currentLineIsBlank) {

              struct out = parseHTTPmsg(HTTP_req);

              switch(out.msgType){
                case UPDATE_VELOCITY_RTL:
                  velRTL = out.velocity / ropeRadius;
                  Serial2.print("RTL velocity has been updated to: "); Serial2.println(velRTL);
                  break;

                case UPDATE_VELOCITY_LTR:
                  velLTR = out.velocity / ropeRadius;
                  Serial2.print("LTR velocity has been updated to: "); Serial2.println(velLTR);
                  break;

                case UPDATE_BEGIN_CALIBRATION:
                  sysMode = MS_CALIBRATION;
                  sendXBEEmsg(GOTO_CALIBRATION);
                  FSM_State = WAIT_FOR_RESPONSE;
                  desiredIn = SLOW_VELOCITY;
                  break;

                case UPDATE_BEGIN_RUNNING:
                  sysMode = MS_RUNNING;
                  sendXBEEmsg(GOTO_RUN);
                  FSM_State = WAIT_FOR_RESPONSE;
                  break;

                case UPDATE_STOP:
                  if(sysMode == MS_CALIBRATION && FSM_State == PULL_AND_COUNT){ //if master is pulling in calibration mode
                    stopSignal == true;
                    Serial2.println("StopSignal acknowledge");
                    break;
                  }
                  else if(sysMode == MS_CALIBRATION && FSM_State == WAIT_FOR_RESPONSE){ //if slave is pulling in calibration mode
                    sendXBEEmsg(STOP_MOTOR);
                    Serial2.println("stop message has been sent");
                    break;
                  }
                  else if(sysMode == MS_RUNNING && FSM_State == PULL_STATE){ //if master is pulling in running mode
                    stopSignal == true;
                    Serial2.println("StopSignal acknowledge");
                    break;
                  }
                  else if(sysMode == MS_RUNNING && FSM_State == WAIT_FOR_RESPONSE){ //if slave is pulling in running mode
                    stopSignal == true;
                    Serial2.println("StopSignal acknowledge");
                    break;
                  }
                  break;

                case UPDATE_STATUS:
                  sendHTTPResponse('x',client);  
                  respondXML();
                  break;
                
                case UPDATE_CSS:
                  sendHTTPResponse('h',client);
                  openWebPage(CSS_FILENAME, client);                   
                  break;

                case UPDATE_JS:
                  sendHTTPResponse('h',client);
                  openWebPage(JAVASCRIPT_FILENAME, client);   
                  break;
                
                case UPDATE_JPG:
                  sendHTTPResponse('h',client);
                  // send file
                  openWebPage(JPG_FILENAME, client);  
                  break;

                default:
                  // send rest of HTTP header
                  sendHTTPResponse('h',client);
                  // send web page
                  openWebPage(WEBSITE_FILENAME, client);
                break;

              }
              Serial2.print(HTTP_req); // display received HTTP request on serial port
              req_index = 0;// reset buffer index and all buffer elements to 0
              StrClear(HTTP_req, REQ_BUF_SZ);
            }
            // every line of text received from the client ends with \r\n
            if (c == '\n'){ 		currentLineIsBlank = true; } // last character on line of received text, starting new line with next character read
            else if (c != '\r'){   currentLineIsBlank = false; }// a text character was received from client
        } // end if (client.available())
      } // end while (client.connected())
      delay(1);      // give the web browser time to receive the data
      client.stop(); // close the connection	
      Serial2.println("Connection closed");
  	} //end if(client)
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
  vAngular= 2 * pi * 1000 * Revolutions / (numOfMagnets*deltaT); //multiply 1/period by 1000 to get the time in seconds and the velocity in Hz
    
  float delta = vAngular - vAngularOld; 
  if(delta > 0){        kk = K;  }
    else if(delta < 0){ kk = -K; }
    else{               kk = 0;  }
   
  measuredIn = ( vAngular + kk * abs (delta ) ) * 255 / maximumMotorVelocity; //convert vAngular to PWM values
  Revolutions = 0;
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

void serialEvent() {
  byte temp_msg = 0;
  
  if(Serial.available() > 2){ //if there are 3 bytes or more in the buffer
    processedMSG = procRecMsg();
    clrXBEErecBuff();
  }
  
  Serial2.print("proccesed msg is: "); Serial2.println(processedMSG);

  switch(processedMSG){
    case ACK_KA: //keep alive ack
      numOfKAmsgs = 0; 
      break;

    case GOTO_CALIBRATION:
      sysMode = SL_CALIBRATION;
      sendXBEEmsg(ACK_CALIB);
      FSM_State = WAIT_FOR_RESPONSE;
      break;

    case GOTO_RUN:
      sysMode = SL_RUNNING;
      sendXBEEmsg(ACK_RUN);
      FSM_State = WAIT_FOR_RESPONSE;      
      break;

    case GOTO_IDLE:
      sysMode = SL_IDLE;
      FSM_State = ZERO_STATE;
      sendXBEEmsg(ACK_IDLE);
      break;

    case SWITCH_TO_COAST:
      coastMotor();
      sendXBEEmsg(ACK_COAST);
      FSM_State = WAIT_FOR_RESPONSE;
      break;

    case KEEP_ALIVE:
      sendXBEEmsg(ACK_KA);
      break;

    case ACK_CALIB:
      coastMotor();
      sendXBEEmsg(PULL_SLOWLY);
      FSM_State = WAIT_FOR_RESPONSE;
      break;

    case ACK_RUN:
      coastMotor();
      sendXBEEmsg(PULL_SLOWLY);
      FSM_State = WAIT_FOR_RESPONSE;
      break;

    case ACK_IDLE:
      sysMode = MS_IDLE;
      FSM_State = ZERO_STATE;
      break; 

    case ACK_COAST: //only master motor get ack_coast
      switch(sysMode){
        case MS_CALIBRATION:
          desiredIn = convertVelToInput(SLOW_VELOCITY);
          FSM_State = PULL_AND_COUNT;
          break;
        case MS_RUNNING:
          desiredIn = convertVelToInput(velRTL);
          FSM_State = PULL_STATE;
          break;
        default:
          Serial2.println("ERROR - ACK_COAST receievd but I don't know what to do with it");
          FSM_State = MS_IDLE;
          break; //do nothing
      }
      break;

    case DONE_PULLING:
      sendXBEEmsg(SWITCH_TO_COAST);
      FSM_State = WAIT_FOR_RESPONSE;
      break;  

    case LTR_VELOCITY_UPDATE:
      if(Serial.available > 2){
        processedMSG = procRecMsg();  
        clrXBEErecBuff();
        velLTR = processedMSG;
      }
      else{
        Serial2.println("Error - LTR velocity was not received");
      }
      break;
      
    case PULL_SLOWLY:
      sendXBEEmsg(ACK_PULL);
      desiredIn = convertVelToInput(SLOW_VELOCITY);
      FSM_State = PULL_STATE;
      break;

    case LTR_DELAY_UPDATE:
      if(Serial.available > 2){
        processedMSG = procRecMsg();  
        clrXBEErecBuff();
        LTR_delay = processedMSG;
      }
      else{
        Serial2.println("Error - LTR delay was not received");
      }
      break;
    default:
      break; //do nothing   
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

boolean openWebPage(char *Tfilename, EthernetClient cl){
  TServerFile = SD.open(Tfilename);        // open web page file
  if(TServerFile) {
    while(TServerFile.available()) {
       cl.write(TServerFile.read()); // send web page to client
      }
    TServerFile.close();
    return 0;

    }
  else{
    Serial2.println("ERROR - Can't openwebpage");
    return 1;
  }
  }

void StrClear(char *str, char str_length){ // sets every element of str to 0 (clears array)
    for (int i = 0; i < str_length; i++){ str[i] = 0;}
}


unsigned int StrContains(char *str, char *sfind){ // searches for the string sfind in the string str. returns the index of the last letter if string found, returns 0 if string not found
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
    cl.println("Connection: keep-alive");
  }
  else if(type == 'h'){ //"h" for html response 
    cl.println("HTTP/1.1 200 OK");
    cl.println("Content-Type: text/html");
    cl.println("Connection: close");
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

boolean respondXML(EthernetClient cl){
  cl.print("<?xml version = \"1.0\" ?>");
  cl.print("<Targuino>");
  cl.print("<RTL_Velocity>");
  cl.print(velRTL);
  cl.print("</RTL_Velocity>");
  cl.print("<LTR_Velocity>");
  cl.print(velLTR);
  cl.print("</LTR_Velocity>");
  cl.print("<Calibration>");
  if(calReq){ cl.print("ON");}
  else{       cl.print("OFF");}
  cl.print("</Calibration>");
  cl.print("<Run>");
  if(RunReq){ cl.print("ON");}
  else{       cl.print("OFF");}
  cl.print("</Run>");
  cl.print("<Status>");
  cl.print(TarStatus);
  cl.print("</Status>");
  cl.print("<XBEE_STATUS>");
  //if there is keep_alive
  cl.print("</XBEE_STATUS>");
  cl.print("</Targuino>");
  TarStatus = ""; //clears TarStatus after every time it is sent to the webserver
  return 0;
}

void coastMotor(){
  digitalWrite(pwmLPin,0);
  digitalWrite(pwmHPin,0);
}

void clrXBEEsendBuff(){
  msgToSend = {0, 0, 0};
}

void clrXBEErecBuff(){
  receivedMsg = {0, 0, 0};
}

void sendXBEEmsg(byte buff){
  msgToSend = {buff, buff, buff};
  Serial.write(msgToSend, 3);
}

byte procRecMsg(){
  Serial.readBytes(receivedMsg, 3);
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
  return ( vel / ropeRadius ) * 255 / maximumMotorVelocity;
}

struct parseHTTPmsg(char msg[]){
  String str(msg);
  struct output{byte msgType ; float velocity};
  //look for RTL velocity
  unsigned int ind = StrContains(str, MSG_RTL_VEL);
  if(ind){
    msgType = UPDATE_VELOCITY_RTL;
    velocity = parseFloat(str.substring(ind + 1, ind + 3));
    return output;
  }
  //look for LTR velocity
  ind = StrContains(str, MSG_LTR_VEL);
  if(ind){
    msgType = UPDATE_VELOCITY_LTR;
    velocity = parseFloat(str.substring(ind + 1, ind + 3));
    return output;
  }

  //look for BeginCalibration
  StrContains(str, MSG_CAL);
  if(ind){
    msgType = UPDATE_BEGIN_CALIBRATION;
    velocity = 99.99;
    return output;
  }

  //look for BeginRun
  ind = StrContains(str, MSG_RUN);
  if(ind){
    msgType = UPDATE_BEGIN_RUNNING;
    velocity = 99.99;
    return output;
  }

  //look for StopSignal
  ind = StrContains(str, MSG_STOP);
  if(ind){
    msgType = UPDATE_STOP;
    velocity = 99.99;
    return output;
  }

 //look for status
  ind = StrContains(str, MSG_STATUS);
  if(ind){
    msgType = UPDATE_STATUS;
    velocity = 99.99;
    return output;
  }
  
  //look for css file
  ind = StrContains(str, MSG_CSS);
  if(ind){
    msgType = UPDATE_CSS;
    velocity = 99.99;
    return output;
  }

  //look for js file
  ind = StrContains(str, MSG_JS);
  if(ind){
    msgType = UPDATE_JS;
    velocity = 99.99;
    return output;
  }
  
  //loog for jpg file
  ind = StrContains(str, MSG_JPG);
  if(ind){
    msgType = UPDATE_JPG;
    velocity = 99.99;
    return output;
  }
}

