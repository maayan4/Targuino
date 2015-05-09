#include <PID_v1.h>
#include <SPI.h>
#include <Ethernet.h>
#include <SD.h>

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

//leds and temporary buttons/potentiometers
/*#define calButton		  29
#define runButton		  31
#define LED1			  35
#define LED2 	 		  37	*/	

//Analog Pins
#define PSpin            A12 //motor driver power supply pin
#define CSPin            A13 //motor driver current sensor pin
//#define potPin            A2 //potentiometer pin (optional)

//State machine
#define numOfStates         8
#define ZERO_STATE          0x01
#define WAIT_FOR_RESPONSE   0x02
#define PULL_MODE           0x03
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

//Scenarios
#define KEEP_RUNNING              0
#define RIGHT_TO_LEFT             1
#define LEFT_TO_RIGHT             2

//Other Constants
#define pi                      3.14159
#define radius                  6.5 //[cm] radius of the wheel. represents velocity at the end point
#define ropeRadius              2.23 // [cm] represent velocity where the rope is
#define numOfMagnets            2 //number of magnets available on the motor
#define maximumMotorVelocity    157.08  //maximum motor velocity [rad/second] = 2*pi*1500[RPM]/60[seconds]   
#define VMEASURE_SAMPLE_TIME    500 //what is the timeframe for velocity measurement. [ms]
#define SLOW_VELOCITY           6 //the motor use this velocity when moveing slowly from one side to the other. [rad/second]
#define MAX_PWM_VALUE           255
#define NOMINAL_TRACK_LENGTH    3000 //nominal length of the track (distance between motors) [cm]
#define SAFETY_DISTANCE         10 //safety distance from the edges in numOfmagnets
#define DELAY_BETWEEN_RUNNINGS  2000 //time to wait between runnings in [ms]

//XBEE
#define XBEE_DATA_RATE 57600
//XBEE messages
//Master->slave
#define GOTO_CALIBRATION    0x11
#define GOTO_RUN            0x12
#define GOTO_IDLE           0x13
#define SWITCH_TO_COAST     0x14
#define KEEP_ALIVE          0x15
//Slave->master
#define ACK_CALIB           0x21
#define ACK_RUN             0x22
#define ACK_IDLE            0x23
#define ACK_COAST           0x24
#define DONE_PULLING        0x25
#define ACK_KA              0x26 

//Ethernet
#define REQ_BUF_SZ          100 // size of buffer used to capture HTTP requests
#define WEBSITE_FILENAME    "targuino.htm" //name of the main html file on the SD card
#define JAVASCRIPT_FILENAME "targuino.js" //javascript filename
#define CSS_FILENAME        "targuino.css" //css filename

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
byte currentScenario    = KEEP_RUNNING; //this variable keeps the current chosen scenario. default is to keep running from side to side
float velLTR            = 0; //the velocity the user has requested for the run from left to right
float velRTL            = 0; //the velocity the user has requested for the run from right to left
String TarStatus        = ""; //string that contains system status
boolean openGate        = false; //a general flag which uses to control whether the system goes to the next stage

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
              PrevKAmsgSent = CurrentTime;
              numOfKAmsgs++; //this number is zeroed if we get KA-ACK (this is handled in SerialEvent)
            }
            if(numOfKAmsgs > KA_WATCHDOG){
              TarStatus += "XBEE Slave error - no response from slave";
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
            sendXBEEmsg(GOTO_RUN);
            Serial2.println("GO_TO_CALIB msg sent. waiting for response...");
            FSM_State = WAIT_FOR_RESPONSE;
            break;
          case SL_RUNNING:
            sendXBEEmsg(ACK_RUN);
            Serial2.println("ACK_RUN msg sent. waiting for response...");
            break;
          default:
            break; //do nothing
        }
        break;
       
       case WAIT_FOR_RESPONSE:
        //this is an empty state, where the controller just wait for response. In case of response SerialEvent would update FSM_STATE we'll move on.
        break;

       case MOVE_TO_ONE_SIDE:
         timeGuard = millis() - tBeginMovement;
         if(digitalRead(calButton) == HIGH || digitalRead(runButton) == HIGH){
         	stopSignal = true;
         	delay(500);
         }
         if(timeGuard > maximumRunTime || stopSignal){ //if target arrived to one of the sides
          Serial2.print(" I think i'm at the other side! it took me "); Serial2.print(timeGuard); Serial2.println(" [ms] to get there...");
          stopMotors();
          DIR = 1; //change direction of motors
          if(calReq){
            digitalWrite(LED1, LOW);	
        	  digitalWrite(LED2, LOW);
        	  delay(1000);
        	  digitalWrite(LED1, HIGH);	
        	  digitalWrite(LED2, LOW);    
            FSM_State = MEASURE_LENGTH;
            RunLength = 0; //resets measurement of the distance the target has moved before run 
            tBeginMovement = millis(); //start watchdog for the target movement
            Serial2.println("going into state: measuring length! measuring..."); 
          }
          else if(RunReq){
            digitalWrite(LED1, LOW);	
          	digitalWrite(LED2, LOW);
          	delay(1000);
          	digitalWrite(LED1, HIGH);	
          	digitalWrite(LED2, HIGH);
            FSM_State = RUN_TARGET;
            RunLength = 0; //resets measurement of the distance the target has moved before run 
            desiredIn = map(reqVel, 0, maximumMotorVelocity, 0, MAX_PWM_VALUE);//set desired velocity before starting to move target
          }
          else{            
            FSM_State = ZERO_STATE; //if something is wrong - go back to zero state
            } 
          Revolutions = 0; //reset number of revolutions
          stopSignal = false; //reset stopSignal for the future
          break;
         }
         if(checkTime(VMEASURE_SAMPLE_TIME)){  
         	MeasureVelocity(VMEASURE_SAMPLE_TIME);
         	myPID.Compute();
         	movemotorB(DIR,sysIn,MAX_PWM_VALUE);
         	movemotor(DIR,sysIn,MAX_PWM_VALUE); 
         }
         
         break;
       
       case MEASURE_LENGTH:
        Serial2.print("DesiredIn: "); Serial.print(desiredIn); Serial.print(" MeasuredIn: "); Serial.println(measuredIn);
        timeGuard = millis() - tBeginMovement;
         if(digitalRead(calButton) == HIGH || digitalRead(runButton) == HIGH){
         	stopSignal = true;
         	delay(500);
         }
         if(timeGuard > maximumRunTime || stopSignal){ // target probably arrived to the other edge of the rope 
          //Serial.print("i'm at the end! Distance is now: "); Serial.print(RunLength); Serial.println(" revolutions. stopping...");
          stopMotors();
          FSM_State = SAVE_DATA;
          DIR = 0;
          stopSignal = false; //reset stopSignal for the future
          break;
        }
         if(checkTime(VMEASURE_SAMPLE_TIME)){  
         	MeasureVelocity(VMEASURE_SAMPLE_TIME);
         	myPID.Compute();
         	movemotorB(DIR,sysIn,MAX_PWM_VALUE);
               movemotor(DIR,sysIn,MAX_PWM_VALUE);
               Serial.println(sysIn);
         }
         break;
       
       case SAVE_DATA:
         Serial2.print("i'm in state: saving data! length of track is: "); Serial2.print(RunLength); Serial2.println(" revolutions. Saving...");
         lTrack = RunLength;
         //save lTrack to EEPROM
         calReq = 0;
         FSM_State = ZERO_STATE;
         break;
       
       case RUN_TARGET:
         Serial2.print("i'm in state: starting a run! distance target has gone is: "); Serial2.print(RunLength); Serial2.print("/"); Serial2.print(lTrack); Serial2.println(" revolutions. running...");
         distance_from_edge = lTrack - RunLength;
         Serial2.println(distance_from_edge);
         if(digitalRead(calButton) == HIGH || digitalRead(runButton) == HIGH){
         	stopSignal = true;
         	delay(500);
         }
         if(distance_from_edge < (2 * SAFETY_DISTANCE)){
          desiredIn = 0.8 * desiredIn; //reducing velocity towards the edge
          digitalWrite(LED1, LOW);	
        	digitalWrite(LED2, LOW);
        	delay(500);
        	digitalWrite(LED1, HIGH);	
        	digitalWrite(LED2, HIGH);
			    delay(500);
        	digitalWrite(LED1, LOW);	
        	digitalWrite(LED2, LOW);
        	delay(500);
        	digitalWrite(LED1, HIGH);	
        	digitalWrite(LED2, HIGH);
         }
         if(distance_from_edge < SAFETY_DISTANCE || stopSignal){ //if target arrived to the other side 
          Serial2.print("i'm close to the edge! Distance is now: "); Serial2.print(RunLength); Serial2.println(" revolutions. stopping...");
          stopMotors();
          FSM_State = ZERO_STATE;
          RunReq = 0;
          DIR = 0;
          stopSignal = false; //reset stopSignal for the future
          break;
        }
         if(checkTime(VMEASURE_SAMPLE_TIME)){  
         	MeasureVelocity(VMEASURE_SAMPLE_TIME);
         	myPID.Compute();
                movemotorB(DIR,sysIn,MAX_PWM_VALUE);
                movemotor(DIR,sysIn,MAX_PWM_VALUE);
         }

         break;
       
       case SLAVE_STATE:
         break;  
       
       default:
         FSM_State = ZERO_STATE;
         stopMotors(); //just in case something happens...
         break;
	} 
  //current sensor
  sensorValue = analogRead(CSPin);  
  current = ( map(sensorValue, 0, 1023, 0, 5) - 2.5) / 0.066;
  reqVel = analogRead(potPin) * maximumMotorVelocity / 1023;

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
              unsigned int ind = StrContains(HTTP_req, "SaveVel=");
              if(ind){
              	reqVel = float(HTTP_req[ind+1]+HTTP_req[ind+2])/ropeRadius;
                sendHTTPResponse('h',client);
                client.println(reqVel);
              }
              if(StrContains(HTTP_req, "status")) {
                sendHTTPResponse('x',client);  
                respondXML();
                  // send XML file containing input states
                  //XML_response(client);
              }
              else{  // web page request
                // send rest of HTTP header
                sendHTTPResponse('h',client);
                // send web page
                openWebPage("index.htm", client);
                Serial2.print(HTTP_req); // display received HTTP request on serial port
                req_index = 0;// reset buffer index and all buffer elements to 0
                StrClear(HTTP_req, REQ_BUF_SZ);
                break;
              }
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
  vAngular=2*pi*1000*Revolutions/(numOfMagnets*deltaT); //multiply 1/period by 1000 to get the time in seconds and the velocity in Hz
    
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
      
      break;

    case GOTO_RUN:
      sysMode = SL_RUNNING;
      
      break;

    case GOTO_IDLE:
      ///////
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
      ///////
      break;

    case ACK_RUN:
      ///////
      break;

    case ACK_IDLE:
      ///////
      break; 

    case ACK_COAST:
      switch(sysMode){
        case MS_CALIBRATION:
          break;
        case MS_RUNNING:
          break;
        case SL_RUNNING:
        default:
          break; //do nothing
      }
      break;

    case DONE_PULLING:
      ///////
      break;  
    default:
      break; //do nothing   
  }
}
  
void stopMotors(){
  movemotor(DIR,0,MAX_PWM_VALUE);
}

/*boolean beginXbee() {
  Serial.print("+++");
  delay(1000);
  if (Serial.available() > 0) {
    //digital/Write(ledPin,HIGH);
    //String inStr(2);
    char inStr[2];
    Serial.readBytes(inStr,2);
    if(inStr == "OK"){
      //digitalWrite(ledPin,HIGH);
    }
  }
}*/

boolean initSD(){ //initialize SD card
    if (!SD.begin(4)) {
        Serial2.println("ERROR - SD card initialization failed!");
        return 1;    // init failed
    }
    Serial2.println("SUCCESS - SD card initialized.");
    // check for index.htm file
    if (!SD.exists(WEBSITE_FILENAME)) {
        Serial2.println("ERROR - Can't find index.htm file!");
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
  }
  else if(type == 'h'){ //"h" for html response 
    cl.println("HTTP/1.1 200 OK");
    cl.println("Content-Type: text/html");
  }
  else{
    return 1;
  }
  cl.println("Connection: keep-alive");
  cl.println();
  return 0;
}

boolean respondXML(EthernetClient cl){
  cl.print("<?xml version = \"1.0\" ?>");
  cl.print("<Targuino>");
  cl.print("<Scenario>");
  switch(currentScenario){
    case KEEP_RUNNING: 
      cl.print("KR");
      break;
    case RIGHT_TO_LEFT:
      cl.print("RTL");
      break;
    case LEFT_TO_RIGHT:
      cl.print("LTR");
      break;
    }
  cl.print("</Scenario>");
  cl.print("<Velocity>");
  cl.print(reqVel);
  cl.print("</Velocity>");
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

