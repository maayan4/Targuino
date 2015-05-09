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
#define calButton		  29
#define runButton		  31
#define LED1			  35
#define LED2 	 		  37		


//Analog Pins
#define PSpin            A12 //motor driver power supply pin
#define CSPin            A13 //motor driver current sensor pin
#define potPin            A2 //potentiometer pin (optional)

//State machine
#define numOfStates       6
#define ZERO_STATE        1
#define MOVE_TO_ONE_SIDE  2
#define MEASURE_LENGTH    3
#define SAVE_DATA         4
#define RUN_TARGET        5
#define SLAVE_STATE       6

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
#define SLOW_VELOCITY           25 //the motor use this velocity when moveing slowly from one side to the other. [rad/second]
#define MAX_PWM_VALUE           255
#define NOMINAL_TRACK_LENGTH    3000 //nominal length of the track (distance between motors) [cm]
#define SAFETY_DISTANCE         10 //safety distance from the edges in numOfmagnets

//XBEE
#define XBEE_DATA_RATE 57600

//Ethernet
#define REQ_BUF_SZ   60 // size of buffer used to capture HTTP requests



//Motor State description variables
boolean DIR                         = 1; // 1 - clockwise, 0 - counter-clockwise
volatile unsigned int Revolutions   = 0; //number of revolutions. global variable needs to be defined as volatile in order to be used by interrupts
volatile unsigned int RunLength     = 0; //this variable counts revolutions during a run and compare it to lTrack [Revolutions]. It also delivers the length to lTrack during calibration
unsigned int lTrack                 = 0; //stores the length of the track (=distance between motors)
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
boolean stopSignal                  = false; //when this flag is true the system will stop moving and go to the next stage

//PID Variables
double desiredIn  = 0; // this variable stores the desired velocity in PWM range (0-255)
double sysIn      = 0; //the calculated input to the driver
PID myPID(&measuredIn, &sysIn, &desiredIn, Kp, Ki, Kd, DIRECT);

//system state variables
boolean MS              = 0; //is arduino in Master or Slave mode (0 for slave mode, 1 for master)
byte FSM_State          = 0; //system's currect state in the state machine. default value is 0 - initial state (see #define)
boolean SetupReq        = 0; //flag indicating if a setup request was received  
boolean RunReq          = 0; //flag indicating if a run request was received

//user request variables
double reqVel           = 0; //the velocity the user requested for the run
char inMsg[5]           = "";

//Ethernet
byte mac[] 						= {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress 				        ip(192, 168, 0, 177);
EthernetServer   			    server(80);
char HTTP_req[REQ_BUF_SZ] 		= {0};  	// 0 = null in ascii
char req_index   				= 0;       // index into HTTP_req buffer

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
  Serial2.begin(             XBEE_DATA_RATE); 

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
  //beginXbee();
  
  if(MS){
    //initialize Ethernet server
    Ethernet.begin(mac, ip);
    server.begin();
  	initSD();
  }
  
  myPID.SetOutputLimits(0,255);  //tell the PID to range between 0 and 255 (PWM range)    // some arbitrary value I've decided of.
  myPID.SetMode(    AUTOMATIC); //turn the PID on
  myPID.SetSampleTime(    200);

  maximumRunTime = float(NOMINAL_TRACK_LENGTH / SLOW_VELOCITY / ropeRadius) * 1000;

  //setup interrupt for the magnet sensor
  attachInterrupt(4,IncrRevolution,RISING); //interrupt 4 is for pin number 19 - magnet sensor input

  PreviousInterruptTime = millis(); //start timer

}

//=====================MAIN=============================================
void loop(){
  switch (FSM_State) {
       case ZERO_STATE:
        if(tmpFlag){
          //Serial.println("i'm in state: zero! waiting for instructions");
          tmpFlag = false;
          digitalWrite(LED1, LOW);	
  		  digitalWrite(LED2, LOW);
        }        
         if(!MS){ //if arduino in slave mode - go to zero_state_slave
          //Serial.println("going into state: zero slave");
          FSM_State = SLAVE_STATE;
         }
         else{ //if arduino is master
         if(SetupReq || RunReq){
          FSM_State = MOVE_TO_ONE_SIDE;
          tBeginMovement = millis(); //start watchdog for the target movement
          desiredIn = map(SLOW_VELOCITY, 0, maximumMotorVelocity, 0, MAX_PWM_VALUE); //set desired velocity before starting to move target
          //Serial.println("going into state: moving to one side");
          tmpFlag = true; //to get back into the condition on the beginning of the state when we get back
         }
        if(digitalRead(calButton) == HIGH){
        		SetupReq = true;
        		digitalWrite(LED1, HIGH);
        		delay(500);	
         }
        if(digitalRead(runButton) == HIGH){
            RunReq = true;
        	digitalWrite(LED1, HIGH);	
        	digitalWrite(LED2, HIGH);
        	delay(500);
        }
	 }
         break;
       
       case MOVE_TO_ONE_SIDE:
         timeGuard = millis() - tBeginMovement;
         if(digitalRead(calButton) == HIGH || digitalRead(runButton) == HIGH){
         	stopSignal = true;
         	delay(500);
         }
         if(timeGuard > maximumRunTime || stopSignal){ //if target arrived to one of the sides
          //Serial.print(" I think i'm at the other side! it took me "); Serial.print(timeGuard); Serial.println(" [ms] to get there...");
          stopMotors();
          DIR = !DIR; //change direction of motors
          if(SetupReq){
            digitalWrite(LED1, LOW);	
        	digitalWrite(LED2, LOW);
        	delay(1000);
        	digitalWrite(LED1, HIGH);	
        	digitalWrite(LED2, LOW);    
            FSM_State = MEASURE_LENGTH;
            RunLength = 0; //resets measurement of the distance the target has moved before run 
            tBeginMovement = millis(); //start watchdog for the target movement
            //Serial.println("going into state: measuring length! measuring..."); 
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
         if(checkTime(VMEASURE_SAMPLE_TIME)){  MeasureVelocity(VMEASURE_SAMPLE_TIME);}
         myPID.Compute();
         movemotorB(DIR,sysIn,MAX_PWM_VALUE);
         movemotor(DIR,sysIn,MAX_PWM_VALUE);
         break;
       
       case MEASURE_LENGTH:
        //Serial.print("DesiredIn: "); Serial.print(desiredIn); Serial.print(" MeasuredIn: "); Serial.println(measuredIn);
        timeGuard = millis() - tBeginMovement;
         if(digitalRead(calButton) == HIGH || digitalRead(runButton) == HIGH){
         	stopSignal = true;
         	delay(500);
         }
         if(timeGuard > maximumRunTime || stopSignal){ // target probably arrived to the other edge of the rope 
          //Serial.print("i'm at the end! Distance is now: "); Serial.print(RunLength); Serial.println(" revolutions. stopping...");
          stopMotors();
          FSM_State = SAVE_DATA;
          stopSignal = false; //reset stopSignal for the future
          break;
        }
         if(checkTime(VMEASURE_SAMPLE_TIME)){  MeasureVelocity(VMEASURE_SAMPLE_TIME);}
         myPID.Compute();
         movemotorB(DIR,sysIn,MAX_PWM_VALUE);
         movemotor(DIR,sysIn,MAX_PWM_VALUE);
         break;
       
       case SAVE_DATA:
         //Serial.print("i'm in state: saving data! length of track is: "); Serial.print(RunLength); Serial.println(" revolutions. Saving...");
         lTrack = RunLength;
         //save lTrack to EEPROM
         SetupReq = 0;
         FSM_State = ZERO_STATE;
         break;
       
       case RUN_TARGET:
         //Serial.print("i'm in state: starting a run! distance target has gone is: "); Serial.print(RunLength); Serial.print("/"); Serial.print(lTrack); Serial.println(" revolutions. running...");
         distance_from_edge = lTrack - RunLength;
         //Serial.println(distance_from_edge);
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
         if( distance_from_edge < SAFETY_DISTANCE || stopSignal){ //if target arrived to the other side 
          //Serial.print("i'm close to the edge! Distance is now: "); Serial.print(RunLength); Serial.println(" revolutions. stopping...");
          stopMotors();
          FSM_State = ZERO_STATE;
          RunReq = 0;
          stopSignal = false; //reset stopSignal for the future
          break;
        }
         if(checkTime(VMEASURE_SAMPLE_TIME)){  
         	MeasureVelocity(VMEASURE_SAMPLE_TIME);

         }
          myPID.Compute();
          movemotorB(DIR,sysIn,MAX_PWM_VALUE);
          movemotor(DIR,sysIn,MAX_PWM_VALUE);
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
  	while (client.connected()) {
          if (client.available()) {   // client data available to read
  			char c = client.read(); // read 1 byte (character) from client
              if (req_index < (REQ_BUF_SZ - 1)){     // limit the size of the stored received HTTP request.  buffer first part of HTTP request in HTTP_req. leave last element in array as 0 to null terminate string (REQ_BUF_SZ - 1)
                  HTTP_req[req_index] = c;          // save HTTP request character
                  req_index++;
              	}
              // last line of client request is blank and ends with \n
              // respond to client only after last line received
              if (c == '\n' && currentLineIsBlank) {
                  // send a standard http response header
                  client.println("HTTP/1.1 200 OK");
                  // remainder of header follows below, depending on if
                  // web page or XML page is requested
                  // Ajax request - send XML file
                  unsigned int ind = StrContains(HTTP_req, "SaveVel=");
                  if(ind){
                  	 reqVel = 60;
                  	 //reqVel = float(HTTP_req[ind+1]+HTTP_req[ind+2]+ /  ropeRadius; 
                  	client.println("Content-Type: text/html");
                      client.println("Connection: keep-alive");
                      client.println();
                  }
                  if (StrContains(HTTP_req, "status")) {
                      // send rest of HTTP header
                      client.println("Content-Type: text/xml");
                      client.println("Connection: keep-alive");
                      client.println();
                      client.println(FSM_State);
                      //SetLEDs();
                      // send XML file containing input states
                      //XML_response(client);
                  	}
                  else {  // web page request
                      // send rest of HTTP header
                      client.println("Content-Type: text/html");
                      client.println("Connection: keep-alive");
                      client.println();
                      // send web page
                      openWebPage("index.htm", client);
                      
                      //Serial.print(HTTP_req); // display received HTTP request on serial port

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
      //Serial.println("Connection closed");
      //Serial.println(req_index);
  	} //end if(client)
  }
}//end loop
  
//=====================FUNCTIONS=============================================
boolean movemotor(boolean DIR, int pwmH, int pwmL){ //set motor velocity and direction
  
  if(pwmH > 255 || pwmL >255){  
  	pwmH = 0;
  	} //illigal values
  
  digitalWrite(dirPin,DIR);
  analogWrite(pwmHPin, pwmH);
  analogWrite(pwmLPin, pwmL);
  return 0;
}

boolean movemotorB(boolean DIR, int pwmH, int pwmL){ //send motor command to the slave arduino
  if(MS){ 
  Serial.print('d');
  delay(100);
  Serial.print(DIR);
  delay(100);
  Serial.print('i');
  delay(100);
  Serial.print(pwmH);
  delay(100);
}
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
  char temp_c;
  int temp_i;
  if(!MS){
	  if(Serial.available()) {
      // get the new byte:
      temp_c = (char)Serial.read();
	  temp_i = Serial.parseInt();

	  //Serial.print(temp_c);
	  //Serial.println(temp_i);

	   switch(temp_c){
	    case 'd': 
		  DIR = temp_i;
	      break;
	    case 'i': 
          sysIn = temp_i;
          //Serial.println(sysIn);
          movemotor(DIR,sysIn,MAX_PWM_VALUE);
	      break;
	    default:
	      break;    	
	  	}
	  }
	}
}
  
void stopMotors(){
  movemotorB(DIR,999,MAX_PWM_VALUE);
  movemotor(DIR,0,MAX_PWM_VALUE);
}

boolean beginXbee() {
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
}

boolean initSD(){ //initialize SD card
    //Serial.println("Initializing SD card...");
    if (!SD.begin(4)) {
        //Serial.println("ERROR - SD card initialization failed!");
        return 1;    // init failed
    }
    //Serial.println("SUCCESS - SD card initialized.");
    // check for index.htm file
    if (!SD.exists("index.htm")) {
        //Serial.println("ERROR - Can't find index.htm file!");
        return 1;  // can't find index file
    }
    //Serial.println("SUCCESS - Found index.htm file.");
}

boolean openWebPage(char *Tfilename, EthernetClient cl){
  	TServerFile = SD.open(Tfilename);        // open web page file
	if(TServerFile) {
	  while(TServerFile.available()) {
	     cl.write(TServerFile.read()); // send web page to client
	    }
	  TServerFile.close();
	  }
	  return 0;
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
    
    if (strlen(sfind) > len){ return 0; }
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