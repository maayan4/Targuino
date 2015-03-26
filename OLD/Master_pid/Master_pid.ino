////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int mastermotorspeed = 70;           // speed of the mastermotor!!!!!!!!!!!!!!  <<< change here!!!!!!!!!!!!1
int brakemoterspeed = 50;            // intensity of braking
int calibrationbreakspeed = 30;      // intensity of calibration main motor breaking
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// debug flags
//////////////////////////////////////////////////////////
int maindebug = 0;

int buttonprobedebug = 0;

int magnetprobedebug = 0;

int PIDprobedebug = 0;

int callibrationdebug = 0; // supresses 3999 out of 4000 messages


unsigned int debugcounter = 0;


// data to slave
////////////////////////////////////////////////////////////////////////////////////
char startcommand = 'Y';             // send to slave to start motor

char stopcommand = 'N';              // send to slave to stop motor

char acnowledgecommand = 'R';        // send and receive for acnowledge

char calibrationbreak = 'b';         // send calibration break

char normalbreak = 'B';              // send normal operation break
////////////////////////////////////////////////////////////////////////////////////



int circuitlength = 0;               // this will store the length in pulses of the circuit - and will hold 0 when calibration is needed
int calibrationbuttoncounter = 0;    // counts how many time button was pressed when needing callibration
int runcount = 0;                    // keeps track of target location

//button probe
unsigned long buttonofftimer = 0;              // used to enforce minimum time between button pressings 
unsigned long buttonmintimer = 0;              //  button press time

//magnet probe
unsigned long magnetmintimer = 0;  
unsigned long magnetofftimer = 0;

//PID probe
unsigned long PIDmintimer = 0;  
unsigned long PIDofftimer = 0;


unsigned long noevery0dot1sec = 0;             // timer for sending stop to slave every 0.1 seconds
unsigned long delaybetweenruns = 0;            // timer for counting delays between the end of one run and the start of the other


int tripcurrent = 630;
int buttonState = 0;

int firsttimerunning = 1;

int MotorPWMPin=9;                                      // pwm fo engine
int OverCurrentCount=5;                                                    //over current number>3
int driverresetpin=22;                                  //modul for the driver
int ff1 = 26;                                               //LOW	Fault flag 1 indicator: FF1 goes high when certain faults have occurred. See table below for details.
int ff2 = 28;                                               //LOW	Fault flag 2 indicator: FF2 goes high when certain faults have occurred. See table below for details.
int button = 48;                                       //start button
int magnet = 31;

int serialmode = 0;  // debug mode!!!!!!!!!!!

int MotorSpeed = 80; //between 0-255;

char incomingByte;                    //Communication

int moving = 0;

#include "functions.h" // messy functions
void setup() 
{
  pinMode(ff1, INPUT); //not in use 
  pinMode(ff2, INPUT); //not in use
  pinMode(driverresetpin, OUTPUT); // reset pin for the driver
  pinMode(button, INPUT_PULLUP); 
  pinMode(magnet, INPUT_PULLUP);   
  pinMode(MotorPWMPin, OUTPUT);

  digitalWrite(driverresetpin, HIGH);
  analogWrite(MotorPWMPin,0);

  Serial.begin(57600);

  Serial.println('N');  // stop slave motor
  if (!maindebug) // DEBUGG!!
  {
    Serial.println();
  }

  analogWrite(MotorPWMPin, 0);  // stop main motor
}


//
//                VOID LOOOP
//
/////////////////////////////////////////////


void loop() 
{

  // calibration run OR "press button 3 times to start!!!"
  ///////////////////////////
  while (circuitlength == 0) 
  {


    if( buttonprobe(button) )
    {
      calibrationbuttoncounter =  calibrationbuttoncounter + 1;

      if (maindebug)   // DEBUGG!!!
      {
        Serial.println("inside first while: button was pressed"); 
        Serial.print("button counter = "); 
        Serial.println(calibrationbuttoncounter);
        Serial.println();
      }
    }

    if (calibrationbuttoncounter == 3)
    {
      {
        Serial.println("we have 3 presses, entering callibration"); 
      }

      circuitlength = calibration();
    }
    if (maindebug)   // DEBUGG!!!
    {
      debugcounter = debugcounter +1;

      if ( debugcounter == 40000) // post stats every 1000 cycles
      {
        Serial.println("inside first while"); 
        Serial.print("button counter = "); 
        Serial.println(calibrationbuttoncounter);
        Serial.println();
        debugcounter = 0;
      }
    }

  } // end of calibration run

  if (maindebug)   // DEBUGG!!!
  {
    Serial.println("debug ended "); 
    Serial.print("pulse counts = "); 
    Serial.println(circuitlength);
    Serial.println();
    Serial.println("now waiting for button imput to start operation... ");
  }

  while ( !( buttonprobe(button) ) )  // waiting while button not pressed
  {
  }

  Serial.println('Y');  // start slave motor 
  if (maindebug)   
  {   
    Serial.println();
  }

  while ( runcount < circuitlength) // while target not reached circuit length
  {
    if (firsttimerunning)
    {
      firsttimerunning = 0;
      circuitlength = circuitlength - 40; // remove 2 meters from circuit length 
      runcount = -20;
    }
    runcount = runcount + magnetprobe(magnet);

    if ( (runcount + 20 ) > circuitlength )
    {
      analogWrite(MotorPWMPin, brakemoterspeed);  // master motor brake
    }  

  }
  
  Serial.println('N'); // stop slave motor
  if (maindebug)   
  {
    Serial.println();
  }

  delaybetweenruns = millis();

  while ( ( millis() - delaybetweenruns ) < 2000 ) // 2000 millis between stop of slave and start of master motor to be sure everything stopped
  {
    runcount = runcount + magnetprobe(magnet);
  }

  // start main motor
  analogWrite(MotorPWMPin, mastermotorspeed);


  while ( runcount > 0 ) // while target not reached circuit length
  {

    runcount = runcount - magnetprobe(magnet);

    if ( (runcount - 20 ) < 0  )
    {
      Serial.println('B');  // slave motor brake
      if (maindebug)   
      {
        Serial.println();
      }
    }

  }
  analogWrite(MotorPWMPin, 0); // stop main motor

  delaybetweenruns = millis();


  while ( ( millis() - delaybetweenruns ) < 2000 ) // 2000 millis between stop of slave and start of master motor to be sure everything stopped
  {
    runcount = runcount + magnetprobe(magnet);
  }

}










