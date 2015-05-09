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
#define SLOW_VELOCITY           6 //the motor use this velocity when moveing slowly from one side to the other. [rad/second]
#define MAX_PWM_VALUE           255
#define NOMINAL_TRACK_LENGTH    3000 //nominal length of the track (distance between motors) [cm]
#define SAFETY_DISTANCE         10 //safety distance from the edges in numOfmagnets

//XBEE
#define XBEE_DATA_RATE 57600

//Ethernet
#define REQ_BUF_SZ   60 // size of buffer used to capture HTTP requests



//Motor State description variables
boolean DIR                         = 0; // 1 - clockwise, 0 - counter-clockwise
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
 


}

//=====================MAIN=============================================
void loop(){
  digitalWrite(pwmHPin, false);
  digitalWrite(pwmLPin, false);
}//end loop
  
//=====================FUNCTIONS=============================================
