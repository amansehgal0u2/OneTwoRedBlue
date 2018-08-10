//
// University of Pennsylvania
// ESE 505 - 2015C
// Project 2
// OneTwoRedBlue.ino
//
// Headers contain device drivers
//
#define AUTO 1

#include <SPI.h>  
#include <Pixy.h>
#include <Servo.h>
const int ONE = -28;
const int TWO = 32;
const int RED = -56;
const int BLUE = 60;
const int ONE_SECOND  = 50;
const int FIVE_SECONDS = ONE_SECOND*5;

//
// Pixy is an object type defined in Pixy header file
// Servo is an object type defined in Servo header file
//
Pixy PingPongPixy;
Servo PingPongServo;

//
// here are the pins used
//
const int PingPongServoPin = 3;
const int JoystickPin = A0;

//
// servo constants -- YOU MAY CHANGE THESE TO GET GOOD CALIBRATION
//
const int ServoCenter_us = 1438;
const double ServoScale_us = 8.0;    // micro-seconds per degree

void setup()
{
//
// Serial baud rate -- faster is usually better
//
  Serial.begin(57600);
//
// Servo attached to Arduino digital pin 3
//
  PingPongServo.attach(3);
  PingPongServo.writeMicroseconds(ServoCenter_us);
//
// initialize the Pixy camera
//
  PingPongPixy.init();
}

void loop()
{ 
  static unsigned long microsLast = 0; // elapsed time at end of last call to loop
  double deltaT = 0.02; // time between digital updates (20ms = PIXY & Servo update rates)

  word JoystickRaw = 512;
  double JoystickPercent = 0.0;
  static int xBallRawLast = 2;  // start with max left position as indication of failed PIXY read
  int xBallRaw;
  double xBallmm = 0.0;
  double ServoDeg = 0.0;
  double t_us;
  ///////////////////////////////////////////
  //new variables added
  ///////////////////////////////////////////
  double tau = 0.060;// filter time constant
  double alpha = deltaT/(tau+deltaT); //Discrete LPF filter weight
  boolean ServoDisabled;
  static double errorIntegral = 0.0;
  static double errorLast = 0.0;
  static double xballmmFilteredPercentLast = 0.0;
  static double xBallmmPercentLast = 0.0;
  static double xballmmFilteredLast = 0.0;
  static  int wait = ONE_SECOND;//startup wait
  static double xballmmAutoIn = 0.0;
  ///////////////////////////////////////////
  // control loop begins here
  ///////////////////////////////////////////
//
// get the joystick position
// Convert to +/- 100% relative to center
//

#if AUTO
// Automatic control  
  if(wait-- <= 0)//wait on power up to stabilize
  {
//    xballmmAutoIn = ONE;
    //time to pick a new input 
    switch((int)xballmmAutoIn)
    {
      case ONE:
      {
        xballmmAutoIn = (double)TWO;
        wait = 2*ONE_SECOND;
      }break;
      case TWO:
      {
        xballmmAutoIn = (double)RED;
        wait = 2*ONE_SECOND;
      }break;
      case RED:
      {
        xballmmAutoIn = (double)BLUE;
        wait = FIVE_SECONDS;
      }break;
      case BLUE:
      {
        xballmmAutoIn = (double)ONE;
        wait = 2*ONE_SECOND;
      }break;
      default: // only goes here on startup
      {
        xballmmAutoIn = (double)ONE; //start the sequence
        wait = ONE_SECOND; //reset timer
      }break;
    }
  }
#else
// Manual control
  JoystickRaw = analogRead(JoystickPin);
  JoystickPercent = -100.0 + (200.0*(JoystickRaw))/1023.0;
#endif
//
// get the ball position
// negative value indicates no ball found
// convert distance in mm from center of camera field of view
// YOU SHOULD FIX THE DISTANCE CALIBRATION!
//
  xBallRaw = PingPongCheck();
  if (xBallRaw < 0)
  {
    xBallRaw = xBallRawLast;
  }
  else
  {
    xBallRawLast = xBallRaw;
  }
  xBallmm = -111.0 + (222.0*xBallRaw)/319.0;  // 319 = max pixel value in PIXY camera


//////////////////////////////////////////////////////////////////
// Filter PIXY data using low pass filter
//////////////////////////////////////////////////////////////////

//Filter xBallmm
double xballmmFiltered = ((1-alpha)*xballmmFilteredLast - alpha*xBallmm);
xballmmFilteredLast = xballmmFiltered;

#if AUTO
  double xballmmFiltered_feedback = -1*xballmmFiltered;
#endif
//////////////////////////////
// convert ball pos to percent
//////////////////////////////
//filter negates the value of the xBallmm input. Subtract from 100 to renegate the value
double xballmmFilteredPercent = 100 - ((72.8 -(-1*xballmmFiltered))/(72.8 + 66.5)*200);
double xBallmmPercent = 100 - ((72.9 -xBallmm)/(72.9 + 67.9)*200) ;


//////////////////////////////////////////////////////////////////
// Filter Joystick Percent data using low pass filter
//////////////////////////////////////////////////////////////////
//static double JoystickFilteredLast = 0.0;
//static double JoystickFiltered = 0.0;
////Filter Joystick Percent
//JoystickFiltered = ((1-alpha)*JoystickFilteredLast - alpha*JoystickPercent);
//JoystickFilteredLast = JoystickFiltered;
// apply trim to one side

///////////////////////////
//Filtered stuff here
///////////////////////////

// double error = -1*JoystickFiltered - xballmmFilteredPercent;
// double errorBallPosPercent = xballmmFilteredPercent - xballmmFilteredPercentLast;
// xballmmFilteredPercentLast = xballmmFilteredPercent;

///////////////////////////
//  UNFiltered stuff here
///////////////////////////
#if AUTO 
 double error = xballmmAutoIn - xballmmFiltered_feedback;
#else
 double error = JoystickPercent - xballmmFilteredPercent;
#endif
// double errorxBallmmPercent = xBallmmPercent - xBallmmPercentLast;
// xBallmmPercentLast = xBallmmPercent;
 double errorDelta = error - errorLast;//for derivative
 errorLast = error;
 

 //////////////////////////////////////////////////////////////////
 // Control Law Starts Here
 //////////////////////////////////////////////////////////////////
 double Kp = .03, Ki = 0.02, Kd = 0.06;

 //////////////////////
 // Error Integral
 //////////////////////
 
 errorIntegral = errorIntegral + error*deltaT;
 // integrator anti windup
 if(abs(errorIntegral) > 100 )
 {
   errorIntegral = 100;
 }
#if AUTO 
  ServoDeg = 0.05*xballmmAutoIn + Kp*(error) + Ki*(errorIntegral) + Kd*(errorDelta/deltaT) ;  // PID control with feedforward
#else
  ServoDeg = 0.02*JoystickPercent + Kp*(error) + Ki*(errorIntegral) + Kd*(errorDelta/deltaT) ;  // PID control with feedforward  
#endif



//////////////////////////////////////////////////////////////////
// Control Law Ends Here
//////////////////////////////////////////////////////////////////
//ServoDeg = 0.2*-1*JoystickFiltered;
// ServoDeg =  0.2*JoystickPercent;
//
// convert ServoDeg to MicroSeconds
// then send the command to the servo
// smaller values of t_us tilt right (positive sense of ServoOne)
// DON'T CHANGE LIMITS IN CONSTRAIN FUNCTION -- CAN DAMAGE SERVO
//
  t_us = constrain(ServoCenter_us - ServoScale_us * ServoDeg, 1000, 1800);

 
  PingPongServo.writeMicroseconds(t_us);

//
// now send serial information
//
//  Serial.print(JoystickPercent,1);
//  Serial.print(',');  
//  Serial.print(-1*JoystickFiltered,1);
//  Serial.print(',');  
//    Serial.print(xballmmFilteredPercent,1);
//    Serial.print(',');
//    Serial.println(error,1);
    Serial.print(xballmmFiltered,1);
    Serial.print(',');
    Serial.print(xballmmAutoIn,1);
    Serial.print(','); 
    Serial.println(xBallmm,1);
//  Serial.print(',');
//  Serial.print(error,1);
//  Serial.print(',');
//  Serial.println(ServoDeg,1);

//
// force constant frame rate
//
  while (micros() < microsLast + deltaT*1000000);  // wait for deltaT since last time through
  microsLast = micros(); // save value for next time through
}

//
// modified from online library
// assumes only 1 block is reported
// that block must be the Ping Pong Ball
//
word PingPongCheck()
{
  int xpos=-1;
  uint16_t nblocks;

  nblocks = PingPongPixy.getBlocks();  
  if (nblocks)
  {
    xpos = PingPongPixy.blocks[0].x;
  }
  
  return xpos;
}


