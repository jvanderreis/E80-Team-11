/********
Modified E80 Code for Team 11
Current Author:
    Jack Van der Reis (jvanderreis@g.hmc.edu) '28 (contributed in 2026)
Previous Contributors:
    Wilson Ives (wives@g.hmc.edu) '20 (contributed in 2018)
    Christopher McElroy (cmcelroy@g.hmc.edu) '19 (contributed in 2017)  
    Josephine Wong (jowong@hmc.edu) '18 (contributed in 2016)
    Apoorva Sharma (asharma@hmc.edu) '17 (contributed in 2016)                    
*/

/* Libraries */

// general
#include <Arduino.h>
#include <Wire.h>
#include <Pinouts.h>

// E80-specific
#include <SensorIMU.h>
#include <MotorDriver.h>
#include <Logger.h>
#include <Printer.h>


/* Global Variables */

// period in ms of logger and printer
#define LOOP_PERIOD 100

// Motors
MotorDriver motorDriver;

// IMU
SensorIMU imu;

// Logger
Logger logger;
bool keepLogging = true;

// Printer
Printer printer;

// loop start recorder
int loopStartTime;

void setup() {
  printer.init();

  /* Initialize the Logger */
  logger.include(&imu);
  logger.include(&motorDriver);
  logger.init();

  /* Initialise the sensors */
  imu.init();

  /* Initialize motor pins */
  motorDriver.init();

  /* Keep track of time */
  printer.printMessage("Starting main loop",10);
  loopStartTime = millis();
}


void loop() {

  int currentTime = millis() - loopStartTime;
  
  ///////////  Don't change code above here! ////////////////////
  // write code here to make the robot fire its motors in the sequence specified in the lab manual 
  // the currentTime variable contains the number of ms since the robot was turned on 
  // The motorDriver.drive function takes in 3 inputs arguments motorA_power, motorB_power, motorC_power: 
  //       void motorDriver.drive(int motorA_power,int motorB_power,int motorC_power); 
  // the value of motorX_power can range from -255 to 255, and sets the PWM applied to the motor 
  // The following example will turn on motor B for four seconds between seconds 4 and 8 
  
  // For lab time, motor A will be the left motor, motor B will be the right motor, motor C will be the vertical motor
  if (currentTime < 4000) {
    // S0: Wait for 4 seconds, if robot is neutrally bouyant it should just hover around water surface
    motorDriver.drive(0, 0, 0); 
  }

  else if (currentTime < (7000)) {
    // S1: Dive for 3 seconds
    // Note: if in testing robot goes up, flip sign for motor C
    motorDriver.drive(0, 0, 150); 
  }

  else if (currentTime < 13000) {
    // S2: Forward for 6 seconds
    // Activate port and starboard motors (left and right)
    // If robot yaws left, turn down right motor power
    // If robot yaws right, turn down left motor power
    motorDriver.drive(200, 200, 0); 
  } 

  else if (currentTime < 17000) {
    // S3: Move robot to surface for 4 seconds, will just power past surface if too fast
    // Activate motor C, if state 1 needs flipping, flip sign here as well
    motorDriver.drive(0, 0, -150); 
  } 

  else {
    // S4 Halt
    motorDriver.drive(0, 0, 0); 
  }

  // DONT CHANGE CODE BELOW THIS LINE 
  // --------------------------------------------------------------------------

  
  if ( currentTime-printer.lastExecutionTime > LOOP_PERIOD ) {
    printer.lastExecutionTime = currentTime;
    printer.printValue(0,imu.printAccels());
    printer.printValue(1,imu.printRollPitchHeading());
    printer.printValue(2,motorDriver.printState());
    printer.printToSerial();  // To stop printing, just comment this line out
  }

  if ( currentTime-imu.lastExecutionTime > LOOP_PERIOD ) {
    imu.lastExecutionTime = currentTime;
    imu.read(); // this is a sequence of blocking I2C read calls
  }

  if ( currentTime-logger.lastExecutionTime > LOOP_PERIOD && logger.keepLogging) {
    logger.lastExecutionTime = currentTime;
    logger.log();
  }

}
