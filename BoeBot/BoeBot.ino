/*
 Robotics with the BOE Shield – LeftServoClockwise
 Generate a servo full speed clockwise signal on digital pin 13.
 */

#include <Servo.h> // Include servo library
 
Servo servoLeft; // Declare left servo
Servo servoRight; // Declare right servo

// Robot hardware parameters
int leftWheelPin = 13;
int rightWheelPin = 12;

// FSM parameters
int standbySignalWidth = 1500; // Standard standby signal width
int numIterations;
int maxNumIterations = 50;
int standardForwardSpeed = 500;
int standardRotationSpeed = 200;

/*
 * Function for setting left wheel signal
 * Input: v - addition in micro seconds to add to standby signal width
 */
int setLeftWheelSpeed(int v) {
  int signalWidth = standbySignalWidth + v;
  servoLeft.writeMicroseconds(signalWidth);
}

/*
 * Function for setting right wheel signal
 * Input: v - addition in micro seconds to add to standby signal width
 */
int setRightWheelSpeed(int v) {
  int signalWidth = standbySignalWidth - v;
  servoRight.writeMicroseconds(signalWidth);
}

/*
 * Robot initialization
 */
void setup() // Built in initialization block
{
  numIterations = 0;
  
  servoLeft.attach(leftWheelPin);
  servoRight.attach(rightWheelPin);
  
  setLeftWheelSpeed(500);
  setRightWheelSpeed(500);
}
 
/*
 * Robot main loop
 */
void loop()
{
  if (numIterations >= maxNumIterations) {
    deEscelateRobot();
  }
  delay(100);
  numIterations++;
}

/*
 * Function for handling robot deescelation
 */
void deEscelateRobot() {
  setLeftWheelSpeed(0);
  setRightWheelSpeed(0);
}