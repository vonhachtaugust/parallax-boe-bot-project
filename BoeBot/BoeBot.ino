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
int leftPhototransistor = A1;
int rightPhototransistor = A0;
int serialInputNumber = 9600;

// FSM parameters
int currentState;
int numIterations;
int standbySignalWidth = 1500; // Standard standby signal width
int timeStep = 100; // time step in ms
int maxNumIterations = 100;
int standardForwardSpeed = 500;
int standardRotationSpeed = 200;
int standardBackwardSpeed = - standardForwardSpeed;
int moveBackwardTimer = 200;
int trunTimer = 100;

// Sensor measuring parameters
float photoTransistorThreshold = 0.2;

/*
 * Turn over after the Robot is inside the back area
 */
void turnAwayFromBlackArea() {
  setLeftWheelSpeed(standardBackwardSpeed);
  setRightWheelSpeed(standardBackwardSpeed);
  
  // Until the reading goes high
  delay(moveBackwardTimer);
  setLeftWheelSpeed(standardRotationSpeed); // Turn to right
  setRightWheelSpeed(-standardRotationSpeed); // Turn to right
  delay(trunTimer);
}

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
void setup() { // Built in initialization block
  numIterations = 0;
  
  currentState = 0; // initial state, search
  setLeftWheelSpeed(standardForwardSpeed);
  setRightWheelSpeed(standardForwardSpeed);
  
  Serial.begin(serialInputNumber); // Make console listen to serial input
  servoLeft.attach(leftWheelPin);
  servoRight.attach(rightWheelPin);
}

/*
 * Robot main loop
 */
void loop() {
  
  if (numIterations >= maxNumIterations) {
    stopRobot();
  }

  // Check collision with photo transistors
  boolean aboutToCollide = checkPhototransistors();
  if (aboutToCollide) {
    stopRobot();
  }

  // ----------
  // FSM
  // ----------
  if (currentState == 0) {

    // Robot should here do general sensor checks, and when appropriate
    // decide on a behaviour.
    
    // Check if at edge
    boolean atEdge = checkIfAtEdge();
    if (atEdge) {
      int rotDir;
      if (sensorBelowThreshold(rightPhototransistor)) {
        rotDir = 1; // right rotation
      } else if (sensorBelowThreshold(leftPhototransistor)) {
        rotDir = -1; // left rotation
      }
    }
  } else if (currentState == 1) {
    // At zone corner
  } else if (currentState == 2) {
    // Entering goal zone
  } else if (currentState == 3) {
    // Backing out of goal zone
  }
  
  delay(timeStep);
  numIterations++;

  printTransistorReadings(); // DEBUG
}

/* 
 * Function for checking if sensor reading is small enough
 */
boolean sensorBelowThreshold(int Ax) {
  return (getVoltsByPin(Ax) < photoTransistorThreshold);
}

/* 
 * Function for checking phototransistor values
 */
boolean checkPhototransistors() {
  return (sensorBelowThreshold(rightPhototransistor) && sensorBelowThreshold(leftPhototransistor));
}

/* 
 * Function for checking if robot is at an edge, i.e. one sensor gives large enough reading
 */
boolean checkIfAtEdge() {
  return (sensorBelowThreshold(rightPhototransistor) || sensorBelowThreshold(leftPhototransistor));
}

/*
 * Function for returning sensor value in volts
 */
float getVoltsByPin(int adPin) {
  float reading = float(analogRead(adPin));
  reading = reading * 5.0 / 1024.0;
  return reading;
}

// Function for stopping the servo motors
void stopRobot() {
    servoLeft.detach();
    servoRight.detach();
}

/*
 * Debugging purpose - printing phototransistor sensor values
 */
void printTransistorReadings(){
  Serial.print(getVoltsByPin(rightPhototransistor)); // prints voltage reading of phototransistor
  Serial.println(" Volts");
  Serial.println("");
}
