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

// FSM variables
int currentState;
int nextState;
int nextStateTime;
int numIterations;
int currentTime;

// FSM parameters
int standbySignalWidth = 1200; // Standard standby signal width
int timeStep = 100; // time step in ms
int maxNumIterations = 10;
int standardForwardSpeed = -1200; //Forward speed
int standardRotationSpeed = 200; //Rotation speed
int standardBackwardSpeed =  1200;//Backward speed
int moveBackwardTimer = 500; //Move backward timer
int trunTimer = 1200; // Turn timer about 180 degree
int rotDir;// 1:Rigth  -1:Left
boolean aboutToCollide; // To check if BoeBot is in the goal area


float photoTransistorThreshold = 0.1; // Sensor measuring parameters

/*
   Turn over after the Robot is inside the back area
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

//////////////////////////////ROBOT INITIALIZATION///////////////////////////////////
/*
   Robot initialization
*/
void setup() { // Built in initialization block
  numIterations = 0;
  currentTime = 0;

  currentState = 1; // initial state, search
  nextState = 0;
  nextStateTime = -1;

  setLeftWheelSpeed(standardForwardSpeed);
  setRightWheelSpeed(standardForwardSpeed);

  Serial.begin(serialInputNumber); // Make console listen to serial input
  servoLeft.attach(leftWheelPin);
  servoRight.attach(rightWheelPin);
}


/////////////////////////////////MAIN LOOP/////////////////////////////
/*
   Robot main loop
*/
void loop() {

  currentTime = (float(timeStep) * 0.001) * numIterations;

  if (numIterations > maxNumIterations) {
    stopRobot();
  }

  /*
     Should do obstacle avoidance here
  */

  // state change timer
  if ((nextStateTime >= currentTime) && (nextStateTime <= currentTime + timeStep)) {
    currentState = nextState;
    nextStateTime = -1; // reset timer
  }

  // -------------- //
  // FSM
  // -------------- //
  if (currentState == 0) {

    // Robot should here do general sensor checks, and when appropriate
    // decide on a behaviour.

    // Check if at edge
    boolean atEdge = checkIfAtEdge();
    if (atEdge) {
      initiateTurnTowardsGoalZoneState();
      currentState = 1;
    }

  } else if (currentState == 1) {
    if (checkPhototransistors()) {
      // both sensors gives a reading, go forwards
      setLeftWheelSpeed(standardForwardSpeed);
      setRightWheelSpeed(standardForwardSpeed);
      currentState = 2;

      nextState = 4;
      nextStateTime = currentTime + 1500;
    }
  } else if (currentState == 2) {
    // Entering goal zone, timer for state 3 should be set here
    currentState = 3;
  } else if (currentState == 3) {
    // Backing out of goal zone
    currentState = 4;
  } else if (currentState == 4) { //Backing out of goal zone
    turnAwayFromGoalArea();
    //currentState =0;
  }

  delay(timeStep);
  numIterations++;

  printTransistorReadings(); // DEBUG
}

/*
   Function for initiating state 1
*/
void initiateTurnTowardsGoalZoneState() {
  // determine which way to turn
  int rotDir;
  int leftWheelRotSpeed;
  int rightWheelRotSpeed;
  if (sensorBelowThreshold(rightPhototransistor)) { // do right turn
    leftWheelRotSpeed = standardForwardSpeed;
    rightWheelRotSpeed = 0;
  } else if (sensorBelowThreshold(leftPhototransistor)) { // do left turn
    leftWheelRotSpeed = 0;
    rightWheelRotSpeed = standardForwardSpeed;
  }

  setLeftWheelSpeed(leftWheelRotSpeed);
  setRightWheelSpeed(rightWheelRotSpeed);
}

/*
   Function for setting left wheel signal
   Input: v - addition in micro seconds to add to standby signal width
*/
int setLeftWheelSpeed(int v) {
  int signalWidth = standbySignalWidth + v;
  servoLeft.writeMicroseconds(signalWidth);
}

/*
   Function for setting right wheel signal
   Input: v - addition in micro seconds to add to standby signal width
*/
int setRightWheelSpeed(int v) {
  int signalWidth = standbySignalWidth - v;
  servoRight.writeMicroseconds(signalWidth);
}


////////////////////////////////////////////////FUNCTIONS/////////////////////////////////////////



/*
   Function for checking if sensor reading is small enough
*/
boolean sensorBelowThreshold(int Ax) {
  return (getVoltsByPin(Ax) < photoTransistorThreshold);
}

/*
   Function for checking phototransistor values
*/
boolean checkPhototransistors() {
  return (sensorBelowThreshold(rightPhototransistor) && sensorBelowThreshold(leftPhototransistor));
}

/*
   Function for checking if robot is at an edge, i.e. one sensor gives large enough reading
*/
boolean checkIfAtEdge() {
  return (sensorBelowThreshold(rightPhototransistor) || sensorBelowThreshold(leftPhototransistor));
}


/*
   Function for returning sensor value in volts
*/
float getVoltsByPin(int adPin) {
  float reading = float(analogRead(adPin));
  reading = reading * 5.0 / 1024.0;
  return reading;
}


/*
  Function for stopping the servo motors
*/
void stopRobot() {
  servoLeft.detach();
  servoRight.detach();
}




/*
   Turn over after the Robot is inside the back area
*/
void turnAwayFromGoalArea() {

  setLeftWheelSpeed(standardBackwardSpeed);
  setRightWheelSpeed(standardBackwardSpeed);

  //while(aboutToCollide){
  //delay(1000);
  //aboutToCollide = checkPhototransistors();
  //}

  // Until the reading goes high
  delay(moveBackwardTimer);
  setLeftWheelSpeed(standardRotationSpeed); // Turn to right
  setRightWheelSpeed(-standardRotationSpeed); // Turn to right
  delay(trunTimer);
}










///////////////////////////////////////////////DEBUGGERS//////////////////////////////
/*
   Debugging purpose - printing phototransistor sensor values
*/
void printTransistorReadings() {
  Serial.println("\n Right:");
  Serial.print(getVoltsByPin(rightPhototransistor)); // prints voltage reading of phototransistor
  Serial.println("  \n Left:");
  Serial.print(getVoltsByPin(leftPhototransistor));
 Serial.println("----------------------");
 // Serial.println(" Volts");
 // Serial.println("");
}
