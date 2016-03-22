/*
  Robotics with the BOE Shield – LeftServoClockwise
  Generate a servo full speed clockwise signal on digital pin 13.
*/

#include <Servo.h> // Include servo library

Servo servoRight; // Declare right servo
Servo servoLeft; // Declare left servo
Servo servoClaw; //Declares claw servo

// Robot hardware parameters
int rightWheelPin = 12;
int leftWheelPin = 13;
int clawPin; // TO BE SET
int rightPhototransistor = A0;
int leftPhototransistor = A1;
int serialInputNumber = 9600; //bps channel for serial input

// FSM variables
int currentState;
int nextState;
int nextStateTime;
int numIterations;
int currentTime;

// FSM parameters
int standbySignalWidth = 1500; // Standard standby signal width
int timeStep = 100; // time step in ms
int maxNumIterations = 10;
int standardForwardSpeed = -1300; //Forward speed
int standardRotationSpeed = 200; //Rotation speed
int standardBackwardSpeed =  1300;//Backward speed
int clawOpenPWMwidth = 1; // To be set depending on parameters for claw-servo.
int clawGripPWMwidth = 1; // To be set depending on parameters for claw-servo.
int moveBackwardTimer = 500; //Move backward timer
int turnTimer = 1200; // Turn timer about 180 degree
int rotDir;// 1:Rigth  -1:Left
boolean aboutToCollide; // To check if BoeBot is in the goal area
boolean clawGrippedObject = false; // To check if claw has gripped an object, initially assumed to be false.

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
  delay(turnTimer);
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

  Serial.begin(serialInputNumber); // Make console listen to serial input
  servoLeft.attach(leftWheelPin);
  servoRight.attach(rightWheelPin);
  setLeftWheelSpeed(standardForwardSpeed);
  setRightWheelSpeed(standardForwardSpeed);

  //servoClaw.attach(clawPin); //Un-comment once claw servo has been dedicated to a pin and clawPin thereby has been set.
  //servoClaw.setMicroseconds(clawOpenPWMwidth); //Un-comment once servoClaw has been properly attached.
}


//////////////////////////////////////////// MAIN LOOP ////////////////////////////////////////////
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

  //////////////////////////////////////////////// FSM ///////////////////////////////////////////

  if (currentState == 0) {

    // Robot should here do general sensor checks, and when appropriate
    // decide on a behaviour. Otherwise move forward

    setLeftWheelSpeed(standardForwardSpeed);
    setRightWheelSpeed(standardForwardSpeed);

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
  } else if (currentState == 5) { //Claw control state, here robot should either grip an object (If it's not currently gripping one) or release an object (If it's currently gripping one)
    if (!clawGrippedObject) {
      clawServo.writeMicroseconds(clawGripPWMwidth);
      clawGrippedObject = true;
      currentState = x; //After gripping, set current state to the state that finds the goal. ~( REMEMBER TO CHANGE x )~
      } else{
      clawServo.writeMicroseconds(clawOpenPWMwidth);
      clawGrippedObject = false;
      currentState = y; //After releasing, back away from the object to not drag it away from the goal. ~( REMEMBER TO CHANGE y )~
    }
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


//////////////////////////////////////////////// FUNCTIONS /////////////////////////////////////////



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
  delay(turnTimer);
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
