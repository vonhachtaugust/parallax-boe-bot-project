

///////////////////////////////////Hardware Map//////////////////////////////
/*         Digital port: D1..D13
           Analog port: A0..A5
   /////////////////////////////////////////////////////////////////////////
   D0:
   D1:
   D2:rightPhototransistor
   D3:leftPhototransistor
   D4:
   D5:
   D6:
   D7:
   D8:
   D9:pingTrigerPin
   D10:pingEchoPin
   D11:sonarServoPin
   D12:leftWheelPin
   D13:rightWheelPin

   A0:
   A1:
   A2:
   A3:
   A4:
   A5:

*/



///////////////////////////////////Function Map//////////////////////////////
/*         Short description of the funtions. Plus their input and output

   /////////////////////////////////////////////////////////////////////////




  void initiateTurnTowardsGoalZoneState() :Function for initiating state 1

  boolean sensorBelowThreshold(int Ax) :Function for checking if sensor reading is small enough

  boolean sensorAboveThreshold(int Ax) :Function for checking if sensor reading is small enough

  boolean checkPhototransistors() :Function for checking phototransistor values

  boolean checkIfAtEdge() : Function for checking if robot is at an edge, i.e. one sensor gives large enough reading

  float getVoltsByPin(int adPin) : Function for returning sensor value in volts

  void turnAwayFromGoalArea() :   Turn over after the Robot is inside the goal area

  void sonarServoTurn( int sonarServoMinAngle, int sonarServoMaxAngle): Sonar servo turn

  void turnAwayFromGoalArea() :Turn over after the Robot is inside the back area

  int SonarReadingDistance() :Sonar reading function and its calculation

  long  rcTime(int pin) :  Measures and displays microsecond decay time for light sensor.

  float differenceLeftRigthPhotoSensor(int leftPhotoSensorPin, int rightPhotoSensorPin) :  Function to calculate the normalzed of right and left photosensors ----> return Vlaue:[-0.5 0.5]=[left right]

  Void findIn(int value): Finds the first instance of a value in an array.

  int FindOpenRoam(int indexTemp) :Find an opening in system's 180-degree field of distance detection. Returns:    Distance measurement dictated by the scalar

  void WallCornerAvoidance() :Avoid Wall and Corner function

  void ReadingSonarAllAngles() :Reading Sonar in all angles of sequenceOfScan array

  turnSonarServoToCertainDegree(int degree,int timer):   Position the horn of Sonar servo for a cetain degree

  int cmDistance():Get cm distance measurment from Ping Ultrasonic Distance Sensor and returns: distance measurement in cm.
  /*
*/



//************************************************************************************************************************************************************
//************************************************************************************************************************************************************


#include <Servo.h> // Include servo library

Servo servoRight; // Declare right servo
Servo servoLeft; // Declare left servo
Servo servoClaw; //Declares claw servo
Servo sonarServo;  // create servo object to control a servo


// Robot hardware parameters
const int rightPhototransistor = 2;
const int leftPhototransistor = 3;
const int pingTrigerPin = 4; // Sonar triger pin
const int pingEchoPin = 5; //Sonar Echo Pin
const int sonarServoPin = 11; // Sonar Servo
const int leftWheelPin = 12; // Left whell servo
const int rightWheelPin = 13; //Right wheel Servp
const double sonarSensorOffsetX = 0.0; // Sonar sensor horizontal offset in cm:s
const double sonarSensorOffsetY = 8.0; // Sonar sensor vertical offset in cm:s

//const int clawPin; // TO BE SET

const int serialInputNumber = 9600; //bps channel for serial input


// FSM variables
int currentState;
int nextState;
int nextStateTime;
int numIterations;
int currentTime;
boolean aboutToCollide; // To check if BoeBot is in the goal area
boolean clawGrippedObject = false; // To check if claw has gripped an object, initially assumed to be false.
int rotDir;// 1:Rigth  -1:Left


// FSM parameters
int standbySignalWidth = 1500; // Standard standby signal width
int clawOpenPWMwidth; // To be set depending on parameters for claw-servo.
int clawGripPWMwidth; // To be set depending on parameters for claw-servo.
int standardForwardSpeed = 200; //Forward speed
int standardRotationSpeed = 200; //Rotation speed
int standardBackwardSpeed =  -200;//Backward speed
int moveBackwardTimer = 1000; //Move backward timer
int turnTimer = 1080; // Turn timer about 180 degree /180*6 miliseconds
float photoTransistorThreshold = 0.25; // Sensor measuring parameters
float sensorErrorMarginCorner = 0.05;
int sonarServoMinAngle = 0;
int sonarServoMaxAngle = 90;
const int msPerTurnDegree = 6;                     // delay for maneuvers each cycle
const int tooCloseCm = 30;                         // For distance decisions
const int bumpedCm = 6;                            //distance stop condition

// Simulation parameters
int timeStep = 100; // time step in ms
int maxRunTime = 25; // max run time in seconds
int sequenceOfScan[] = {0 , 2 , 4 , 6 , 8 , 10 , 9 , 7 , 5 , 3 , 1 }; // Sequence of turn for servo positions to get Sonar samples
const int elementScanSequence = sizeof(sequenceOfScan) / sizeof(int); //Number of sequences
int cm[sizeof(sequenceOfScan)]; // Array to store the distances (cm) of Sonar reading
int angleValueofsequenceOfScan[sizeof(sequenceOfScan)]; //angle degree that Sonar servo will get samples from
int minIndexOfSequenceCollisionAvoidance = 3; //Minimum angle of the sonar reading sequence "sequenceOfScan" that might cause robot run into something
int maxIndexOfSequenceCollisionAvoidance = 7; //Maximum angle of the sonar reading sequence "sequenceOfScan" that might cause robot run into something
int servoSonatTheta ;  // Set up initial sonar servo turn routate angle
const int sonarOpeningAngle = 180; // Opening angle for the sonar
const int sonarForwardAngle = 90; // The angle for which the sonar is pointing forwards
const int degreesStepTurnSonarServo = sonarOpeningAngle / (elementScanSequence - 1); // degree differences in the Sonar servo sequences



// Simulation variables
int maxNumIterations;





//********************************************************** INITIALIZATION *************************************************************************************

/*
   Robot initialization
*/
void setup() { // Built in initialization block
  numIterations = 0;
  currentTime = 0;

  maxNumIterations = float(maxRunTime) / (timeStep * 0.001);

  currentState = 1; // initial state, search
  nextState = 0;
  nextStateTime = -1;

  maneuver(standardForwardSpeed, standardForwardSpeed);

  Serial.begin(serialInputNumber); // Make console listen to serial input
  servoLeft.attach(leftWheelPin);
  servoRight.attach(rightWheelPin);

  //servoClaw.attach(clawPin); //Un-comment once claw servo has been dedicated to a pin and clawPin thereby has been set.
  //servoClaw.setMicroseconds(clawOpenPWMwidth); //Un-comment once servoClaw has been properly attached.

  sonarServo.attach(sonarServoPin);  // attaches the servo on pin 11 for sonar
  ConvertServoSequenceToDegree();
  servoSonatTheta = angleValueofsequenceOfScan[1];  // Set up initial turn routate angle
}


//************************************************************* MAIN LOOP *******************************************************************************
/*
   Robot main loop
*/
void loop() {
  currentTime = (float(timeStep) * 0.001) * numIterations;
  //Serial.println(angleValueofsequenceOfScan[0]);
  //Serial.println(angleValueofsequenceOfScan[5]);
  //Serial.println(angleValueofsequenceOfScan[6]);
  if (numIterations > maxNumIterations) {
    maneuver(0, 0, -1); //Permanently stop Robot
  }
  /*
     Should do obstacle detection here
  */
  // state change timer
  if ((nextStateTime >= currentTime) && (nextStateTime <= currentTime + timeStep)) {
    currentState = nextState;
    nextStateTime = -1; // reset timer
  }



  //***************************************************** FSM ************************************************************************

  /*
    Brain of Robot Carlo
  */
  /*
      state 0 : Moving forward and checking the sensors to avoid colision and also to find obstacles if anything detected -------> go to state 1
      state 1: Robot saw an abstacle/Wall thus stops and make more investgation -->if wall/Corner : go to state 2 -->if Obstacle: go to state 3 -->if Goal area and no obsticle grabed already: go to state 9
      state 2: turns away from wall/corner -----> go to state 0
      state 3: get very close to obstacle in right angle ----->if not in goal area: go to state 4 ---> if in the goal area:go to step 9
      state 4: the Robot is touched the obstacle so it grabs it   -----> go to state 5
      state 5: Robots makes investigation to find the area goal through IRF beam scaning  -----> go to state 6
      State 6: Robot moves toward the goal until Robot finds the goal area -----> go to state 7
      state 7: Robot find the right angle to enter the goal area and walk into it -----> go to state 8
      state 8: Robot release the Obstacle --------> go to state 9
      stage 9: Robot turns away from goal area ---------> go to state 0 (start all over again)
  */


  if (currentState == 0) { //Default status

    // Robot should here do general sensor checks, and when appropriate
    // decide on a behaviour. Otherwise move forward
    maneuver(standardForwardSpeed, standardForwardSpeed); //Move forward



  } else if (currentState == 1) {
    WallCornerAvoidance();

    //if (wall/conrner){  // to be fixed
    // currentState = 2;  //To be uncomment
    //}else if(obstacle){ //To be fixed
    //   currentState = 3; //uncomment
    //}else if(goal area){ //To be fixed
    // currentState = 9;   //Uncomment
    //}




  } else if (currentState == 2) {
    // Turn away from wall
    currentState = 0;


  } else if (currentState == 3) {
    //Get closer to obstacle

    // if (in the geal area){
    //    currentState = 9;
    //    }else
    //    {currentState = 4;
    //    }




  } else if (currentState == 4) {
    //Grab the obstacle
    if (!clawGrippedObject) {
      //clawServo.writeMicroseconds(clawGripPWMwidth);
      clawGrippedObject = true;
    }
    currentState = 5;




  } else if (currentState == 5) {
    //find the IRF beacon
    currentState = 6;




  } else if (currentState == 6) {
    //move toward the goal area untill reaches there
    currentState = 7;



  } else if (currentState == 7) {
    //Enter with the right angle into the goal area
    boolean atEdge = checkIfAtEdge();
    if (atEdge) {
      initiateTurnTowardsGoalZoneState();
    }
    currentState = 8;




  } else if (currentState == 8) {
    //release the obstacle
    if (clawGrippedObject) {
      //clawServo.writeMicroseconds(clawOpenPWMwidth);
      clawGrippedObject = false;
    }
    currentState = 9;




  } else if (currentState == 9) {
    //Turn away from the goal area
    turnAwayFromGoalArea();
    currentState = 0;
  }



  delay(timeStep);
  numIterations++;

  // printTransistorReadings(); // DEBUG
}








/* Previous version
  if (currentState == 0) { //Default status

    // Robot should here do general sensor checks, and when appropriate
    // decide on a behaviour. Otherwise move forward

    setLeftWheelSpeed(standardForwardSpeed);
    setRightWheelSpeed(standardForwardSpeed);

    // Check if at edge
    boolean atEdge = checkIfAtEdge();
    if (atEdge) {
      initiateTurnTowardsGoalZoneState();
      currentState = 1;                    /////??????? would you please check Karl why this is here ?
    } else if (checkPhototransistors()) {
      currentState = 2;
    }

  } else if (currentState == 1) {
    if (checkPhototransistors()) {
      // both sensors gives a reading, go forwards
      setLeftWheelSpeed(standardForwardSpeed);
      setRightWheelSpeed(standardForwardSpeed);
      currentState = 2;
    }


  } else if (currentState == 2) {
    // Entering goal zone, timer for state 3 should be set here
    currentState = 3;


  } else if (currentState == 3) {
    // releasing the obstacle in the goal area
    currentState = 4;


  } else if (currentState == 4) {
    //Backing out of goal zone
    turnAwayFromGoalArea();
    currentState =0; // return to the default behavior


  } else if (currentState == 5) { //Claw control state, here robot should either grip an object (If it's not currently gripping one) or release an object (If it's currently gripping one)
    if (!clawGrippedObject) {
      //clawServo.writeMicroseconds(clawGripPWMwidth);
      clawGrippedObject = true;
      //currentState = x; //finalTurnAngleter gripping, set current state to the state that finds the goal. ~( REMEMBER TO CHANGE x )~
      } else{
      //clawServo.writeMicroseconds(clawOpenPWMwidth);
      clawGrippedObject = false;
      //currentState = y; //finalTurnAngleter releasing, back away from the object to not drag it away from the goal. ~( REMEMBER TO CHANGE y )~
    }


    }


  delay(timeStep);
  numIterations++;

  printTransistorReadings(); // DEBUG
  }

*/





//**********************************************************************************************************************************
//************************************************************* FUNCTIONS **********************************************************


/////////////////////// FSM functions ///////////////////////////////////////////////////////////////////
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

  maneuver(leftWheelRotSpeed, rightWheelRotSpeed);
}


/*
   Function for checking if robot is at an edge, i.e. one sensor gives large enough reading
*/
boolean checkIfAtEdge() {
  boolean leftSensorLow = sensorBelowThreshold(leftPhototransistor);
  boolean rightSensorLow = sensorBelowThreshold(rightPhototransistor);
  boolean leftSensorHigh = sensorAboveThreshold(leftPhototransistor);
  boolean rightSensorHigh = sensorAboveThreshold(rightPhototransistor);
  return (leftSensorLow && rightSensorHigh) || (leftSensorHigh && rightSensorLow);
}

/*
   Turn over after the Robot is inside the goal area
*/
void turnAwayFromGoalArea() {

  maneuver(standardBackwardSpeed, standardBackwardSpeed);
  //while(aboutToCollide){
  //delay(1000);
  //aboutToCollide = checkPhototransistors();
  //}
  // Until the reading goes high
  delay(moveBackwardTimer);
  maneuver(standardRotationSpeed, -standardRotationSpeed);
  delay(turnTimer);
}



/*
   Control BOE Wheel servo direction, speed, Without time mentioned.
   Parameters: speedLeft - left servo speed
               speedRight - right servo speed
               Backward  Linear  Stop  Linear   Forward
               -200      -100......0......100       200
*/
void maneuver(int speedLeft, int speedRight)
{
  // Call maneuver with just 1 ms blocking; servos will keep going indefinitely.
  maneuver(speedLeft, speedRight, 1);
}


/*
   Control BOE Wheel servo direction, speed and maneuver duration.
   Parameters: speedLeft - left servo speed
               speedRight - right servo speed
               Backward  Linear  Stop  Linear   Forward
               -200      -100......0......100       200
               msTime - time to block code execution before another maneuver
*/
void maneuver(int speedLeft, int speedRight, int msTime)
{
  servoLeft.writeMicroseconds(1500 + speedLeft);   // Set Left servo speed
  servoRight.writeMicroseconds(1500 - speedRight); // Set right servo speed
  if (msTime == -1)                                // if msTime = -1
  {
    servoLeft.detach();                            // Stop servo signals
    servoRight.detach();
  }
  delay(msTime);                                   // Delay for msTime
}



////////////////////////////////// Photosenosors functions ////////////////////////////////////////////////////////////////////
/*
   Function for checking if photosensor reading is small enough
*/
boolean sensorBelowThreshold(int Ax) {
  return (getVoltsByPin(Ax) < photoTransistorThreshold - sensorErrorMarginCorner);
}
boolean sensorAboveThreshold(int Ax) {
  return (getVoltsByPin(Ax) > photoTransistorThreshold + sensorErrorMarginCorner);
}


/*
   Function for checking phototransistor values
*/
boolean checkPhototransistors() {
  return (sensorBelowThreshold(rightPhototransistor) && sensorBelowThreshold(leftPhototransistor));
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
   Measures and displays microsecond decay time for photosensor.
*/
long  rcTime(int pin)                         // ..returns decay time
{
  pinMode(pin, OUTPUT);                      // Charge capacitor
  digitalWrite(pin, HIGH);                   // ..by setting pin ouput-high
  delay(1);                                  // ..for 5 ms
  pinMode(pin, INPUT);                       // Set pin to input
  digitalWrite(pin, LOW);                    // ..with no pullup
  long time  = micros();                     // Mark the time
  while (digitalRead(pin));                  // Wait for voltage < threshold
  time = micros() - time;                    // Calculate decay time
  return time;                               // Return decay time
}


/*
   Function to calculate the normalzed of right and left photosensors ----> return Vlaue:[-0.5 0.5]=[left right]
*/
float differenceLeftRigthPhotoSensor(int leftPhotoSensorPin, int rightPhotoSensorPin) {
  float tLeft = float(rcTime(leftPhotoSensorPin)); // Get left light & make float
  float tRight = float(rcTime(rightPhotoSensorPin));  // Get right light & make float
  float ndShade; // Normalized differential shade
  ndShade = tRight / (tLeft + tRight) - 0.5; // Calculate it and subtract 0.5
  return ndShade;
}









/////////////////////////////////////////////// Sonar functions ////////////////////////////////////////////////////////////////////////////


/*
   Sonar reading function and its calculation
*/
int SonarReadingDistance() {
  //Ping
  pinMode(pingTrigerPin, OUTPUT);
  digitalWrite(pingTrigerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingTrigerPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingTrigerPin, LOW);
  //read
  pinMode(pingEchoPin, INPUT);
  int  duration = pulseIn(pingEchoPin, HIGH);

  // convert the time into a distance

  return duration ;
}


/*
   Get cm distance measurment from Ping Ultrasonic Distance Sensor
   Returns: distance measurement in cm.
   3 micro seconds delay to cancel noise
*/
int cmDistance()
{
  int usTocm = 29;                                 // Ping conversion constant
  int distance = 0;                                // Initialize distance to zero
  do                                               // Loop in case of zero measurement
  {
    int us = SonarReadingDistance();                        // Get Ping microsecond measurement
    distance = us / usTocm / 2; // Convert to cm measurement
    delay(3);                                      // Pause before retry (if needed)
  }
  while (distance == 0);                           //To cancel the error of Sonar reading
  return distance;                                 // Return distance measurement
}


/*
   Convert the Sonar servo sequences to degree
*/
void ConvertServoSequenceToDegree() {

  for (int i = 0; i < elementScanSequence; i++) {
    angleValueofsequenceOfScan[i] = sequenceOfScan[i] * degreesStepTurnSonarServo;
  }
}



/*
   Position the horn of Sonar servo for a cetain degree
*/
void turnSonarServoToCertainDegree(int degree, int timer)
{
  sonarServo.write(degree);
  delay(timer);
}



/*
   Sonar servo turn Manual between two angles (Min angle to Maximum) and return
*/
void sonarServoTurn(int sonarServoMinAngle, int sonarServoMaxAngle) {
  int pos;
  for (pos = sonarServoMinAngle; pos <= sonarServoMaxAngle; pos += 1) { // goes from min degrees to max degrees
    // in steps of 1 degree
    sonarServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = sonarServoMaxAngle; pos >= sonarServoMinAngle; pos -= 1) { // goes from 180 degrees to 0 degrees
    sonarServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}




/*
   Reading Sonar in all angles of sequenceOfScan array
*/
void ReadingSonarAllAngles() {

  for (int i = 0; i < elementScanSequence; i++) {
    turnSonarServoToCertainDegree(angleValueofsequenceOfScan[i], 120); // full rotation of servo is 250 miliseconds
    cm[i] = cmDistance();                            // Measure cm from this turn angle
    Serial.println(cm[i]);
  }
}


/*
   Avoid Wall and Corner function
*/
void WallCornerAvoidance() {
  ReadingSonarAllAngles();
  for (int i = minIndexOfSequenceCollisionAvoidance; i <= maxIndexOfSequenceCollisionAvoidance; i++) {
    if (cm[i] < tooCloseCm) {
      maneuver(0, 0);
      int turnAngle = FindOpenRoam(i);                     // Get opening (in terms of sequence element
      int turnAngleTime = turnAngle * msPerTurnDegree;

      if (turnAngleTime < 90)                         // If negative turning angle,
      {
        maneuver(-200, 200, turnAngleTime);         // then rotate CCW for turningAngleTime ms
      }
      else                                           // If positive turning angle,
      {
        maneuver(200, -200, turnAngleTime);          // then rotate CW for turningAngleTime ms
      }
      maneuver(200, 200);                            // Start going forward again
    }


  }
}



/*
   Find an opening in system's 180-degree field of distance detection.
   Returns:    Distance measurement dictated by the scalar
*/
int FindOpenRoam(int indexTemp) {
  int initialTurnAngle;                                  // Initial turn angle
  int finalTurnAngle;                                   // Final turn angle
  int currentTurnIndex = sequenceOfScan[indexTemp];                   // Copy sequence[i] to local variable
  int kTemp = currentTurnIndex;                                      // Second copy of current sequence[i]
  int inc;                                         // Increment/decrement variable
  int dt;                                          // Time increment
  int reconnaissanceRepeat = 0;                    // Repetitions count
  int minDistance;                                 // Minimum distance measurement
  int i;

  if (currentTurnIndex < (elementScanSequence / 2)) // Increment or decrement depending on where Sonar servo is pointing
  {
    inc = 1;
  }
  else
  {
    inc = -1;
  }


  do { // Rotate Sonar servo until an opening becomes visible.  If it reaches servo limit, turn back to first angle of detection and try scanning toward other servo limit. If still no opening, rotate robot 90 degrees and try again.
    reconnaissanceRepeat++;                                        // Increment repetition count

    if (reconnaissanceRepeat > (elementScanSequence) * 2)       // If no opening after two scans Back up, turn, and stop to try again
    {
      maneuver(-200, -200, 100);
      maneuver(-200, 200, 90 * 6);
      maneuver(0, 0, 1);
    }

    currentTurnIndex += inc;                                        // Increment/decrement k
    if (currentTurnIndex == -1)                                     // Change inc/dec value if limit reached
    {
      currentTurnIndex = kTemp;
      inc = -inc;
      dt = 250;                                      // Whole turn of servo timer
    }
    if (currentTurnIndex == 11)
    {
      currentTurnIndex = kTemp;
      inc = -inc;
      dt = 250;
    }

    i = findIn(currentTurnIndex);  // Look for index of next turn position

    servoSonatTheta = angleValueofsequenceOfScan[i]; // Set robot turn to next position
    turnSonarServoToCertainDegree(servoSonatTheta, dt); //Position to turn
    dt = 100;                                        // Reset for small increment turn movement
    cm[i] = cmDistance();                            // Take Ping measurement for the new robot heading

  } while (cm[i] < tooCloseCm);                                // Keep checking to edge of obstacle

  minDistance = 1000;                                       // Initialize minimum distance to impossibly large value
  for (int t = 0; t < elementScanSequence; t++)
  {
    if (minDistance > cm[t]) minDistance = cm[t];                  // Use minDistance to track smallest distance
  }
  if (minDistance < bumpedCm)                                      // If less than 6 cm, back up a little
  {
    maneuver(-200, -200, 350);                            //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! to check the timer
    currentTurnIndex = -1;                                          // Get Sonar servo ready to start over
  }

  maneuver(0, 0);                                    // Stay still indefinitely


  // Keep rotating Sonar servo until another obstacle that's under 30 cm is detected or the Sonar servo
  // reaches the servo limit.  Keep track of the maximum distance and the angle when this portion
  // of the scan started and stopped.

  initialTurnAngle = sequenceOfScan[i];              // Save initial angle when obstacle disappeared from Sonar
  currentTurnIndex = sequenceOfScan[i];                             // Make a copy of the Sonar servo position again

  int aMax = -2;                                     // Initialize maximum distance measurements to impossibly small values
  int cmMax = -2;

  do {                                              // Loop for scan
    currentTurnIndex += inc;                                       // Inc/dec Sonar servo position
    i = findIn(currentTurnIndex);                                  // Look up index for Sonar servo position
    servoSonatTheta = angleValueofsequenceOfScan[i];         // Position Sonar servo
    turnSonarServoToCertainDegree(servoSonatTheta, 100); //Position to turn

    cm[i] = cmDistance();                            // Measure distance

    if (cm[i] > cmMax)                               // Keep track of max distance and angle(max distance)
    {
      cmMax = cm[i];
      aMax = sequenceOfScan[i];
    }
  } while ((cm[i] > tooCloseCm) && (sequenceOfScan[i] != 0) && (sequenceOfScan[i] != (elementScanSequence - 1)));

  finalTurnAngle = sequenceOfScan[i];                                  // Record final Sonar servo position
  int middleAngle = initialTurnAngle + ((finalTurnAngle - initialTurnAngle) / 2);                      // Calculate middle of opening

  i = findIn((elementScanSequence - 1) / 2);
  servoSonatTheta = angleValueofsequenceOfScan[i];
  turnSonarServoToCertainDegree(servoSonatTheta, 250 / 2); //Bring the Sonar servo to middle

  if (minDistance < bumpedCm + 1)                                    // Turn further if too close
  {
    if (middleAngle < aMax) return aMax; else return middleAngle;
  }
  else
  {
    if (middleAngle < aMax) return middleAngle; else return aMax;
  }

}


/*
   Finds the first instance of a value in an array.
   Parameters: value to search for
   Returns:    index where the matching element was found or -1
*/
int findIn(int value)
{
  int elements = elementScanSequence;
  for (int i = 0; i < elements; i++)
  {
    if (value == sequenceOfScan[i]) return i;
  }
  return -1;
}

/*
  Function for calculating the desired turning angle for the robot to rotate to, assuming sensor is only offset in Y
  Parameters: 1. The sensor reading, i.e. distance from sensor
              2. The sensor reading angle
              3. The sensor vertical offset
  Returns:    Distance from middle of robot wheel axis
*/
double getObjectRobotRelAngle(double sensorReading, double sensorAngle, double sensorOffsetY) {
  return atan( ( sensorReading*sin(sensorAngle) + sensorOffsetY ) / (sensorReading*cos(sensorAngle)) );
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
