

///////////////////////////////////Hardware Map//////////////////////////////
/*         Digital port: D1..D13
           Analog port: A0..A5
   /////////////////////////////////////////////////////////////////////////
   D0:
   D1:
   D2:
   D3:
   D4:firstSonarPingTrigerPin
   D5:firstSonarPingEchoPin
   D6:SecondSonarPingTrigerPin
   D7:SecondSonarPingEchoPinFirst
   D8:IRF right Reciever Pin
   D9:IRF left Reciever Pin
   D10:
   D11:sonarServoPin
   D12:leftWheelPin
   D13:rightWheelPin

   A0:
   A1:
   A2:
   A3:
   A4:leftPhototransistor
   A5:rightPhototransistor

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

  int FirstSonarReadingDistance() :Upper Sonar reading function and its calculation

  int int SecondSonarReadingDistance():Lower Sonar reading

  long  rcTime(int pin) :  Measures and displays microsecond decay time for light sensor.

  float differenceLeftRigthPhotoSensor(int leftPhotoSensorPin, int rightPhotoSensorPin) :  Function to calculate the normalzed of right and left photosensors ----> return Vlaue:[-0.5 0.5]=[left right]

  Void findIn(int value): Finds the first instance of a value in an array.

  int FindOpenRoam(int indexTemp) :Find an opening in system's 180-degree field of distance detection. Returns:    Distance measurement dictated by the scalar

  void WallCornerAvoidance() :Avoid Wall and Corner function

  void ReadingSonarAllAngles() :Reading Sonar in all angles of sequenceOfScan array

  turnSonarServoToCertainDegree(int degree,int timer):   Position the horn of Sonar servo for a cetain degree

  int cmDistance():Get cm distance measurment from Ping Ultrasonic Distance Sensor and returns: distance measurement in cm. Input: 1: Sonar one(up) 2:Sonar two(down)

  double getObjectRobotRelAngle(double sensorReading, double sensorAngle, double sensorOffsetY): Find the right angle to the obstacle

  int GetCloseToObstacle(): Find the closest obstacle and go toward it. Stay with some distance from it. retun -1 if fail

  void ClawGrip(): To grap obstacle

  void ClawRelease(): To reloase obstacle

  boolean irRead(IRFReceiverPinRight) : Read IRF RECIEVER /  0:Detect  1: no detction

  int GetDirectionToSafeZone() : Get direction to safe zone function

*/



//************************************************************************************************************************************************************
//************************************************************************************************************************************************************


#include <Servo.h> // Include servo library

Servo servoRight; // Declare right servo
Servo servoLeft; // Declare left servo
Servo servoClaw; //Declares claw servo
Servo sonarServo;  // create servo object to control a servo


// Robot hardware parameters
const int rightPhototransistor = 5;
const int leftPhototransistor = 4;
const int firstSonarPingTrigerPin = 6; // Sonar triger pin
const int firstSonarPingEchoPin = 7; //Sonar Echo Pin
const int secondSonarPingTrigerPin = 4; // Sonar triger pin
const int secondSonarPingEchoPin = 5; //Sonar Echo Pin
const int IRFReceiverPinRight = 8; //IRF reciever Pin
const int sonarServoPin = 11; // Sonar Servo
const int leftWheelPin = 12; // Left whell servo
const int rightWheelPin = 13; //Right wheel servo
const int clawPin = 10; // claw servo pin for gripper
const double sonarSensorOffsetX = 0.045; // Sonar sensor horizontal offset in m:s
const double sonarSensorOffsetY = 0.115; // Sonar sensor vertical offset in m:s

//const int clawPin; // TO BE SET

const int serialInputNumber = 9600; //bps channel for serial input


// FSM variables
int currentState;
int nextState;
int nextStateTime;
int numIterations;
int currentTime;
boolean aboutToCollide; // To check if BoeBot is in the goal area
boolean clawGrippedObjectFlag = false; // To check if claw has gripped an object, initially assumed to be false.
boolean moveTowardSafeFlag = 0;


// FSM parameters
int maxForwardSpeed = 200; // Max Forward speed
int linearForwardSpeed = 100; // linear speed
int maxBackwardSpeed =  -200;//Max Backward speed
int linearBackwardSpeed = -100; // linear backward speed
int turnTimer = 1080; // Turn timer about 180 degree /180*6 miliseconds
float photoTransistorThreshold = 0.25; // Sensor measuring parameters
float sensorErrorMarginCorner = 0.05;
const int msPerTurnDegree = 6;                     // delay for maneuvers each cycle
const int tooCloseCmSonarReading = 35;                         // For distance decisions of Sonar reading
const int bumpedCmSonarReading = 10;                            //distance stop condition of Sonar reading
const int minDistanceBetweenWallAndObstacle = tooCloseCmSonarReading + 5; //40 cm beween obstacle and a wall
const int maxStopDistanceFromObstacle = 7; //How far Robto stops from an obstacle
//const int maxSonarRange=200; //How far sonar reding matters

// Simulation parameters
int timeStep = 100; // time step in ms
int maxRunTime = 25; // max run time in seconds
int sequenceOfScan[] = {0 , 2 , 4 , 6 , 8 , 10 , 9 , 7 , 5 , 3 , 1 }; // Sequence of turn for servo positions to get Sonar samples
const int elementScanSequence = sizeof(sequenceOfScan) / sizeof(int); //Number of sequences
int distanceDiffSonarsRead ; //Differences between two sonar reading
int angleValueofsequenceOfScan[sizeof(sequenceOfScan)]; //angle degree that Sonar servo will get samples from
int minIndexOfSequenceCollisionAvoidance = 2; //Minimum angle of the sonar reading sequence "sequenceOfScan" that might cause robot run into something
int maxIndexOfSequenceCollisionAvoidance = 8; //Maximum angle of the sonar reading sequence "sequenceOfScan" that might cause robot run into something
int servoSonarTheta ;  // Set up initial sonar servo turn routate angle
const int sonarOpeningAngle = 180; // Opening angle for the sonar
const int sonarForwardAngle = 90; // The angle for which the sonar is pointing forwards
const int degreesStepTurnSonarServo = sonarOpeningAngle / (10); // degree differences in the Sonar servo sequences
const int maxDistanceWallObstacleDetect = 80; //Maximum distance from robto for an object or wall to be detected
int sonarServoSeq;
int  upSonarRead;
int downSonarRead;


// Simulation variables
int maxNumIterations;
int inc = 1;
int incTemp;
boolean tempReachGoalFlag = 0;




//********************************************************** INITIALIZATION *************************************************************************************

/*
   Robot initialization
*/
void setup() { // Built in initialization block
  numIterations = 0;
  currentTime = 0;

  maxNumIterations = float(maxRunTime) / (timeStep * 0.001);

  currentState = 0;        //               initial state, search
  nextState = 0;
  nextStateTime = -1;


  Serial.begin(serialInputNumber); // Make console listen to serial input
  servoLeft.attach(leftWheelPin);
  servoRight.attach(rightWheelPin);
  servoClaw.attach(clawPin);

  sonarServo.attach(sonarServoPin);  // attaches the servo on pin 11 for sonar
  ConvertServoSequenceToDegree();
  sonarServoSeq = minIndexOfSequenceCollisionAvoidance - 1;
  servoClaw.write(20);

  turnSonarServoToCertainDegree(angleValueofsequenceOfScan[sonarServoSeq], 100); // full rotation of servo is 250 miliseconds
}




//************************************************************* MAIN LOOP *******************************************************************************
/*
   Robot main loop
*/
void loop() {

  currentTime = (float(timeStep) * 0.001) * numIterations;

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

  /* Add yr Robot's name here :)
    Brain of Robot Karl's:Carlo /Babak's: MICH-V1 /
  */
  /*
      state 0 : Moving forward and checking the sensors to avoid colision and also to find obstacles if anything detected -------> go to state 1
      state 1: Robot saw an abstacle/Wall thus stops and make more investgation -->if wall/Corner : go to state 2 -->if Obstacle: go to state 3 -->if Goal area and no obsticle grabed already: go to state 9
      state 2: turns away from wall/corner -----> go to state 0
      state 3: get very close to obstacle in right angle ----->if not in goal area: go to state 4 ---> if in the goal area:go to step 9
      state 4: the Robot is touched the obstacle so it grabs it   -----> go to state 5
      state 5: Robots makes investigation to find the area goal through IRF beam scaning  -----> go to state 6
      State 6: Robot moves toward the goal area until finds the goal area -----> go to state 7
      state 7: Robot find the right angle to enter the goal area and walk into it -----> go to state 8
      state 8: Robot release the Obstacle --------> go to state 9
      stage 9: Robot turns away from goal area ---------> go to state 0 (start all over again)
  */

  if (currentState == 0) //Default status                                                    ////////////////////////////////////State 0
  {

    maneuver(100, 100);
    sonarServoSeq += inc;
    incTemp = inc;
    if ((sonarServoSeq < 1) || (sonarServoSeq > elementScanSequence - 2 )) inc = -inc;                     // Change inc/dec value if limit reached
    int tempValue1 = 0;
    int tempValue2 = 0;
    for (int t = 0; t < 8; t++) {
      tempValue1 += cmDistance(1);
      tempValue2 += cmDistance(2);
    }
    upSonarRead = tempValue1 / 8;
    downSonarRead = tempValue2 / 8;

    turnSonarServoToCertainDegree(angleValueofsequenceOfScan[sonarServoSeq], 50);
   
    if (!clawGrippedObjectFlag) {
      distanceDiffSonarsRead = abs(upSonarRead - downSonarRead);
      if ((upSonarRead > 0) && (upSonarRead < 200) && (downSonarRead < 150)) {
        if ((upSonarRead < maxDistanceWallObstacleDetect) || (downSonarRead < maxDistanceWallObstacleDetect)) {
          currentState = 1;
        } else {
          maneuver(maxForwardSpeed, maxForwardSpeed); //Move forward
        }
      }
    } else {
      if (upSonarRead < tooCloseCmSonarReading) {
        currentState = 2;
      } else {
        currentState = 5;
      }
    }
  }

  else if (currentState == 1)                                                ///////////////////////////////////////////State 1
  {

    if (upSonarRead < tooCloseCmSonarReading)
    {
      maneuver(0, 0);
      currentState = 2;
    } else if (distanceDiffSonarsRead >= minDistanceBetweenWallAndObstacle)
    {
      maneuver(0, 0);
      currentState = 3;
    } else
    {
      currentState = 0;
      //currentState = 9; //Disable for now should be 9
    };


  } else if (currentState == 2) // Turn away from wall                                    ///////////////////////////////state 2
  {

    WallCornerAvoidance();
    currentState = 0;


  } else if (currentState == 3) //Get close to obstacle                                    ///////////////////////////////state 3
  {
    int tempDeg = getObjectRobotRelAngle(1, angleValueofsequenceOfScan[sonarServoSeq - incTemp]);
    int tempReturn = GetCloseToObstacle(tempDeg);

    if (tempReturn == 1)
    {
      currentState = 4;
    } else
    {
      currentState = 0;
      turnSonarServoToCertainDegree(angleValueofsequenceOfScan[sonarServoSeq], 100);
    };


  }
  else if (currentState == 4)                                                              ///////////////////////////////state 4
  {

    //Grab the obstacle
    if (!clawGrippedObjectFlag)
    {
      ClawGrip();
      clawGrippedObjectFlag = true;
    }
    // currentState = 8; //for test
    //currentState = 5;
  }
  else if (currentState == 5) //find the IRF beacon                                           ///////////////////////////////state 5
  {
    moveTowardSafeFlag = GetDirectionToSafeZone();
    if (moveTowardSafeFlag == 1) {
      currentState = 6;   //Maybe to impliment different
    } else {
      currentState = 0;
    };
  } else if (currentState == 6) //move toward the goal area untill reaches there              ///////////////////////////////state 6
  {

    maneuver(200, 200);
    //maneuver(200, 200,1000); //Maybe
    currentState = 7;

  } else if (currentState == 7) //Enter with the right angle into the goal area               ///////////////////////////////state 7
  {
    boolean atEdge = checkIfAtEdge();
    if (atEdge) {
      tempReachGoalFlag = initiateTurnTowardsGoalZoneState();
    }
    if (tempReachGoalFlag == 1)
    {

      currentState = 8;
      tempReachGoalFlag = 0;
    }
    else {
      //currentState = 6;         // Maybe ?!
      currentState = 0;
    }


  } else if (currentState == 8)  //release the obstacle                                      ///////////////////////////////state 8
  {

    if (clawGrippedObjectFlag)
    {
      ClawRelease();
      clawGrippedObjectFlag = false;
      currentState = 9;
    } else {
      currentState = 0;
    }


  } else if (currentState == 9)                                                              ///////////////////////////////state 9
  {
    turnAwayFromGoalArea();
    currentState = 0;
  }


  delay(timeStep);
  numIterations++;


  Serial.print("====================================================");// debug
  Serial.print("currentState :"); //debug
  Serial.println(currentState); //debug
  Serial.println("--------------------");
  Serial.print("CM1: "); Serial.println(upSonarRead);
  Serial.print("CM2: "); Serial.println(downSonarRead);
  delay(1);

  Serial.print("distanceDiffSonarsRead :"); //debug
  Serial.println(distanceDiffSonarsRead); //debug
  printTransistorReadings();//debug
  Serial.print("IR read :"); //debug
  Serial.println(irRead(IRFReceiverPinRight)); //debug
  //  Serial.println(irDetect( ));

  //  delay(250);
}


//**********************************************************************************************************************************
//************************************************************* FUNCTIONS **********************************************************


/////////////////////// FSM functions ///////////////////////////////////////////////////////////////////
/*
   Function for initiating state 1
*/
boolean initiateTurnTowardsGoalZoneState() {
  // determine which way to turn
  int rotDir;
  int leftWheelRotSpeed;
  int rightWheelRotSpeed;
  if (sensorBelowThreshold(rightPhototransistor)) { // do right turn
    leftWheelRotSpeed = maxForwardSpeed;
    rightWheelRotSpeed = 0;
  } else if (sensorBelowThreshold(leftPhototransistor)) { // do left turn
    leftWheelRotSpeed = 0;
    rightWheelRotSpeed = maxForwardSpeed;
  }

  maneuver(leftWheelRotSpeed, rightWheelRotSpeed);
  if (!(sensorBelowThreshold(rightPhototransistor)) && (!sensorBelowThreshold(leftPhototransistor))) {
    return 1;
  } else {
    return 0;
  }
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
  maneuver(maxBackwardSpeed, maxBackwardSpeed);
  //while(aboutToCollide){
  //delay(1000);
  //aboutToCollide = checkPhototransistors();
  //}
  // Until the reading goes high
  delay(1000);
  maneuver(maxForwardSpeed, -maxForwardSpeed);
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




/////////////////////////////////////////////////////////// Sonar functions ////////////////////////////////////////////////////////////////////////////
/*
  First Sonar reading function and its calculation
*/
int FirstSonarReadingDistance() {
  //Ping
  pinMode(secondSonarPingTrigerPin, OUTPUT);
  digitalWrite(secondSonarPingTrigerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(secondSonarPingTrigerPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(secondSonarPingTrigerPin, LOW);
  //read
  pinMode(secondSonarPingEchoPin, INPUT);
  int  duration = pulseIn(secondSonarPingEchoPin, HIGH);

  // convert the time into a distance

  return duration ;
}



/*
  Second Sonar reading function and its calculation
*/
int SecondSonarReadingDistance() {
  //Ping
  pinMode(firstSonarPingTrigerPin, OUTPUT);
  digitalWrite(firstSonarPingTrigerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(firstSonarPingTrigerPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(firstSonarPingTrigerPin, LOW);
  //read
  pinMode(firstSonarPingEchoPin, INPUT);
  int  duration = pulseIn(firstSonarPingEchoPin, HIGH);

  // convert the time into a distance

  return duration ;
}


/*
   Get cm distance measurment from Ping Ultrasonic Distance Sensor
   input : 1 : First sonar 2: Second Sonar
   Returns: distance measurement in cm.
   3 micro seconds delay to cancel noise
*/
int cmDistance(int num)
{
  int us;
  int usTocm = 29;                                 // Ping conversion constant
  int distance = 0;                                // Initialize distance to zero
  int iter = 0;
  int maxIter = 2;
  if (num == 1) {
    us = FirstSonarReadingDistance();
  } else {
    us = SecondSonarReadingDistance();
  }
  do                                               // Loop in case of zero measurement
  {
    iter++;
    distance = us / usTocm / 2; // Convert to cm measurement
    delay(3);                                      // Pause before retry (if needed)
  }
  while ((distance == 0) && (iter < maxIter));                         //To cancel the error of Sonar reading and reduce noise
  return abs(distance);                                 // Return distance measurement
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
   Avoid Wall and Corner function
*/
void WallCornerAvoidance() {
  int cm1[sizeof(sequenceOfScan)]; // Array to store the distances (cm) of up Sonar reading
  int cm2[sizeof(sequenceOfScan)]; //Array to store the distances (cm) of down Sonar reading

  int iter = 0;
  do {
    iter++;
    for (int k = 0; k < elementScanSequence ; k++) {
      turnSonarServoToCertainDegree(angleValueofsequenceOfScan[k], 100); // full rotation of servo is 250 miliseconds
      cm1[k] = cmDistance(1);                            // Measure cm from this turn angle
      //      GetCloseToObstacleV2();                           //Debuging!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      if ( (sequenceOfScan[k] >= minIndexOfSequenceCollisionAvoidance) && (sequenceOfScan[k] <= maxIndexOfSequenceCollisionAvoidance) && (cm1[k] < tooCloseCmSonarReading)) {
        maneuver(0, 0);
        int turnAngle = FindOpenRoam(k); // Get opening (in terms of sequence element
        getObjectRobotRelAngle(1, turnAngle);

        int turnAngleTime = turnAngle * msPerTurnDegree;
        if (turnAngle > 90)                         // If negative turning angle,
        {
          maneuver(-200, 200, turnAngleTime);         // then rotate CCW for turningAngleTime ms  //Maybe need modification!
          maneuver(0, 0);
        }
        else                                           // If positive turning angle,
        {
          maneuver(200, -200, turnAngleTime);          // then rotate CW for turningAngleTime ms   //Maybe need modification!
          maneuver(0, 0);
        }
      }
    }
  } while (iter < 2);////// improved

}



/*
   Find an opening in system's 180-degree field of distance detection.
   Returns:    Distance measurement dictated by the scalar
*/
int FindOpenRoam(int k) {                                                                                 ///////////Improvmnt

  int cm1[sizeof(sequenceOfScan)]; // Array to store the distances (cm) of up Sonar reading
  int cm2[sizeof(sequenceOfScan)]; //Array to store the distances (cm) of down Sonar reading
  int iter = 0;
  int initialTurnAngle;                                  // Initial turn angle
  int finalTurnAngle;                                   // Final turn angle
  int currentTurnIndex = sequenceOfScan[k];                   // Copy sequence[i] to local variable
  int kTemp = currentTurnIndex;                                      // Second copy of current sequence[i]
  int seqInc;                                         // Increment/decrement variable
  int dt;                                          // Time increment
  int reconnaissanceRepeat = 0;                    // Repetitions count
  int minDistance;                                 // Minimum distance measurement
  int i;

  if (currentTurnIndex < (elementScanSequence / 2)) // Increment or decrement depending on where Sonar servo is pointing
  {
    seqInc = 1;
  }
  else
  {
    seqInc = -1;
  }
  do { // Rotate Sonar servo until an opening becomes visible.  If it reaches servo limit, turn back to first angle of detection and try scanning toward other servo limit. If still no opening, rotate robot 90 degrees and try again.
    reconnaissanceRepeat++;                                        // Increment repetition count
    iter++;
    if (reconnaissanceRepeat > (elementScanSequence) * 2)       // If no opening after two scans Back up, turn, and stop to try again
    {
      maneuver(maxBackwardSpeed, maxBackwardSpeed, 100);
      maneuver(maxBackwardSpeed, maxForwardSpeed, 90 * 6);                      ///////////Better function!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      maneuver(0, 0, 1);
    }

    currentTurnIndex += seqInc;                                        // Increment/decrement current Index
    if (currentTurnIndex <= -1)                                     // Change inc/dec value if limit reached
    {
      currentTurnIndex = kTemp;
      seqInc = -seqInc;
      dt = 250;                                      // Whole turn of servo timer
    }
    if (currentTurnIndex > elementScanSequence)
    {
      currentTurnIndex = kTemp;
      seqInc = -seqInc;
      dt = 250;
    }

    i = findIn(currentTurnIndex);  // Look for index of next turn position

    servoSonarTheta = angleValueofsequenceOfScan[i]; // Set robot turn to next position
    turnSonarServoToCertainDegree(servoSonarTheta, dt); //Position to turn
    dt = 100;                                        // Reset for small increment turn movement
    cm1[i] = cmDistance(1);                            // Take Ping measurement for the new robot heading

  } while ((cm1[i] < tooCloseCmSonarReading) && (iter <= 3));                            // Keep checking to edge of obstacle

  minDistance = 1000;                                       // Initialize minimum distance to impossibly large value
  for (int t = 0; t < elementScanSequence; t++)
    if (minDistance > cm1[t]) minDistance = cm1[t];                  // Use minDistance to track smallest distance

  if (minDistance < bumpedCmSonarReading)                                      // If less than 6 cm, back up a little
  {
    maneuver(maxBackwardSpeed, maxBackwardSpeed, 300);                            //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! to check the timer
    currentTurnIndex = -1;                                          // Get Sonar servo ready to start over
  }

  maneuver(0, 0);                                    // Stay still indefinitely


  // Keep rotating Sonar servo until another obstacle that's under 30 cm is detected or the Sonar servo
  // reaches the servo limit.  Keep track of the maximum distance and the angle when this portion
  // of the scan started and stopped.

  initialTurnAngle = sequenceOfScan[i];              // Save initial angle when obstacle disappeared from Sonar
  currentTurnIndex = sequenceOfScan[i];                             // Make a copy of the Sonar servo position again

  int angleMax = -2;                                     // Initialize maximum distance measurements to impossibly small values
  int distanceMax = -2;
  iter = 0;

  do {                                              // Loop for scan
    iter++;
    currentTurnIndex += seqInc;                                       // Inc/dec Sonar servo position
    i = findIn(currentTurnIndex);                                  // Look up index for Sonar servo position
    servoSonarTheta = angleValueofsequenceOfScan[i];         // Position Sonar servo
    turnSonarServoToCertainDegree(servoSonarTheta, 100); //Position to turn

    cm1[i] = cmDistance(1);                            // Measure distance

    if (cm1[i] > distanceMax)                               // Keep track of max distance and angle(max distance)
    {
      distanceMax = cm1[i];
      angleMax = sequenceOfScan[i];
    }
  } while ((cm1[i] > tooCloseCmSonarReading) && (sequenceOfScan[i] != 0) && (sequenceOfScan[i] != 10) && (iter <= 2)); // 10 is the biggest number in the sequences


  finalTurnAngle = sequenceOfScan[i];                                  // Record final Sonar servo position
  int middleAngle = initialTurnAngle + ((finalTurnAngle - initialTurnAngle) / 2);                      // Calculate middle of opening
  middleAngle *= degreesStepTurnSonarServo;
  angleMax *= degreesStepTurnSonarServo;
  i = findIn(5);
  servoSonarTheta = angleValueofsequenceOfScan[i];
  turnSonarServoToCertainDegree(servoSonarTheta, 250 / 2); //Bring the Sonar servo to middle

  if (minDistance < bumpedCmSonarReading + 1)                                    // Turn further if too close
  {
    if (middleAngle < angleMax) return angleMax; else return middleAngle;
  }
  else
  {
    if (middleAngle < angleMax) return middleAngle; else return angleMax;
  }

}//function


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
double getObjectRobotRelAngle(double sensorReading, double sensorAngle) {
  double  val = 180.0 / PI;
  float X = sonarSensorOffsetX;
  float Y = sonarSensorOffsetY;
  int tempDegree;
  if (sensorAngle > 90) {
    tempDegree = sensorAngle - 90;
  } else {
    tempDegree = sensorAngle;
  }
  X -= (tempDegree / (90 / X));
  Y = (tempDegree / (90 / Y));
  if (sensorAngle == 90) return 90;
  val = atan( ( sensorReading * sin(sensorAngle * PI / 180.0) + Y ) / (sensorReading * cos(sensorAngle * PI / 180.0)) + X ) * val;

  if (val < 0) {
    val = 180 + val;
  };
  Serial.print("val:");
  Serial.println(val);
  //delay(10000);
  return val;
}


/*
   Get close to object function
*/
int GetCloseToObstacle(int degree) {      //////////////////////////////////Improvement also flags

  if (degree < 90) {
    maneuver(200, -200, (90 - degree)*msPerTurnDegree);
    maneuver(0, 0);
  } else if (degree > 90) {
    maneuver(-200, 200, (degree - 90)*msPerTurnDegree);
    maneuver(0, 0);
  }

  //int sequenceScan[] = {80,90,100 };
  boolean tempFlag = 0;
  int cm1Temp[sizeof(sequenceOfScan)];
  int cm2Temp[sizeof(sequenceOfScan)];
 maneuver(0, 0);
  int minTempDistance = 1000;
  int temp1Distance;
  int temp2Distance;
  int temp3Distance;
  int temp = 90;
  boolean flag = 0;
  int tempValue1;
  int tempValue2;
  int tempDis = 1000;
  int tempDisCurrent;

  for (int k = 0; k < elementScanSequence ; k++) {
    turnSonarServoToCertainDegree(angleValueofsequenceOfScan[k], 100); // full rotation of servo is 250 miliseconds
    tempValue1 = 0;
    tempValue2 = 0;
    for (int t = 0; t < 5; t++) {
      tempValue1 += cmDistance(1);
      tempValue2 += cmDistance(2);
    } //t for
    cm1Temp[k] = tempValue1 / 5;
    cm2Temp[k] = tempValue2 / 5;
  } //k FOR

  int tempDegree;
  int tempDegreeK;
  int diffTemp;
  for (int k = 1; k < elementScanSequence ; k++) {
    temp1Distance = cm2Temp[findIn(k)];
    temp2Distance = cm2Temp[findIn(k - 1)];
    int tempWhatever = cm1Temp[findIn(k)];
    diffTemp = abs(tempWhatever - temp1Distance);

    if ((temp1Distance < maxDistanceWallObstacleDetect) && (temp2Distance < maxDistanceWallObstacleDetect) && ((diffTemp) > minDistanceBetweenWallAndObstacle)) {
      temp3Distance = (temp1Distance + temp2Distance) / 2;
      if (temp3Distance < minTempDistance) {
        minTempDistance = temp3Distance;
        tempDegreeK = k;
        flag = 1;
      }
    }
  } //For

  tempDegree = (angleValueofsequenceOfScan[findIn(tempDegreeK)] + angleValueofsequenceOfScan[findIn(tempDegreeK - 1)]) / 2;
  if (flag) {
    temp = getObjectRobotRelAngle(minTempDistance * 0.01, tempDegree);

    if (temp < 90) {
      maneuver(200, -200, (90 - temp)*msPerTurnDegree);
      maneuver(0, 0);
    } else if (temp > 90) {
      maneuver(-200, 200, (temp - 90)*msPerTurnDegree);
      maneuver(0, 0);
    }
    turnSonarServoToCertainDegree(90, 100);
    // int iter=0;
    for (int j = 0; j < 40; j++) {
      maneuver(linearForwardSpeed, linearForwardSpeed);
      delay(100);
      tempDisCurrent = (cmDistance(2) + cmDistance(2)) / 2;
      if (( cmDistance(1) < bumpedCmSonarReading) || (tempDisCurrent >= (tempDis - 5))) {
        maneuver(0, 0);
        return -1;
      }
      if (tempDisCurrent <= maxStopDistanceFromObstacle) {
        maneuver(0, 0);
        tempFlag = 1;
        break;
      }
      tempDis = (cmDistance(1) + cmDistance(1)) / 2;
    } //J for
  }
  if (tempFlag)
  {
    return 1;
  } else if (diffTemp < minDistanceBetweenWallAndObstacle)
  {
    return -1;
  } else
  {
    return 0;
  }
} //function




///////////////////////////////////////////////////////////////////////////// Hands functions ///////////////////////////////////////////////////////////////

/*
   Grap Obstacle
*/
void ClawGrip() {
  maneuver(50, 50);
  delay(500);
  for (int i = 20; i < 120; i += 20)
    servoClaw.write(i); // Grips the can;
  delay(200);
  maneuver(0, 0);
}

/*
   Release Obstacle
*/
void ClawRelease() {
  servoClaw.write(20); // Releases the can;
  delay(100);
  clawGrippedObjectFlag = false;

}

/////////////////////////////////////////////////////////////////////////IR RECEIVER////////////////////////////////////////////////////////////////////
/*
   Read IRF RECIEVER
*/
boolean irRead(int  readPin)
{
  pinMode(readPin, INPUT);
  delay(1);
  return analogRead(readPin);;
}

/*
   Get direction to safe zone function
*/
boolean GetDirectionToSafeZone() {

  int tempDegree;
  int temp;
  maneuver(-100, -100, 500);
  maneuver(0, 0);
  boolean flag = 0;
  for (int j = 0; j < 2; j++) {
    if (flag) break;

    for (int k = 0; k < elementScanSequence ; k++) {
      tempDegree = angleValueofsequenceOfScan[k];
      turnSonarServoToCertainDegree(tempDegree, 50); // full rotation of servo is 250 miliseconds
      if (irRead(IRFReceiverPinRight) == 0) {
        flag = 1;
        break;
      }
    } //FOR k
  }//For j

  if (flag) {
    temp = getObjectRobotRelAngle(1, tempDegree);
    if (temp < 90) {
      maneuver(200, -200, (90 - temp)*msPerTurnDegree);
      maneuver(0, 0);
    } else if (temp > 90) {
      maneuver(-200, 200, (temp - 90)*msPerTurnDegree);
      maneuver(0, 0);
    }
    return 1;
  } else
  {
    maneuver(200, -200, (180)*msPerTurnDegree); //Full rotate
    maneuver(0, 0);
  }

  return 0;
} //function








///////////////////////////////////////////////////////////////////////////////////////// DEBUGGERS ///////////////////////////////////////////////
//
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


void GetCloseToObstacleV2() {

  // maneuver(0, 0);
  // turnSonarServoToCertainDegree(90, 100);
  int temp1 = cmDistance(1);
  int temp2 = cmDistance(2);
  Serial.println("--------------------");
  Serial.print("CM1: "); Serial.println(temp1);
  Serial.print("CM2: "); Serial.println(temp2);
  delay(1);

}

int irDetect( )
{
  int irLedPin = 13;
  long frequency = 37500;
  tone(irLedPin, frequency, 8);             // IRLED 38 kHz for at least 1 ms
  delay(1);                                  // Wait 1 ms
  int ir = digitalRead(IRFReceiverPinRight);       // IR receiver -> ir variable
  //int ir = analogRead(1);
  delay(1);                                  // Down time before recheck
  return ir;                                 // Return 1 no detect, 0 detect
}

int irReadtest(int readPin, int triggerPin)
{
  int halfPeriod = 13; //one period at 38.5khZ is aproximately 26 microseconds
  int cycles = 38; //26 microseconds * 38 is more or less 1 millisecond
  int i;
  for (i = 0; i <= cycles; i++)
  {
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(halfPeriod);
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(halfPeriod - 1);     // - 1 to make up for digitaWrite overhead
  }
  return digitalRead(readPin);
}

