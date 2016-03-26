

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
   
int setLeftWheelSpeed(int v): Function for setting left wheel signal   Input: v - addition in micro seconds to add to standby signal width
 
int setRightWheelSpeed(int v) : Function for setting right wheel signal  Input: v - addition in micro seconds to add to standby signal width

boolean sensorBelowThreshold(int Ax) :Function for checking if sensor reading is small enough

boolean sensorAboveThreshold(int Ax) :Function for checking if sensor reading is small enough

boolean checkPhototransistors() :Function for checking phototransistor values

boolean checkIfAtEdge() : Function for checking if robot is at an edge, i.e. one sensor gives large enough reading

float getVoltsByPin(int adPin) : Function for returning sensor value in volts

void stopRobot() : Function for stopping the servo motors

void stopTempRobot() :  Function for Temprary stop the servo motors

void moveForwardRobot() : Function for Move forward the servo motors

void turnAwayFromGoalArea() :   Turn over after the Robot is inside the goal area

void sonarServoTurn(Servo myservo, int sonarServoMinAngle, int sonarServoMaxAngle): Sonar servo turn
   
void turnAwayFromGoalArea() :Turn over after the Robot is inside the back area
    
int SonarReadingDistance() :Sonar reading function and its calculation 

long  rcTime(int pin) :  Measures and displays microsecond decay time for light sensor.

float differenceLeftRigthPhotoSensor(int leftPhotoSensorPin, int rightPhotoSensorPin) :  Function to calculate the normalzed of right and left photosensors ----> return Vlaue:[-0.5 0.5]=[left right]

*/






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
int turnTimer = 1200; // Turn timer about 180 degree
float photoTransistorThreshold = 0.25; // Sensor measuring parameters
float sensorErrorMarginCorner = 0.05;
int sonarServoMinAngle = 0;
int sonarServoMaxAngle = 90;

// Simulation parameters
int timeStep = 100; // time step in ms
int maxRunTime = 25; // max run time in seconds
int val1; //first reading of the sonar
int val2; // second reading of the sonar
int val3; //third reading of the sonar

// Simulation variables
int maxNumIterations;




////////////////////////////////// INITIALIZATION ///////////////////////////////////

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

  setLeftWheelSpeed(standardForwardSpeed);
  setRightWheelSpeed(standardForwardSpeed);

  Serial.begin(serialInputNumber); // Make console listen to serial input
  servoLeft.attach(leftWheelPin);
  servoRight.attach(rightWheelPin);

  //servoClaw.attach(clawPin); //Un-comment once claw servo has been dedicated to a pin and clawPin thereby has been set.
  //servoClaw.setMicroseconds(clawOpenPWMwidth); //Un-comment once servoClaw has been properly attached.

  sonarServo.attach(sonarServoPin);  // attaches the servo on pin 11 for sonar
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
     Should do obstacle detection here
  */
  // state change timer
  if ((nextStateTime >= currentTime) && (nextStateTime <= currentTime + timeStep)) {
    currentState = nextState;
    nextStateTime = -1; // reset timer
  }




  //////////////////////////////////////////////// FSM ///////////////////////////////////////////

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
    moveForwardRobot();


  } else if (currentState == 1) {
    stopTempRobot();
    //sonarServoTurn(sonarServo,sonarServoMinAngle,sonarServoMaxAngle);
   // sonarServoTurn(sonarServo, 0, 45);
    val1 = SonarReadingDistance();
Serial.print(SonarReadingDistance());
Serial.println("  \n ");
delay(500);
   // sonarServoTurn(sonarServo, 45, 90);
   // val2 = SonarReadingDistance();
   // Serial.print(SonarReadingDistance());
   // delay(500);
//Serial.println("  \n ");
  //  sonarServoTurn(sonarServo, 90, 125);
   // val3 = SonarReadingDistance();
   // Serial.print(SonarReadingDistance());
//Serial.println("  \n ");
//delay(500);
    //if (wall/conrner){
    // currentState = 2;
    //}else if(obstacle){
    //   currentState = 3;
    //}else if(goal area){
    // currentState = 9;
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
      //currentState = x; //After gripping, set current state to the state that finds the goal. ~( REMEMBER TO CHANGE x )~
      } else{
      //clawServo.writeMicroseconds(clawOpenPWMwidth);
      clawGrippedObject = false;
      //currentState = y; //After releasing, back away from the object to not drag it away from the goal. ~( REMEMBER TO CHANGE y )~
    }


    }


  delay(timeStep);
  numIterations++;

  printTransistorReadings(); // DEBUG
  }

*/




//////////////////////////////////////////////// FUNCTIONS /////////////////////////////////////////
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


/*
   Function for checking if sensor reading is small enough
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
  Function for Temprary stop the servo motors
*/
void stopTempRobot() {
  setLeftWheelSpeed(0);
  setRightWheelSpeed(0);
}


/*
  Function for Move forward the servo motors
*/
void moveForwardRobot() {
  setLeftWheelSpeed(standardForwardSpeed);
  setRightWheelSpeed(standardForwardSpeed);
}


/*
   Turn over after the Robot is inside the goal area
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


/*
 * Sonar servo turn
 */
void sonarServoTurn(Servo myservo, int sonarServoMinAngle, int sonarServoMaxAngle) {
  int pos;
  for (pos = sonarServoMinAngle; pos <= sonarServoMaxAngle; pos += 1) { // goes from min degrees to max degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  //for (pos = sonarServoMaxAngle; pos >= sonarServoMinAngle; pos -= 1) { // goes from 180 degrees to 0 degrees
   // myservo.write(pos);              // tell servo to go to position in variable 'pos'
   // delay(15);                       // waits 15ms for the servo to reach the position
 // }
}


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

  return (duration / 29 / 2);

}


/*
   Measures and displays microsecond decay time for light sensor.
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
