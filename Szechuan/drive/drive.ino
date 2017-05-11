#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>


#define XROW 9
#define YROW 9

#define V1PIN 24
#define DEG_PER_US 0.0216 // (180 deg) / (8333 us)
#define LIGHTHOUSEHEIGHT 6.0

#define DISTANCE_WEIGHT 5.0
#define EYESIGHT_WEIGHT 10.0
#define WALL_WEIGHT 1.0
typedef struct {
  unsigned long changeTime[11];
  int prevPulse;
  double horzAng;
  double vertAng;
  int useMe;
  int collected;
} viveSensor;


volatile viveSensor V1;
unsigned long prevTime = 0;
int state = 0;
double xOld = 0, yOld = 0, xFilt = 0, yFilt = 0;
double enemyX, enemyY;

short ourLastX;
short ourlastY;
short theirLastX;
short theirLastY;

char msg[100];
char msg_index = 0;

// Four corner positions.  
//Back left is 0,0.  
//Back right is 0, 8.
//Front left is 8, 0
//Front right is 8,8
double xInit;
double xMax;
double yInit;
double yMax;

//these pin #s were chosen randomly.  These are for h bridge stuff,
// make sure the A pins are actually PWM pins from teensy lc pinout
// Make sure that the d pins are preferably normal pins that don't have anlaog in
// A pins go to the analog input for the h bridge (enable)
// D pins go t othe digital input for H bridge (phase)
int frontLeftA = 4;
int frontLeftD = 9;
int backLeftA = 6;
int backLeftD = 11;

int frontRightA = 20;
int frontRightD = 10;
int backRightA = 16;
int backRightD = 17;

//Setup the serial for the xbee
void xbeeSetup(){
  Serial3.begin(9600);
}

//setup the light to digital sensor(s?)
void ltdSetup(){
  pinMode(V1PIN, INPUT);
  V1.prevPulse = 0;
  V1.horzAng = 0;
  V1.vertAng = 0;
  V1.useMe = 0;
  V1.collected = 0;
  attachInterrupt(digitalPinToInterrupt(V1PIN), ISRV1, CHANGE);
}




void getEnemyPosition(double &xPos, double &yPos){  
  

   if (Serial3.available() > 0) {
   msg[msg_index] = Serial3.read();
   //Serial.print(msg[msg_index]);
   if (msg[msg_index] == '\n') {

     sscanf(msg, "%f %f", &xPos, &yPos);  
     msg_index = 0;
   }
   else {
     msg_index++;
     if (msg_index == 100) {
       msg_index = 0;
     }
   }
 }

}

// should take it so it has arguments
void posToGrid(short &xCoord, short &yCoord){ 
  double xPercent = (xFilt - xInit) / (xMax - xInit);
  xCoord = round(xPercent * 8.0);
  double yPercent = (yFilt - yInit) / (yMax - yInit);
  yCoord = round(yPercent * 8.0);
}

void gridToPos(short xCoord, short yCoord, double &xExact, double &yExact){
  double xDiff = xMax - xInit;
  double yDiff = yMax - yInit;
  xExact = xInit + xDiff * (double) xCoord / 8.0;
  yExact = yInit + yDiff * (double) yCoord / 8.0;
}

//read X and Y seperated by space.

//eventualy make initMotors() function
//left and right speeds between 0 and 255
//for directions True is forwards, False is backwards
void moveMotors(int leftSpeed, bool leftDirection, int rightSpeed, bool rightDirection) {
  analogWrite(frontLeftA, leftSpeed);
  analogWrite(backLeftA, leftSpeed);
  digitalWrite(frontLeftD, leftDirection);
  digitalWrite(backLeftD, leftDirection);
  analogWrite(frontRightA, rightSpeed);
  analogWrite(backRightA, rightSpeed);
  digitalWrite(frontRightD, rightDirection);
  digitalWrite(backRightD, rightDirection);

}

//setup the outputs for hbridge
void motorSetup() {
  // set the H bridge pins to all output mode
  pinMode(frontLeftA, OUTPUT);
  pinMode(frontLeftD, OUTPUT);
  pinMode(backLeftA, OUTPUT);
  pinMode(backLeftD, OUTPUT);
  pinMode(frontRightA, OUTPUT);
  pinMode(frontRightD, OUTPUT);
  pinMode(backRightA, OUTPUT);
  pinMode(backRightD, OUTPUT);
}




void findPosition(double &xOld, double &yOld, double &xFilt, double &yFilt){
      V1.useMe = 0;

      double xPos = tan((V1.vertAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
      double yPos = tan((V1.horzAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;

      xFilt = xOld * 0.5 + xPos * 0.5;
      yFilt = yOld * 0.5 + yPos * 0.5;

      xOld = xFilt;
      yOld = yFilt;
}

//true meaning this grid position would have the potential for eyesight between our robot and the enemies
//takes only rid positions

//also may want to change this to a double s.t. we talk about how much eyesight we have
bool hasEyeSight(short goalX, short goalY, short baddieX, short baddieY){
  //horizontal
  if ((goalX == baddieX) && (goalX % 2 == 0)){
    return true;
  }

  //vertical
  if ((goalY == baddieY) && (goalY % 2 == 0)){
    return true;
  }
  
  //diagonal
   if (abs(baddieY - goalY) == abs(baddieX - goalX)){
    return true;
   }
   return false;
}

// calculating grid distance
double distanceFunc(short goalX, short goalY, short baddieX, short baddieY){
  short xDiff = baddieX - goalX;
  short yDiff = baddieY - goalY;
  return sqrt((xDiff * xDiff) + (yDiff * yDiff));
}

// overall goodness of function, using the weights above. 
//consider changing the baddie x and y to just be using the enemy x and y or something
// may not be necessary tho, only needs to happen once anyways
// we want the lowest thing possible
// basic idea is distance goodness - eyesight badness
// may / PROBABLY want to take into account where we currently are -- 
// f.e. if there is a slightly better spot really far away from us than one very close, probably worth just going to the one close.
double desirability(short goalX, short goalY, short baddieX, short baddieY){
  //this is function that should definitely be talked about more and changed.
  return DISTANCE_WEIGHT * distanceFunc(goalX, goalY, baddieX, baddieY) - EYESIGHT_WEIGHT * (double) hasEyeSight(goalX, goalY, baddieX, baddieY);
}


void bestCoords(short &decidedX, short &decidedY, short baddieX, short baddieY){
  double bestDistance = 10000;
  short xCounter = 0;
  short yCounter;
  double temp;
  for (xCounter; xCounter < 9; ++xCounter){
    for (yCounter = 0; yCounter < 9; ++yCounter){
      // if one of the two is even, we're not in an obstacle
      if ((xCounter % 2 == 0) || (yCounter % 2 == 0)){
        temp = desirability(xCounter, yCounter, baddieX, baddieY);
        // now if this is a more desirable position:
        if (temp < bestDistance){
          // set the pass by reference vals to be the current best.
          decidedX = xCounter;
          decidedY = yCounter;
          // update the current best
          bestDistance = temp;
        }
      }
    }
  }
}

// if either we or 
bool needToRecalc(short ourX, short ourY, short baddieX, short baddieY){
  
}


void setup(){
  Serial.begin(9600);
  motorSetup();
  xbeeSetup();
  ltdSetup();
}



void loop() {

  // call stuff in here
  // also if the directions are wrong, you can switch which wire goes to which out pin from the 
  // h bridge for that motor
    moveMotors(100, true, 100, true);
    if (micros() - prevTime > 1000000 / 25) {
    if (V1.useMe == 1) {
      prevTime = micros();
      findPosition(xOld, yOld, xFilt, yFilt);
    }
    }
    getEnemyPosition(enemyX, enemyY);
    
    
  

}
void ISRV1() {
  // get the time the interrupt occured
  unsigned long mic = micros();
  int i;

  // shift the time into the buffer
  for (i = 0; i < 10; i++) {
    V1.changeTime[i] = V1.changeTime[i + 1];
  }
  V1.changeTime[10] = mic;

  // if the buffer is full
  if (V1.collected < 11) {
    V1.collected++;
  }
  else {
    // if the times match the waveform pattern
    if ((V1.changeTime[1] - V1.changeTime[0] > 7000) && (V1.changeTime[3] - V1.changeTime[2] > 7000) && (V1.changeTime[6] - V1.changeTime[5] < 50) && (V1.changeTime[10] - V1.changeTime[9] < 50)) {
      V1.horzAng = (V1.changeTime[5] - V1.changeTime[4]) * DEG_PER_US;
      V1.vertAng = (V1.changeTime[9] - V1.changeTime[8]) * DEG_PER_US;
      V1.useMe = 1;
    }
  }
}
