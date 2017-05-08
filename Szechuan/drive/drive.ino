#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>


#define XROW 9
#define YROW 9

#define V1PIN 24
#define DEG_PER_US 0.0216 // (180 deg) / (8333 us)
#define DEG_TO_RAD 3.14/180.0
#define LIGHTHOUSEHEIGHT 6.0

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

char msg[100];
char msg_index = 0;

bool grid[XROW][YROW]; // true means that there's obstacle, false otherwise.


// Four corner positions.  
//Back left is 0,0.  
//Back right is 0, 8.
//Front left is 8, 0
//Front right is 8,8
double x0;
double xMax;
double y0;
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

void gridSetup(){
  short x = 0;
  short y;
  for (x; x < 9; ++x){
    for (y= 0; y < 9; ++y){
      if ((x % 2) == 0 || (y%2) == 0){
        // basically every other x has nothing in it, including the two sides.
        // if it's not that, every other y has something in it (except the two sides)
        grid[x][y] = false;
      }
      else {
        grid[x][y] = true;
      }
    }
  }
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


void posToGrid(short &xCoord, short &yCoord){ 
  double xPercent = (xFilt - x0) / (xMax - x0);
  xCoord = round(xPercent * 8.0);
  double yPercent = (yFilt - y0) / (yMax - y0);
  yCoord = round(yPercent * 8.0);
}

void gridToPos(short xCoord, short yCoord, double &xExact, double &yExact){
  double xDiff = xMax - x0;
  double yDiff = yMax - y0;
  xExact = x0 + xDiff * (double) xCoord / 8.0;
  yExact = y0 + yDiff * (double) yCoord / 8.0;
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


void setup(){
  Serial.begin(9600);
  motorSetup();
  xbeeSetup();
  ltdSetup();
  gridSetup();
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
