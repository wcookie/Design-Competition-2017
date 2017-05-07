#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>


#define XROW 1000
#define YROW 1000

#define V1PIN 24
#define DEG_PER_US 0.0216 // (180 deg) / (8333 us)
#define DEG_TO_RAD 3.14/180.0
#define LIGHTHOUSEHEIGHT 6.0

typedef struct {
  unsigned long changeTime[3];
  int prevPulse;
  double horzAng;
  double vertAng;
  int useMe;
  int firstTime;
} viveSensor;

volatile viveSensor V1;
unsigned long prevTime = 0;
int state = 0;
double xPos, yPos;
double xOld = 0, yOld = 0, xFilt = 0, yFilt = 0;


bool grid[XROW][YROW]; // true means that there's obstacle, false otherwise.

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
  V1.firstTime = 1;

  attachInterrupt(digitalPinToInterrupt(V1PIN), ISRV1, CHANGE);
}
void getEnemyPosition(float &xPos, float&yPos){  
   char msg[20];
   char msg_index = 0;
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



void findPosition(float &xOld, &yOld, &xFilt, &yFilt){
      V1.useMe = 0;

      xPos = tan((V1.vertAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
      yPos = tan((V1.horzAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;

      xFilt = xOld * 0.8 + xPos * 0.2;
      yFilt = yOld * 0.8 + yPos * 0.2;

      xOld = xFilt;
      yOld = yFilt;
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
  

}

void ISRV1() {
  int val = digitalReadFast(V1PIN);
  int pulseTime;

  V1.changeTime[0] = V1.changeTime[1];
  V1.changeTime[1] = V1.changeTime[2];
  V1.changeTime[2] = micros();

  if (val == 1) {
    pulseTime = V1.changeTime[2] - V1.changeTime[1];
    if (pulseTime < 50) {
      if (V1.firstTime == 1) {
        V1.prevPulse = pulseTime;
        V1.firstTime = 0;
      }
      else {
        if (pulseTime < V1.prevPulse) {
          V1.horzAng = (V1.changeTime[1] - V1.changeTime[0]) * DEG_PER_US;
        }
        else {
          V1.vertAng = (V1.changeTime[1] - V1.changeTime[0]) * DEG_PER_US;
        }
        V1.prevPulse = pulseTime;
        V1.useMe = 1;
      }
    }
  }
}

