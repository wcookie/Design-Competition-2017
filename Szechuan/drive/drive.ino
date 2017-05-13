#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>


LSM9DS1 imu;


// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW


#define PRINT_CALCULATED

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -3.58 // Declination (degrees) in Boulder, CO.

#define XROW 9
#define YROW 9

#define V1PIN 12
#define V2PIN 11 // the signal from the sensor
#define V3PIN 2
#define DEG_PER_US 0.0216 // (180 deg) / (8333 us)
#define LIGHTHOUSEHEIGHT 6.0
#define NEUTRAL_POWER 150
#define EXTREME_NEUTRAL_POWER 150
#define DISTANCE_WEIGHT 5.0
#define EYESIGHT_WEIGHT 10.0
#define WALL_WEIGHT 1.0
#define slightkP 2
#define extremekP .8
#define extremekI 0
typedef struct {
  unsigned long changeTime[11];
  int prevPulse;
  double horzAng;
  double vertAng;
  int useMe;
  int collected;
} viveSensor;

typedef struct {
  double x;
  double y;
}lightPoint;

volatile viveSensor V1;
volatile viveSensor V2;
volatile viveSensor V3;
unsigned long prevTime = 0;
unsigned long prevTime2 = 0;
unsigned long prevTime3 = 0;
unsigned long lastYawTime = 0;
unsigned long lastEnemyTime = 0;
unsigned long lastFilt1Time = 0;
unsigned long lastFilt2Time = 0;
unsigned long lastFilt3Time = 0;
int state = 0;
double xCombo = 0, yCombo = 0;
double xOld1 = 0, yOld1 = 0, xFilt1 = 0, yFilt1 = 0;
double xOld2 = 0, yOld2 = 0, xFilt2 = 0, yFilt2 = 0;
double xOld3 = 0, yOld3 = 0, xFilt3 = 0, yFilt3 = 0;
double enemyX, enemyY;
lightPoint enemyArr[10];
lightPoint filterPosArr[30];
char filter1PosIndex = 0;
char filter2PosIndex = 0;
char filter3PosIndex = 0;
char enemyIndex = 0;
double desiredYaw = 0 ;
short ourLastX;
short ourLastY;
short theirLastX;
short theirLastY;
short ourCurrX;
short ourCurrY;
short theirCurrX;
short theirCurrY;
char msg[100];
char msg_index = 0;
double gyroYaw = 0;
double accelYaw = 0;
double complementaryYaw = 0;
double yawITerm = 0;

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
int frontLeftA = 22;
int frontLeftD = 16;
int backLeftA = 20;
int backLeftD = 14;

int frontRightA = 23;
int frontRightD = 17;
int backRightA = 21;
int backRightD = 15;

bool slight = true;

//Setup the serial for the xbee
void xbeeSetup(){
  Serial1.begin(9600);
  lastEnemyTime = millis();
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
  pinMode(V2PIN, INPUT); // to read the sensor
  // initialize the sensor variables
  V2.horzAng = 0;
  V2.vertAng = 0;
  V2.useMe = 0;
  V2.collected = 0;
  // interrupt on any sensor change
  attachInterrupt(digitalPinToInterrupt(V2PIN), ISRV2, CHANGE);
  pinMode(V3PIN, INPUT); // to read the sensor
  // initialize the sensor variables
  V3.horzAng = 0;
  V3.vertAng = 0;
  V3.useMe = 0;
  V3.collected = 0;
  // interrupt on any sensor change
  attachInterrupt(digitalPinToInterrupt(V3PIN), ISRV3, CHANGE);
  lastFilt1Time = millis();
  lastFilt2Time = millis();
  lastFilt3Time = millis();
}


void imuSetup() 
{

  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  while (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
  
  }
  imu.calibrate(true);
  imu.calibrateMag(true);
  lastYawTime = micros();
}

void imuReadVals(){
  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
  }
}


double calcYaw(){
 /*double heading;
 double mx, my;
 mx = imu.calcMag(imu.mx);
 my = imu.calcMag(imu.my);
  if (my == 0)
  {
    heading = (mx < 0) ? PI : 0;
  }
  else{
    heading = (double)  atan2(mx, my);
  }
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  return heading;*/
  imuReadVals();
  double gyroZ = imu.calcGyro(imu.gz);
  unsigned long tempTime= micros();
  gyroYaw += gyroZ * (tempTime - lastYawTime) * .000001;
  lastYawTime = tempTime;
  return fmod(gyroYaw, 360);
}

  


void getEnemyPosition(){  
  
    double tempX;
    double tempY;
   if (Serial1.available() > 0) {
   msg[msg_index] = Serial1.read();
   //Serial.print(msg[msg_index]);
   if (msg[msg_index] == '\n') {
     sscanf(msg, "%lf %lf", &tempX, &tempY);  
     msg_index = 0;
   }
   else {
     msg_index++;
     if (msg_index == 100) {
       msg_index = 0;
     }
   }
   lightPoint tempE;
   tempE.x = tempX;
   tempE.y = tempY;
    double summx = 0;
    double summy = 0;
    short numEnemies = 0;
    for(short i = 0; i < 10; ++i){
      if (enemyArr[i].x > - 100 && enemyArr[i].y > -100){
        summx += enemyArr[i].x;
        summx += enemyArr[i].y;
        ++numEnemies;
    }
   }
   if(abs(tempX - enemyX) < 1.5 || abs(tempY - enemyY) < 1.5 || ((millis() - lastEnemyTime) > 100)){
     enemyArr[enemyIndex] = tempE;
     ++enemyIndex;
     if(enemyIndex > 9){
      enemyIndex = 0;
     }
   }
   else{
    enemyX = summx / numEnemies;
    enemyY = summy / numEnemies;
    return;
   }
  
 
   if (numEnemies > 0 ){
     enemyX = .7 * summx / numEnemies + .3 * tempX;
     enemyY = .7 * summy / numEnemies + .3 * tempY;
   }
   else{
    enemyX = tempX;
    enemyY = tempY;
   }
 }
 

}

double toDegrees(double rads){
  return rads * 180.0 / M_PI;
}



// should take it so it has arguments
void posToGrid(short &xCoord, short &yCoord, double xInGrid, double yInGrid){ 
  double xPercent = (xInGrid - xInit) / (xMax - xInit);
  xCoord = round(xPercent * 8.0);
  double yPercent = (yInGrid - yInit) / (yMax - yInit);
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




void findPosition(double &xOld, double &yOld, double &xFilt, double &yFilt, short num){
      double xPos = 0;
      double yPos = 0;
      /*Serial.print("Num: \t");
      Serial.print(num);
      Serial.print("\r\n");*/
      char tempIndex;
      if (num == 1){
        V1.useMe = 0;
        xPos = tan((V1.vertAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
        yPos = tan((V1.horzAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
       if(abs(xPos - xFilt1) < 1.5 || abs(yPos - yFilt1) < 1.5 || ((millis() - lastFilt1Time) > 100)){
         tempIndex = filter1PosIndex;
         ++filter1PosIndex;
         if (filter1PosIndex > 9){
           filter1PosIndex = 0;
         }
         lastFilt1Time = millis();
       }
       else{
        return;
       }
      }
      else if (num == 2){
        V2.useMe = 0;
        xPos = tan((V2.vertAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
        yPos = tan((V2.horzAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT; 
        if(abs(xPos - xFilt2) < 1.5 || abs(yPos - yFilt2) < 1.5 || ((millis() - lastFilt2Time) > 100)){
         tempIndex = filter2PosIndex;
         ++filter2PosIndex;
         if (filter2PosIndex > 9){
           filter2PosIndex = 0;
         }
         lastFilt2Time = millis();
       }
       else{
        return;
       }
      }
      else if (num == 3){
        V3.useMe = 0;
        xPos = tan((V3.vertAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT;
        yPos = tan((V3.horzAng - 90.0) * DEG_TO_RAD) * LIGHTHOUSEHEIGHT; 
        if(abs(xPos - xFilt3) < 1.5 || abs(yPos - yFilt3) < 1.5 || ((millis() - lastFilt3Time) > 100)){
         tempIndex = filter3PosIndex;
         ++filter3PosIndex;
         if (filter3PosIndex > 9){
           filter3PosIndex = 0;
         }
         lastFilt3Time = millis();
       }
       else{
        return;
       }
      }
      //set to 0 for array indexing.
      --num;
      
      
      double summx = 0;
      double summy = 0;
      double numPoints = 0;
      for (short tempIndex2 = (num * 10); tempIndex < (num + 1) * 10; ++tempIndex2){
        if (filterPosArr[tempIndex2].x > - 100 && filterPosArr[tempIndex2].y > -100){
          summx += filterPosArr[tempIndex2].x;
          summx += filterPosArr[tempIndex2].y;
          ++numPoints;
          }
      }
      lightPoint tempPoint;
      tempPoint.x = xPos;
      tempPoint.y = yPos;
      filterPosArr[num * 10 + tempIndex] = tempPoint;
      double xavg = summx / numPoints;
      double yavg = summy / numPoints;
     
        if (numPoints > 0 ){
          xFilt = .7 * xavg + .3 * xPos;
          yFilt = .7 * yavg + .3 * yPos;
        }
        else{
          xFilt = xPos;
          yFilt = yPos;
        }
      

     /* xFilt = xOld * 0.5 + xPos * 0.5;
      yFilt = yOld * 0.5 + yPos * 0.5;

      xOld = xFilt;
      yOld = yFilt;*/
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

// if either we or they change pos, do stuff 
bool needToRecalc(){
  return false;
 // if(ourLastX != g
}


void setup(){
  Serial.begin(9600);
  motorSetup();
  xbeeSetup();
  ltdSetup();
  imuSetup();
  
  desiredYaw = 90;
}



void loop() {

  // call stuff in here
  // also if the directions are wrong, you can switch which wire goes to which out pin from the 
  // h bridge for that motor

    if (micros() - prevTime > 1000000 / 25) {
    if (V1.useMe == 1) {
      prevTime = micros();
      findPosition(xOld1, yOld1, xFilt1, yFilt1, 1);
    }
    }
    if (micros() - prevTime2 > 1000000 / 25){
      if (V2.useMe == 1){
        prevTime2 = micros();
        findPosition(xOld2, yOld2, xFilt2, yFilt2, 2);
    }
    }  
    if (micros() - prevTime2 > 1000000 / 25){
    if (V3.useMe == 1){
      prevTime3 = micros();
      findPosition(xOld3, yOld3, xFilt3, yFilt3, 2);
    }
    }
   
       
    
     getEnemyPosition();
 

    //calc average from lighthouse
    double newXCombo = (xFilt3 + xFilt2 + xFilt1) / 3.0;
    double newYCombo = (yFilt3 + yFilt2 + yFilt1) / 3.0;

    



    // eliminate outliers
    // :GENERAL STRAT:
    // calculate how far new x is away from last xcombo.  
    // if the idfference between one of lighthouse and other 2 is more than that diff,
    // then ignore it2
    // maybe calc sum and the # of divisors.  decide to subtract from sum and ALSO subtract from # of divisors.
    // Don't forget; if # of divisors is 0, then use old xCombo.   
 



    // after we have the finalized combos in newXCombo and newYCombo::
    // PUT IT INTO Grid woooo
    // Also saving into curr
    ourLastX = ourCurrX;
    ourLastY = ourCurrY;
    posToGrid(ourCurrX, ourCurrY, newXCombo, newYCombo);
    //imu stuff:
   // imuReadVals();
    // get yaw
  /*  double gyroZ = imu.calcGyro(imu.gz);
    unsigned long tempTime= micros();
    gyroYaw += gyroZ * (tempTime - lastYawTime) * .000001;
    lastYawTime = tempTime;
    Serial.print("Gyro z: \t");
    Serial.println(gyroZ);
    Serial.print("total gyroZ: \t");
    Serial.println(gyroYaw);
    Serial.print("Accel z: \t");
    Serial.println(accelZ);
    Serial.print("total Accel: \t");
    Serial.println(accelYaw);
    Serial.print("copmlmenetary Yaw: \t");
    Serial.println(complementaryYaw);
    */
    double yaw = calcYaw();
    double rightMotorSpeed;
    double leftMotorSpeed;
    // NEED TO DO THIS WITH THE 360 wrap around thing
    if (desiredYaw > 360){
      desiredYaw = fmod(desiredYaw, 360);
    }
    double diff = yaw - desiredYaw;
    if (diff > 180){
      diff = diff - 360;
    }
    else if (diff < -180){

    diff = 360 - abs(diff);
    }
    yawITerm += diff;
    if (abs(diff) >25){
      slight = false;
    }
    if(slight){    
      rightMotorSpeed = NEUTRAL_POWER + (diff * slightkP) + (extremekI * yawITerm);
      leftMotorSpeed = NEUTRAL_POWER - (diff * slightkP) - (extremekI * yawITerm);
    
      if (leftMotorSpeed > 255){
        leftMotorSpeed = 255;
      }
      else if (leftMotorSpeed < 0){
        leftMotorSpeed = 0;
      }
      if (rightMotorSpeed > 255){
        rightMotorSpeed = 255;
      }
      else if (rightMotorSpeed < 0){
        rightMotorSpeed = 0;
      }
      //3.6 x 4.2 y
      //4.2 x -5.6 y
      // -5.9x -6.2 y
      //- 6x 3.8 y
   // moveMotors(leftMotorSpeed, true, rightMotorSpeed, true);
    }
    else if (abs(diff) > 1){
      
      rightMotorSpeed = EXTREME_NEUTRAL_POWER + (abs(diff) * extremekP) + (extremekI * yawITerm);;
      leftMotorSpeed = EXTREME_NEUTRAL_POWER + (abs(diff) * extremekP) - (extremekI * yawITerm);;
      if (leftMotorSpeed > 255){
        leftMotorSpeed = 255;
      }
      else if (leftMotorSpeed < 0){
        leftMotorSpeed = 0;
      }
      if (rightMotorSpeed > 255){
        rightMotorSpeed = 255;
      }
      else if (rightMotorSpeed < 0){
        rightMotorSpeed = 0;
      }
      // if we turn left:
      if (diff > 0){
      //  moveMotors(leftMotorSpeed, false, rightMotorSpeed, true);       
      }
      // if we go right tho
      else{
    //    moveMotors(leftMotorSpeed, true, rightMotorSpeed, false); 
      }
    }
    else{
      slight = true;
    }
   
    //print stuff
    /*
    Serial.print("Xfilt1: \t");
    Serial.print(xFilt1);
    Serial.print("\t");
    Serial.print("Yfilt1: \t");
    Serial.print(yFilt1);
    Serial.print("\r\n");
    Serial.print("Xfilt2: \t");
    Serial.print(xFilt2);
    Serial.print("\t");
    Serial.print("Yfilt2: \t");
    Serial.print(yFilt2);
    Serial.print("\r\n");  
    Serial.print("Xfilt3: \t");
    Serial.print(xFilt2);
    Serial.print("\t");
    Serial.print("Yfilt3: \t");
    Serial.print(yFilt2);
    Serial.print("\r\n");
    Serial.print("Enemy xPos: \t");
    Serial.print(enemyX);
    Serial.print("\t");
    Serial.print("Enemy yPos: \t");
    Serial.print(enemyY);
    Serial.print("\r\n");
    
    Serial.print("Yaw: \t");
    Serial.print(yaw);
    Serial.print("\r\n");
    Serial.print("Desired yaw \t");
    Serial.print(desiredYaw);
    Serial.print("\r\n");
    Serial.print(diff);
    Serial.print("\r\n");
    Serial.print("LEft Power: \t");
    Serial.print(leftMotorSpeed);
    Serial.print("\r\n");
    Serial.print("Right motor power: \t");
    Serial.print(rightMotorSpeed);
    Serial.print("\r\n");
 */
    Serial.print("Xcombo: \t");
    Serial.println(newXCombo);
    Serial.print("Ycombo: \t");
    Serial.println(newYCombo); 
   

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


// the second ensor interrupt
void ISRV2() {
  // get the time the interrupt occured
  unsigned long mic = micros();
  int i;

  // shift the time into the buffer
  for (i = 0; i < 10; i++) {
    V2.changeTime[i] = V2.changeTime[i + 1];
  }
  V2.changeTime[10] = mic;

  // if the buffer is full
  if (V2.collected < 11) {
    V2.collected++;
  }
  else {
    // if the times match the waveform pattern
    if ((V2.changeTime[1] - V2.changeTime[0] > 7000) && (V2.changeTime[3] - V2.changeTime[2] > 7000) && (V2.changeTime[6] - V2.changeTime[5] < 50) && (V2.changeTime[10] - V2.changeTime[9] < 50)) {
      V2.horzAng = (V2.changeTime[5] - V2.changeTime[4]) * DEG_PER_US;
      V2.vertAng = (V2.changeTime[9] - V2.changeTime[8]) * DEG_PER_US;
      V2.useMe = 1;
    }
  }
}

//3rd sensor interrupt
void ISRV3() {
  // get the time the interrupt occured
  unsigned long mic = micros();
  int i;

  // shift the time into the buffer
  for (i = 0; i < 10; i++) {
    V3.changeTime[i] = V3.changeTime[i + 1];
  }
  V3.changeTime[10] = mic;

  // if the buffer is full
  if (V3.collected < 11) {
    V3.collected++;
  }
  else {
    // if the times match the waveform pattern
    if ((V3.changeTime[1] - V3.changeTime[0] > 7000) && (V3.changeTime[3] - V3.changeTime[2] > 7000) && (V3.changeTime[6] - V3.changeTime[5] < 50) && (V3.changeTime[10] - V3.changeTime[9] < 50)) {
      V3.horzAng = (V3.changeTime[5] - V3.changeTime[4]) * DEG_PER_US;
      V3.vertAng = (V3.changeTime[9] - V3.changeTime[8]) * DEG_PER_US;
      V3.useMe = 1;
    }
  }
}
