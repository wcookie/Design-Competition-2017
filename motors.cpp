#include "motors.h"

int frontLeftA; //Front left
int frontLeftD;
int frontRightA; //Front right
int frontRightD;
int backLeftA; //Back left
int backLeftD;
int backRightA; //Back right
int backRightD;

void initMotors(){
    softPwmCreate(frontLeftA, 0, 100);
    softPwmCreate(frontRightA, 0, 100);
    softPwmCreate(backLeftA, 0, 100);
    softPwmCreate(backRightA, 0, 100);
    pinMode(frontLeftD, OUTPUT);
    pinMode(frontRightD, OUTPUT);
    pinMode(backLeftD, OUTPUT);
    pinMode(backRightD, OUTPUT);
}


void moveForward(int speed, bool direction){
    // all go at speed speed
    softPwmWrite(frontLeftA, speed);
    softPwmWrite(frontRightA, speed);
    softPwmWrite(backLeftA, speed);
    softPwmWrite(backRightA, speed);
    digitalWrite(frontLeftD, HIGH);
    digitalWrite(frontRightD, HIGH);
    digitalWrite(backLeftD, HIGH);
    digitalWrite(backRightD, HIGH);
}

void turnLeft(int speed, int steepness){
    // want to have right more powerful than left.
    

