#include "motors.h"

int APIN1; //Front left
int DPIN1;
int APIN2; //Front right
int DPIN2;
int APIN3; //Back left
int DPIN3;
int APIN4; //Back right
int DPIN4;

void initMotors(int pin1, int pin2, int pin3, int pin4){
    softPwmCreate(pin1, 0, 100);
    softPwmCreate(pin2, 0, 100);
    softPwmCreate(pin3, 0, 100);
    softPwmCreate(pin4, 0, 100);
    APIN1 = pin1;
    APIN2 = pin2;
    APIN3 = pin3;
    APIN4 = pin4;
}


void moveForward(int speed, bool direction){
    // all go at speed speed
    softPwmWrite(PIN1, speed);
    softPwmWrite(PIN2, speed);
    softPwmWrite(PIN3, speed);
    softPwmWrite(PIN4, speed);
}

void turnLeft(int speed, int steepness){
    // want to have right more powerful than left.
