#include "motors.h"

#define PIN1 3
#define PIN2 4
#define PIN3 5
#define PIN4 6


void initMotors(int pin1, int pin2, int pin3, int pin4){
    softPwmCreate(pin1, 0, 100);
    softPwmCreate(pin2, 0, 100);
    softPwmCreate(pin3, 0, 100);
    softPwmCreate(pin4, 0, 100);
}


void moveForward(int speed){
    // all go at speed speed

}
