#include <wiringPi.h>
#include <softPwm.h>

void initMotors(int pin1, int pin2, int pin3, int pin4);


//Speed between 0 and 100 (seems like everything between 0 and 100)
// Direction 1 for forwards, 0 for backwards

void goForward(int speed, bool direction); 


//turn left or right at such steepness between 0 and 100,
// where 100 is one motor going forwards at 100 % of speed other going
//back at 100%, and everything in between, 0 is just going same exact speed.
void turnLeft(int speed, int steepness);

void turnRight(int speed, int steepness);



