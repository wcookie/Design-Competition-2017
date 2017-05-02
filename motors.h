#include <wiringPi.h>
#include <softPwm.h>


//Speed between 0 and 255 (seems like everything between 0 and 255)

void goForward(int speed);


//turn left or right at such steepness between 0 and 100,
// where 100 is one motor going forwards at 100 % other going
//back at 100%, and everything in between.
void turnLeft(int steepness);

void turnRight(int steepness);



