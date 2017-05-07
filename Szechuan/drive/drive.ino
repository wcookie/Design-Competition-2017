
#define XROW 1000
#define YROW 1000

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

void xbeeSetup(){
  Serial3.begin(9600);
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




void setup(){
  Serial.begin(9600);
  motorSetup();
  xbeeSetup();
}

void loop() {

  // call stuff in here
  // also if the directions are wrong, you can switch which wire goes to which out pin from the 
  // h bridge for that motor
  moveMotors(100, true, 100, true);
  delay(50000);

}

