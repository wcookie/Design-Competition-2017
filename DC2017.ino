//these pin #s were chosen randomly.  These are for h bridge stuff,
// make sure the A pins are actually PWM pins from teensy lc pinout
// Make sure that the d pins are preferably normal pins that don't have anlaog in
// A pins go to the analog input for the h bridge (enable)
// D pins go t othe digital input for H bridge (phase)

// A pins: PWM yellow/white
// D pins: blue/green

// pin5 connection is bad
int frontLeftA = 4;
int frontLeftD = 9;
int backLeftA = 6;
int backLeftD = 11;

int frontRightA = 20;
int frontRightD = 10;
int backRightA = 16;
int backRightD = 17;

int mode1 = 15;
int mode2 = 26;


//////////////////////
// values to adjust //
//////////////////////

float speedPercentage = 20;

int frontLeftVoltage  = int(2.55*speedPercentage*.96);
int backLeftVoltage   = int(2.55*speedPercentage*.98);
int frontRightVoltage = int(2.55*speedPercentage*1);
int backRightVoltage   = int(2.55*speedPercentage*.94);

int tc = 2000;    // time constant, number of milliseconds it takes to move forward one unit
int tcr = 540;   // time constant for right turn, milliseconds to turn 90 degrees right
int tcl = 540;   // time constant for right left, milliseconds to turn 90 degrees right


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

void forward(int units){

  analogWrite(frontLeftA, frontLeftVoltage);
  analogWrite(backLeftA, backLeftVoltage);

  analogWrite(frontRightA, frontRightVoltage);
  analogWrite(backRightA, backRightVoltage);
  
  digitalWrite(frontLeftD, true);
  digitalWrite(backLeftD, true);
  digitalWrite(frontRightD, true);
  digitalWrite(backRightD, true);

  delay(tc*units);
  
}

void backward(int units){

  analogWrite(frontLeftA, frontLeftVoltage);
  analogWrite(backLeftA, backLeftVoltage);

  analogWrite(frontRightA, frontRightVoltage);
  analogWrite(backRightA, backLeftVoltage);
  
  digitalWrite(frontLeftD, false);
  digitalWrite(backLeftD, false);
  digitalWrite(frontRightD, false);
  digitalWrite(backRightD, false);

  delay(tc*units);
  
}

void turnRight(){
  analogWrite(frontLeftA, frontLeftVoltage);
  analogWrite(backLeftA, backLeftVoltage);

  analogWrite(frontRightA, frontRightVoltage);
  analogWrite(backRightA, backLeftVoltage);
  
  digitalWrite(frontLeftD, true);
  digitalWrite(backLeftD, true);
  digitalWrite(frontRightD, false);
  digitalWrite(backRightD, false);

  delay(tcr);
  
}

void turnLeft(){
  analogWrite(frontLeftA, frontLeftVoltage);
  analogWrite(backLeftA, backLeftVoltage);

  analogWrite(frontRightA, frontRightVoltage);
  analogWrite(backRightA, backLeftVoltage);
  
  digitalWrite(frontLeftD, false);
  digitalWrite(backLeftD, false);
  digitalWrite(frontRightD, true);
  digitalWrite(backRightD, true);

  delay(tcl);
  
}

void wait(int t){

  analogWrite(frontLeftA, 0);
  analogWrite(backLeftA, 0);

  analogWrite(frontRightA, 0);
  analogWrite(backRightA, 0);
  
  digitalWrite(frontLeftD, false);
  digitalWrite(backLeftD, false);
  digitalWrite(frontRightD, false);
  digitalWrite(backRightD, false);

  delay(t);
  
}

void setup() {
  // set the H bridge pins to all output mode
  pinMode(frontLeftA, OUTPUT);
  pinMode(frontLeftD, OUTPUT);
  pinMode(backLeftA, OUTPUT);
  pinMode(backLeftD, OUTPUT);
  pinMode(frontRightA, OUTPUT);
  pinMode(frontRightD, OUTPUT);
  pinMode(backRightA, OUTPUT);
  pinMode(backRightD, OUTPUT);
  pinMode(mode1, OUTPUT);
  pinMode(mode2, OUTPUT);

}


void loop() {

  // call stuff in here
forward(1);
  wait(3000);

}
