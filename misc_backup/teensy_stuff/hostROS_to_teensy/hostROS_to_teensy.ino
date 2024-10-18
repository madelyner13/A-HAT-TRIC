// FOR MANUAL CONTROL OF SERVOS FROM LAPTOP
// to be tested on VOXL2 platform...

#include <Servo.h>

// data read-in
int incomingBytes[4];
int command;
// 0 = stop, 1 = forward motion, 2 = backward motion, 3 = turn right, 4 = turn left
// right wheels "reverse" for 180, left wheels "reverse" for 0
// right wheels "forward" for 0, left wheels "forward" for 180
// all wheels stop for 90
int servoR_comm[5] = {90,0,180,180,0}; 
int servoL_comm[5] = {90,180,0,180,0};

// Servo setup
int pinR = 4;
int pinL = 5;
Servo servoR;
Servo servoL;

void setup() {

  // FOR USE OVER USB
  // open serial connection
  // Serial.begin(115200);

  // FOR USE WITH TX(1)/RX(0) PINS (i.e., not via USB)
  // open serial connection
  Serial1.begin(115200);

  // establish Servo connections
  servoR.attach(pinR);
  servoL.attach(pinL);
}

void loop() {

  // FOR USE OVER USB
  // check if data is available, and read it if so
  //if (Serial.available() > 0) {
  //  for (int k = 0; k < 4; k++) {
  //    incomingBytes[k] = Serial.read();
  //  }
  //}

  // FOR USE WITH TX(1)/RX(0) PINS (i.e., not via USB)
  // check if data is available, and read it if so
  if (Serial1.available() > 0) {
    for (int k = 0; k < 4; k++) {
      incomingBytes[k] = Serial1.read();
    }
  }

  if (incomingBytes[0]==14 && incomingBytes[3]==22) {
    command = (incomingBytes[1] << 8 | incomingBytes[2]) & 0xFFFF;
    servoR.write(servoR_comm[command]);
    servoL.write(servoL_comm[command]);
  }

}
