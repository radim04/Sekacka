#include <AFMotor.h>

AF_DCMotor motor1(1, MOTOR12_64KHZ);
AF_DCMotor motor2(2, MOTOR12_64KHZ);
AF_DCMotor motor3(3, MOTOR34_64KHZ);
AF_DCMotor motor4(4, MOTOR34_64KHZ);

void setup() {
  Serial.begin(9600);
  Serial.println("Bluetooth Robot Control");
  Serial.flush();
}

byte speedL = 0; //actual speed left motor
byte speedR = 0; //actual speed right motor
byte dirL = 1;  //1 = forward, 0 = backward
byte dirR = 1;  //1 = forward, 0 = backward
byte targetSpeedL = 0;
byte targetSpeedR = 0;
byte targetDirL = 1;  //1 = forward, 0 = backward
byte targetDirR = 1;  //1 = forward, 0 = backward
char get_char = ' ';  //read serial

void loop() {
  
  Serial.flush();
  
  // wait for incoming data
  if (Serial.available() >= 0) { 
    // parse incoming command 
    get_char = Serial.read();
    if (get_char == 'F') { //forward
      targetDirL = 1;
      targetDirR = 1; 
      targetSpeedL = 200;
      targetSpeedR = 200;
    } else if (get_char == 'B') { //backward
      targetDirL = 0;
      targetDirR = 0;
      targetSpeedL = 200;
      targetSpeedR = 200;
    } else if (get_char == 'L') { //left
      targetDirL = 1;
      targetDirR = 0;
      targetSpeedL = 200;
      targetSpeedR = 200;
    } else if (get_char == 'R') { //right
      targetDirL = 0;
      targetDirR = 1;
      targetSpeedL = 200;
      targetSpeedR = 200;
    } else if (get_char == 'S') { //stop
      targetDirL = 1;
      targetDirR = 1;
      targetSpeedL = 0;
      targetSpeedR = 0;
    };
  }

// adjust left motor speed
  if (dirL == targetDirL && speedL < targetSpeedL)
    speedL = speedL + 5;
  else if (dirL == targetDirL && speedL > targetSpeedL)
    speedL = speedL - 5;    
  else if (dirL != targetDirL && speedL > 0)
    speedL = speedL - 5;    
  else if (speedL == 0) {
    dirL = targetDirL;  
    //motor1.run(RELEASE);
    motor4.run(RELEASE);
    delay(10);
  }  
  //motor1.setSpeed(speedL);
  motor4.setSpeed(speedL);
  if (dirL == 1) {
    //motor1.run(FORWARD);
    motor4.run(FORWARD);
  } else {
    motor1.run(BACKWARD);
    motor4.run(BACKWARD);
  }

// adjust right motor speed
  if (dirR == targetDirR && speedR < targetSpeedR)
    speedR = speedR + 5;
  else if (dirR == targetDirR && speedR > targetSpeedR)
    speedR = speedR - 5;    
  else if (dirR != targetDirR && speedR > 0)
    speedR = speedR - 5;    
  else if (speedR == 0) {
    dirR = targetDirR;  
    //motor2.run(RELEASE);
    motor3.run(RELEASE);
    delay(10);
  }  
  //motor2.setSpeed(speedR);
  motor3.setSpeed(speedR);
  if (dirR == 1) {
    motor2.run(FORWARD);
    motor3.run(FORWARD);
  } else {
    motor2.run(BACKWARD);
    motor3.run(BACKWARD);
  }

  motor1.run(FORWARD);
  motor1.setSpeed(64);
  motor2.run(FORWARD);
  motor2.setSpeed(64);
  delay(10);

}

