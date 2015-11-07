#include <AFMotor.h>

AF_DCMotor motor1(1, MOTOR12_64KHZ);
AF_DCMotor motor2(2, MOTOR12_64KHZ);
AF_DCMotor motor3(3, MOTOR34_64KHZ);
AF_DCMotor motor4(4, MOTOR34_64KHZ);

byte targetSpeedL = 0;
byte targetSpeedR = 0;
byte targetDirL = 1;  //1 = forward, 0 = backward
byte targetDirR = 1;  //1 = forward, 0 = backward

void setup() {
  // put your setup code here, to run once:
  motor1.setSpeed(200);
  motor2.setSpeed(200);
  motor1.run(FORWARD);
  motor2.run(FORWARD);

  motor3.setSpeed(200);
  motor4.setSpeed(200);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void loop() {
  // put your main code here, to run repeatedly:

}
