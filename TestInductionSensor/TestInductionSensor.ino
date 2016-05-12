
#include <Adafruit_MotorShield.h>
#include <Wire.h>

#define inductionSensorPinLeft 2
#define inductionSensorPinRight 3
#define ledPin 13
const int STOP_LIMIT = 40;    //interval in which motors shall not move

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *lMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rMotor = AFMS.getMotor(2);

int lSpeed = 0;     
int rSpeed = 0;     

void setup() {
  AFMS.begin(300);
  pinMode(ledPin, OUTPUT);
  pinMode(inductionSensorPinLeft, INPUT);
  pinMode(inductionSensorPinRight, INPUT);
}

void loop() {
  if (digitalRead(inductionSensorPinLeft) && digitalRead(inductionSensorPinRight)) {
     setMotorSpeed(127, 127);
     delay(100);
  } else {
     setMotorSpeed(-127, -127);
     delay(2000);
     setMotorSpeed(-127, 127);
     delay(1000);     
  }
}

void setMotorSpeed(int lMotorPWM, int rMotorPWM) {
  if (abs(lMotorPWM) < STOP_LIMIT) {                        //TODO
    while(lSpeed > 0) {
      lMotor->setSpeed(lSpeed);
  } else {
    lMotor->setSpeed(abs(lMotorPWM));
    if (lMotorPWM > 0) {
      lMotor->run(FORWARD);
    } else {
      lMotor->run(BACKWARD);
    }
  }
  if (abs(rMotorPWM) < STOP_LIMIT) {
    rMotor->setSpeed(0);
  } else {
    rMotor->setSpeed(abs(rMotorPWM));
    if (rMotorPWM > 0) {
      rMotor->run(FORWARD);
    } else {
      rMotor->run(BACKWARD);
    }
  }
}
