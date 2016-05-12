#include <Adafruit_MotorShield.h>
#include <Wire.h>

/*
 Reads radio control input, mixes X and Y to MotorL and MotorR and controls speed of mowing motor
 */

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *lMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rMotor = AFMS.getMotor(2);

const int RADIO_INPUT_X = A0; //radio control input X
const int RADIO_INPUT_Y = A1; //radio control input Y
const int RADIO_INPUT_MOW = A2; //radio control mowing motors speed
const int PULSE_MIN_X = 1141; //lowest radio control pulse width Y
const int PULSE_MAX_X = 1834; //highest radio control pulse width Y
const int PULSE_MIN_Y = 1124; //lowest radio control pulse width Y
const int PULSE_MAX_Y = 1751; //highest radio control pulse width Y
const int PULSE_MIN_MOW = 1152; //lowest radio control pulse width mowing speed
const int PULSE_MAX_MOW = 1780; //highest radio control pulse width mowing speed
const int STOP_LIMIT = 40;    //interval in which motors shall not move

// Motor 1 (wheel)
//const int DIR1PINA = 2;
//const int DIR2PINA = 3;
//const int SPEEDPINA = 9; // Needs to be a PWM pin to be able to control motor speed

// Motor 2 (wheel)
//const int DIR1PINB = 4;
//const int DIR2PINB = 5;
//const int SPEEDPINB = 10; // Needs to be a PWM pin to be able to control motor speed

// Mowing Motor
//const int IR_SENSOR = 12; //IR sensor input pin
const int MOW_MOTOR_PWM = 6; //PWM motor output
//const int MAX_INPUT = 1023; //PID input if mowing motor doesn't move
//byte mowingMotorPWM = 255;
//double targetMowingMotorSpeed = 500;
//double inputPID = 0;
//double lastInputPID = 0;
//double iTermPID = 0;
//double outputPID;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second
  Serial.begin(9600);
  // make the radio input pins input
  pinMode(RADIO_INPUT_X, INPUT);
  pinMode(RADIO_INPUT_Y, INPUT);
  AFMS.begin();
  // IR speed sensor for mowing motor
  //pinMode(IR_SENSOR, INPUT);
  // motors
  //pinMode(DIR1PINA, OUTPUT);
  //pinMode(DIR2PINA, OUTPUT);
  //pinMode(SPEEDPINA, OUTPUT);
  //pinMode(DIR1PINB, OUTPUT);
  //pinMode(DIR2PINB, OUTPUT);
  //pinMode(SPEEDPINB, OUTPUT);
  //pinMode(MOW_MOTOR_PWM, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  // get radio input
  int radioX = 0;
  int radioY = 0;
  int rc = 0;
  int lMotorPWM = 0;
  int rMotorPWM = 0;
  int mowSpeed = 0;
  rc = getRadioInput(radioX, radioY, mowSpeed);
  if (rc == 0) {
    // print out the pulse length
    //Serial.print("X:"); Serial.print(radioX, DEC);
    //Serial.print("  Y:"); Serial.print(radioY, DEC); Serial.println("|");
    Serial.print("  Mow:"); Serial.print(mowSpeed, DEC); Serial.println("|");
    mix(radioX, radioY, lMotorPWM, rMotorPWM);
    //Serial.print("L:"); Serial.print(lMotorPWM, DEC);
    //Serial.print("  R:"); Serial.print(rMotorPWM, DEC); Serial.println("|");
    setMotorSpeed(lMotorPWM, rMotorPWM);
    analogWrite(MOW_MOTOR_PWM, mowSpeed);
    // mowing motor control
    //int mowingMotorSpeed = getMowingMotorSpeed();
    //Serial.print("Duration:");
    //Serial.print(mowingMotorSpeed, DEC); Serial.print(" ");
    //mowingMotorPWM = constrain(mowingMotorPWM + 0.02*(targetMowingMotorSpeed - mowingMotorSpeed), 0, 255);
    //inputPID = mowingMotorSpeed;
    //computePID(&inputPID, &outputPID, &targetMowingMotorSpeed, 0.2, 0.5, 0.1, 0, 255, &lastInputPID, &iTermPID);
    //mowingMotorPWM = constrain(outputPID, 0, 255);
    //Serial.println(mowingMotorPWM, DEC);
    //analogWrite(MOW_MOTOR_PWM, 0);
    //analogWrite(MOW_MOTOR_PWM, mowingMotorPWM);
  } else {
    // no signal -> stop!
    stopMotors();
  }
  //delay(700);
}
/*
void computePID(double* Input, double* Output, double* Setpoint,
                double Kp, double Ki, double Kd, byte OutMin, byte OutMax, 
                double* LastInput, double* ITerm) {
  //compute all error variables
  double input = *Input;
  double error = *Setpoint - input;
  *ITerm += (Ki * error);
  if (*ITerm > OutMax) *ITerm = OutMax;
  else if (*ITerm < OutMin) *ITerm = OutMin;
  double dInput = (input - *LastInput);
  //compute PID Output
  double output = Kp * error + *ITerm - Kd * dInput;
  if (output > OutMax) output = OutMax;
  else if (output < OutMin) output = OutMin;
  *Output = output;
  //remember for next time
  *LastInput = input;
}
*/
int getRadioInput(int &X, int &Y, int &mowSpeed) {
  unsigned long duration;
  duration = pulseIn(RADIO_INPUT_X, HIGH);
  //Serial.print("DurationX:"); Serial.print(duration, DEC);
  if (duration == 0) {
    return -1; // radio off -> no pulse
  } else {
    duration = constrain(duration, PULSE_MIN_X, PULSE_MAX_X);
    X = map(duration, PULSE_MIN_X, PULSE_MAX_X, -512, 511);
    duration = pulseIn(RADIO_INPUT_Y, HIGH);
    //Serial.print("  DurationY:"); Serial.println(duration, DEC);
    duration = constrain(duration, PULSE_MIN_Y, PULSE_MAX_Y);
    Y = map(duration, PULSE_MIN_Y, PULSE_MAX_Y, -512, 511);
    duration = pulseIn(RADIO_INPUT_MOW, HIGH);
    Serial.print("  MowDuration:"); Serial.println(duration, DEC);
    duration = constrain(duration, PULSE_MIN_MOW, PULSE_MAX_MOW);
    mowSpeed = map(duration, PULSE_MIN_MOW, PULSE_MAX_MOW, 0, 255);
  }
  return 0;
}

void mix(int X, int Y, int &lMotor, int &rMotor) {
  //mix throttle and direction
  int leftMotor = Y + X;
  int rightMotor = Y - X;
  //calculate the scale of the results
  float leftMotorScale =  leftMotor / 255.0;
  leftMotorScale = abs(leftMotorScale);
  float rightMotorScale =  rightMotor / 255.0;
  rightMotorScale = abs(rightMotorScale);
  //choose the max scale value if it is above 1
  float maxMotorScale = max(leftMotorScale, rightMotorScale);
  maxMotorScale = max(1, maxMotorScale);
  //and apply it to the mixed values
  lMotor = constrain(leftMotor / maxMotorScale, -255, 255);
  rMotor = constrain(rightMotor / maxMotorScale, -255, 255);
  //Serial.print("lMotor: "); Serial.println(lMotor, DEC);
  //Serial.print("rMotor: "); Serial.println(rMotor, DEC);
}

void setMotorSpeed(int lMotorPWM, int rMotorPWM) {
  if (abs(lMotorPWM) < STOP_LIMIT) {
    lMotor->setSpeed(0);
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

void stopMotors() {
  lMotor->setSpeed(0);
  rMotor->setSpeed(0);
  analogWrite(MOW_MOTOR_PWM, 0);
}


