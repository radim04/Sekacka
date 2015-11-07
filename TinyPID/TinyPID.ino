#include <PID_v1.h>
#include <TinyDebugKnockBang.h>
//(for ATTINY84)

const int IR_SENSOR = 8; //IR sensor input pin 
const int MOTOR_PWM = 0; //PWM motor output
const double MAX_INPUT = 1000; //PID input if motor doesn't move

//Define Variables we'll be connecting to
double setpoint, input, output;

//Specify the links and initial tuning parameters
PID myPID(&input, &output, &setpoint,2,5,1, DIRECT);

void setup()
{
  //initialize the variables we're linked to
  input = getInput();
  setpoint = 900;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
}

void loop()
{
  input = getInput();
  myPID.Compute();
  analogWrite(MOTOR_PWM,output);
  //Debug.print(input, DEC); Debug.print(" | "); Debug.println(output, DEC);
}

double getInput() {
  unsigned long duration = pulseIn(IR_SENSOR, HIGH);
  //Debug.print("Duration:"); Debug.println(duration, DEC);
  if(duration == 0) {
    return 0;
  } else {
  double input = 1000 - duration;
  input = constrain(input, 0, 1000);
  return input;
  }
}

