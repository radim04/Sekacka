#include <PID_v1.h>
#include <TinyDebugKnockBang.h>
//(for ATTINY84)

const int IR_SENSOR = 8; //IR sensor input pin 
const int MOTOR_PWM = 7; //PWM motor output
const double MAX_INPUT = 1023; //PID input if motor doesn't move

//Define Variables we'll be connecting to
double setpoint, input, output, priorInput;

//Specify the links and initial tuning parameters
PID myPID(&input, &output, &setpoint,1,3,1, DIRECT);

void setup()
{
  //initialize the variables we're linked to
  input = getInput();
  setpoint = 300;
  output = 0;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(10); // compute every 30 millis
}

void loop()
{
  input = getInput();
  //myPID.Compute();
  output = output + 0.05*(setpoint - input);
  output = constrain(output, 0, 255);
  //priorInput = input;
  analogWrite(MOTOR_PWM,output);
  Debug.print(input, DEC); Debug.print(" | "); Debug.println(output, DEC);
  delay(10);
}

double getInput() {
  unsigned long duration = pulseIn(IR_SENSOR, HIGH);
  //Debug.print("Duration:"); Debug.println(duration, DEC);
  if(duration == 0) {
    return 0;
  } else {
  double input = MAX_INPUT - duration;
  input = constrain(input, 0, MAX_INPUT);
  return input;
  }
}

