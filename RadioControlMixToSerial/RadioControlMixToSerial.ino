/*
 Reads radio control input, mixes X and Y to MotorL and MotorL, 
 prints the result to the serial monitor
 */

const int RADIO_INPUT_X = A1; //radio control input X
const int RADIO_INPUT_Y = A0; //radio control input Y
const int PULSE_MIN_X = 1141; //lowest radio control pulse with Y
const int PULSE_MAX_X = 1834; //highest radio control pulse with Y
const int PULSE_MIN_Y = 1124; //lowest radio control pulse with Y
const int PULSE_MAX_Y = 1751; //highest radio control pulse with Y

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second
  Serial.begin(9600);
  // make the radio input pin an input
  pinMode(RADIO_INPUT_X, INPUT);
  pinMode(RADIO_INPUT_Y, INPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  // get radio input
  int radioX = 0;
  int radioY = 0; 
  int rc = 0;
  int lMotor = 0;
  int rMotor = 0;
  rc = getRadioInput(radioX, radioY);
  if(rc == 0) {
    // print out the pulse length
    //Serial.print("X:"); Serial.print(radioX, DEC);
    //Serial.print("  Y:"); Serial.print(radioY, DEC); Serial.println("|");
    mix(radioX, radioY, lMotor, rMotor);
    Serial.print("L:"); Serial.print(lMotor, DEC);
    Serial.print("  R:"); Serial.print(rMotor, DEC); Serial.println("|");
  }
  delay(1000);
}

int getRadioInput(int &X, int &Y) {
  unsigned long duration;
  duration = pulseIn(RADIO_INPUT_X, HIGH);
  //Serial.print("DurationX:"); Serial.print(duration, DEC);
  if(duration == 0) {
    return -1; // radio off -> no pulse
  } else {
    duration = constrain(duration, PULSE_MIN_X, PULSE_MAX_X);
    X = map(duration, PULSE_MIN_X, PULSE_MAX_X, -512, 511);
    duration = pulseIn(RADIO_INPUT_Y, HIGH);
    //Serial.print("  DurationY:"); Serial.println(duration, DEC);
    duration = constrain(duration, PULSE_MIN_Y, PULSE_MAX_Y);
    Y = map(duration, PULSE_MIN_Y, PULSE_MAX_Y, -512, 511);
  }
  return 0;
}

void mix(int X, int Y, int &lMotor, int &rMotor) {
  //mix throttle and direction
  int leftMotor = Y+X;
  int rightMotor = Y-X;
  //calculate the scale of the results 
  float leftMotorScale =  leftMotor/255.0;
  leftMotorScale = abs(leftMotorScale);
  float rightMotorScale =  rightMotor/255.0;
  rightMotorScale = abs(rightMotorScale);
  //choose the max scale value if it is above 1
  float maxMotorScale = max(leftMotorScale,rightMotorScale);
  maxMotorScale = max(1,maxMotorScale);
  //and apply it to the mixed values
  lMotor = constrain(leftMotor/maxMotorScale,-255,255);
  rMotor = constrain(rightMotor/maxMotorScale,-255,255);
}

