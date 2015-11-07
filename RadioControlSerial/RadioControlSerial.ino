/*
 Reads radio control input on pin 1, prints the result to the serial monitor
 */

int RADIO_INPUT = A0;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // make the radio input pin an input:
  pinMode(RADIO_INPUT, INPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input pin
  unsigned long duration;
  duration = pulseIn(RADIO_INPUT, HIGH);
  // print out the pulse length
  Serial.println(duration);
}



