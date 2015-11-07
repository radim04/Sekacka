#include <TinyDebugKnockBang.h>

// measure speed (for ATTINY84)

const int IR_SENSOR = 0; //IR sensor input pin


void setup() {
  Debug.begin( 19200 );
  pinMode(IR_SENSOR, INPUT);
}

void loop() {
  unsigned long duration;
  duration = pulseIn(IR_SENSOR, HIGH);
  Debug.println(duration, DEC);
}
