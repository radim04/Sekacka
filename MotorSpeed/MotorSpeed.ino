#include <TinyDebugKnockBang.h>

// measure speed (for ATTINY84)

const int IR_SENSOR = 8; //IR sensor input pin (must be INT0)
volatile byte revolutions;
unsigned int rpm;
unsigned long timeold;

void setup() {
  Debug.begin( 19200 );
  attachInterrupt(0, rpmInterrupt, RISING);
  pinMode(IR_SENSOR, INPUT);
  revolutions = 0;
  rpm = 0;
  timeold = 0;
}

void loop() {
   if (revolutions >= 20) { 
     //update RPM every 20 counts, increase this for better RPM resolution,
     //decrease for faster update
     //rpm = 60*1000/(millis() - timeold)*revolutions;
     rpm = millis() - timeold;
     timeold = millis();
     revolutions = 0;
     Debug.println(rpm, DEC);
   }
}

 void rpmInterrupt()
 {
   //each rotation, this interrupt function is run
   revolutions++;
 }
