/*
Time for induction loop sender (hidden wire)
see http://rn-wissen.de/wiki/index.php?title=Begrenzungsschleife_-_Induktionsschleife 
*/


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 10 as an output.
  pinMode(10, OUTPUT); // ATTINY84 pin 2
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(10, HIGH);   // turn the induction loop on (HIGH is the voltage level)
  delay(1);                 // wait for 1 ms
  digitalWrite(10, LOW);    // turn the induction loop on off by making the voltage LOW
  delay(99);                // wait for 99 ms
}
