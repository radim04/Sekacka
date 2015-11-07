

int val = 0;     // Variable um den gelesenen Wert zu speichern
int ledPin = 13; // LED verbunden mit dem digitalen Pin 13
int inPin = 2;   // Taster verbunden mit dem digitalen Pin 7

void setup(){
  //start serial connection
  //Serial.begin(9600);
  //configure pin2 as an input and enable the internal pull-up resistor
  pinMode(inPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT); 
}

void loop() {
  val = digitalRead(inPin);   // liest den Eingangs-Pin
  digitalWrite(ledPin, val);  // schaltet die LED auf den Wert, den der Taster vorgibt
}
