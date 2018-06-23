//Pins for the lasers
//Laser outputs are 3.3V switched by transistors
#define GRN_LASER 8
#define RED_LASER 9


void setup() {
  //Don't fire zee lasers
  pinMode(OUTPUT, RED_LASER);
  digitalWrite(RED_LASER, LOW);
  pinMode(OUTPUT, GRN_LASER);
  digitalWrite(GRN_LASER, LOW);  
}

void loop() {
  //Fire zee lasers!
  digitalWrite(RED_LASER, LOW);
  digitalWrite(GRN_LASER, HIGH);
  delay(200);
  digitalWrite(RED_LASER, HIGH);
  digitalWrite(GRN_LASER, LOW);
  delay(200);
}
