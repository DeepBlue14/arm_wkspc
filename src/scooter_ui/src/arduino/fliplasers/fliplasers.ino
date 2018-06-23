//Pins for the lasers
//Laser outputs are 3.3V switched by transistors
#define LASER_1 10
#define LASER_2 9

void setup() {
  // put your setup code here, to run once:
  //Don't fire zee lasers
  pinMode(OUTPUT, LASER_1);
  digitalWrite(LASER_1, LOW);
//  pinMode(OUTPUT, LASER_2);
//  digitalWrite(LASER_2, LOW);  
  
  //digitalWrite(LASER_2, HIGH);
  //digitalWrite(LASER_1, HIGH);
}

void loop() {
    digitalWrite(LASER_1, HIGH);
    //digitalWrite(LASER_2, LOW);
    delay(100);
    digitalWrite(LASER_1, LOW);
    //digitalWrite(LASER_2, HIGH);
    delay(100);
}
