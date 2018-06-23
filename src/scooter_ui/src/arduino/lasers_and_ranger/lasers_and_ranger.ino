#include <I2C.h>

//Pins for the lasers
//Laser outputs are 3.3V switched by transistors
//You may have to change these to account for hardware differences
//9 = red and 10 = green for the box on loc-line (arduino micro)
//8 = red and 9 = green for servo system (arduino leonardo)
#define LASER_1 9
#define LASER_2 10

//For blinking without delay-waits
unsigned long lastChange;
unsigned long currentTime;
bool isLaserOn;

//LIDAR-Lite point lidar I2C address
char LIDARLite_ADDRESS = 0x62;

// From PulsedLight's example code
// Write a register and wait until it responds with success
void llWriteAndWait(char myAddress, char myValue) {
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses
  while (nackack != 0) { // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.write(LIDARLite_ADDRESS, myAddress, myValue); // Write to LIDAR-Lite Address with Value
    delay(2); // Wait 2 ms to prevent overpolling
  }
}

// From PulsedLight's example code
// Read 1-2 bytes from a register and wait until it responds with sucess
byte llReadAndWait(char myAddress, int numOfBytes, byte arrayToSave[2]) {
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK responses
  while (nackack != 0) { // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.read(LIDARLite_ADDRESS, myAddress, numOfBytes, arrayToSave); // Read 1-2 Bytes from LIDAR-Lite Address and store in array
    delay(2); // Wait 2 ms to prevent overpolling
  }
  return arrayToSave[2]; // Return array for use in other functions
}

// Adapted from PulsedLight's example code, this configuration is only good for distance measurement
int llGetDistance() {
  llWriteAndWait(0x00, 0x04); // Write 0x04 to register 0x00 to start getting distance readings
  byte myArray[2]; // array to store bytes from read function
  llReadAndWait(0x8f, 2, myArray); // Read 2 bytes from 0x8f
  int distance = (myArray[0] << 8) + myArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
  return (distance);
}

void setup() {
  //Don't fire zee lasers
  pinMode(OUTPUT, LASER_1);
  digitalWrite(LASER_1, LOW);
  pinMode(OUTPUT, LASER_2);
  digitalWrite(LASER_2, LOW);

  //Start up the serial link
  Serial.begin(57600);
  //while (!Serial) {
  //  ; // wait for serial port to connect. Needed for Leonardo only
  //}

  //Start talking to the LIDARLite
  I2c.begin();
  delay(100); // Let everything wake up
  I2c.timeOut(50);
  llWriteAndWait(0x00, 0x00); // reset device to defaults for distance measurment
  
  //Set up to blink laser
  isLaserOn = true;
  lastChange = millis();
}

void loop() {

  //Get the distance (16-bit int)
  int dist = llGetDistance();

  Serial.println(dist);

  //Blink the laser
  currentTime = millis();
  if (currentTime - lastChange > 33) //30 fps = 0.03 spf = 33.333... mspf
  {
    isLaserOn = !(isLaserOn); //Toggle the laser state
    lastChange = currentTime; //Update the last time this was changed
  }

  if(isLaserOn)
  {
    //Change the laser color if needed
    if (dist > 200)
    {
      digitalWrite(LASER_1, HIGH);
      digitalWrite(LASER_2, LOW);
    } else {
      digitalWrite(LASER_1, LOW);
      digitalWrite(LASER_2, HIGH);
    }
  }
  else
  {
    //Both lasers are off
    digitalWrite(LASER_1, LOW);
    digitalWrite(LASER_2, LOW);
  }
}