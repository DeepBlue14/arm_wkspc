#include <Servo.h>
#include <I2C.h>

//Pins for the lasers
//Laser outputs are 3.3V switched by transistors
#define LASER_1 8
#define LASER_2 9

//Servo center points, defined emperically 
//180 is far right, 0 is far left, 100 is middle
#define H_CENTER 100
#define H_MAX 180
#define H_MIN 0

//12 is minimum to avoid case corners, 50 center, 150 max to avoid breaking green laser
#define V_CENTER 50
#define V_MAX 150
#define V_MIN 13

//Horizontal and vertical servos
Servo h_servo;
Servo v_servo;

//LIDAR-Lite point lidar I2C address
char LIDARLite_ADDRESS = 0x62; 

int boundsCheck(int val, int maxVal, int minVal)
{
  if(val > maxVal)
  {
    return maxVal;
  }
  if(val < minVal)
  {
    return minVal;
  }
  return val;
}

// From PulsedLight's example code
// Write a register and wait until it responds with success
void llWriteAndWait(char myAddress, char myValue){
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.write(LIDARLite_ADDRESS, myAddress, myValue); // Write to LIDAR-Lite Address with Value
    delay(2); // Wait 2 ms to prevent overpolling
  }
}

// From PulsedLight's example code
// Read 1-2 bytes from a register and wait until it responds with sucess
byte llReadAndWait(char myAddress, int numOfBytes, byte arrayToSave[2]){
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK responses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.read(LIDARLite_ADDRESS,myAddress, numOfBytes, arrayToSave); // Read 1-2 Bytes from LIDAR-Lite Address and store in array
    delay(2); // Wait 2 ms to prevent overpolling
  }
  return arrayToSave[2]; // Return array for use in other functions
}

// Adapted from PulsedLight's example code, this configuration is only good for distance measurement
int llGetDistance(){
  llWriteAndWait(0x00,0x04); // Write 0x04 to register 0x00 to start getting distance readings
  byte myArray[2]; // array to store bytes from read function
  llReadAndWait(0x8f,2,myArray); // Read 2 bytes from 0x8f
  int distance = (myArray[0] << 8) + myArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
  return(distance);
}


void circleLaser()
{
  int originX = h_servo.read();
  int originY = v_servo.read();
  int ii = 0;
  int delta = 3;
  //Technically, this is a square, but the mechanincs are pretty sloppy
  for(ii = 0; ii < 4; ii++)
  {
    h_servo.write(boundsCheck(originX + delta, H_MAX, H_MIN));
    v_servo.write(boundsCheck(originY + delta, V_MAX, V_MIN));
    delay(30);
    h_servo.write(boundsCheck(originX + delta, H_MAX, H_MIN));
    v_servo.write(boundsCheck(originY - delta, V_MAX, V_MIN));
    delay(30);
    h_servo.write(boundsCheck(originX - delta, H_MAX, H_MIN));
    v_servo.write(boundsCheck(originY - delta, V_MAX, V_MIN));
    delay(30);
    h_servo.write(boundsCheck(originX - delta, H_MAX, H_MIN));
    v_servo.write(boundsCheck(originY + delta, V_MAX, V_MIN));
    delay(300);    
  }
}

void setup() {
  //Don't fire zee lasers
  pinMode(OUTPUT, LASER_1);
  digitalWrite(LASER_1, LOW);
  pinMode(OUTPUT, LASER_2);
  digitalWrite(LASER_2, LOW);  
  
  //Attach servos to the output pins
  h_servo.attach(10);
  v_servo.attach(11);

  //center the servos
  h_servo.write(H_CENTER);
  v_servo.write(V_CENTER);
  
  //Start up the serial link
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  
  //Start talking to the LIDARLite
  I2c.begin(); 
  delay(100); // Let everything wake up
  I2c.timeOut(50);
  llWriteAndWait(0x00,0x00); // reset device to defaults for distance measurment
}

void loop() {
  if(Serial.available() > 0)
  {
    String input = Serial.readStringUntil('\n');
    if(input == "?"){ //Initialization from ROS
      Serial.println("ready");
    }
    else{ //Parse a string of the form x<number>y<number>[l][m][r]
      //Serial.println(input);
      int splitPoint = input.indexOf("y");
      int buttons = min(input.indexOf("l"), min(input.indexOf("m"), input.indexOf("r")));
      int xCoord, yCoord = 0;
      xCoord = input.substring(1, splitPoint).toInt();
      if(buttons < 0)// no buttons
      {
        yCoord = input.substring(splitPoint+1).toInt();
      }else{
        yCoord = input.substring(splitPoint+1, buttons).toInt();
      }
      //Move the servos into place
      h_servo.write(map(xCoord, 100, -100, H_MAX, H_MIN));
      v_servo.write(map(yCoord, -100, 100, V_MAX, V_MIN));
      
      if(input.indexOf("l") > 0)
      {
        //Left mouse button pressed
        circleLaser();
      }
    }
  }
  
  //Get the distance (16-bit int)
  int dist = llGetDistance();
  
  //Change the laser color
  if(dist > 150)
  {
    digitalWrite(LASER_1, HIGH);
    digitalWrite(LASER_2, LOW);
  }
  else
  {
    digitalWrite(LASER_1, LOW);
    digitalWrite(LASER_2, HIGH);
  }
}
