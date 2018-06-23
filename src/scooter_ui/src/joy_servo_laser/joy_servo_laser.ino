//for talking to the lidarlite
#include <I2C.h>

//This #define tells Leonardo Arduinos to do ROS over the USB/Serial converter
#define USE_USBCON 
#include <ros.h>
#include <ArduinoHardware.h>
#include <sensor_msgs/Joy.h>
#include <Servo.h>

//Pins for the lasers
//Laser outputs are 3.3V switched by transistors
#define LASER_1 8
#define LASER_2 9

#define H_CENTER 92
#define V_CENTER 75

//Horizontal and vertical servos
Servo h_servo;
Servo v_servo;

//#define DEBUG

char LIDARLite_ADDRESS = 0x62; 

ros::NodeHandle nh;

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

void messageCb(const sensor_msgs::Joy& posMsg){
  //for debugging reasons, blink the LED when a message is received
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  
  //Convert joystick position to offsets from centered
  float xOffset = posMsg.axes[0] * 8;
  float yOffset = posMsg.axes[1] * 8;
  
  h_servo.write(H_CENTER + xOffset);
  v_servo.write(V_CENTER + yOffset);

}
//Subscribed to joystick messages
ros::Subscriber<sensor_msgs::Joy> sub("joy", &messageCb );


void setup()
{
  //Set up the debug LED
  pinMode(13, OUTPUT);
  
  //Don't fire zee lasers
  pinMode(OUTPUT, LASER_1);
  digitalWrite(LASER_1, LOW);
  pinMode(OUTPUT, LASER_2);
  digitalWrite(LASER_2, LOW);  
  
  //Attach servos to the output pins
  h_servo.attach(10);
  v_servo.attach(11);
  
  //center servos
  h_servo.write(H_CENTER);
  v_servo.write(V_CENTER);
  
  //Start the ROS components
  nh.initNode();
  nh.subscribe(sub);
  
  //Start talking to the LIDARLite
  I2c.begin(); 
  delay(100); // Let everything wake up
  I2c.timeOut(50);
  llWriteAndWait(0x00,0x00); // reset device to defaults for distance measurment

}

void loop()
{
  //Get the distance (16-bit int)
  int dist = llGetDistance();
  if(dist > 200)
  {
    digitalWrite(LASER_1, HIGH);
    digitalWrite(LASER_2, LOW);
  }else{
    digitalWrite(LASER_1, LOW);
    digitalWrite(LASER_2, HIGH);
  }

  //Check for messages
  nh.spinOnce();
  delay(1);
}

