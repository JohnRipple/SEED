/*
 * Joshua Lee
 * Exercise 6
 * this program sends in voltage values that is obtained through the A0 pin connected to a potentiometer and sends it to the RPI
 */
 // includes the proper Wire library
#include <Wire.h>

// defines the slave address that will be used to send and receive values from the RPI
#define SLAVE_ADDRESS 0x04
// define variables that will be used for the code
int number = 0;
int state = 0;
byte data[32];
byte temp[32];
int offset = 2;
int sensorPin = A0;
int ledPin = 13;
int sensorValue = 0;
// setup block that sets up the serial monitor, wire for the slave address, and when data is requested the sendData function is called
void setup() {
  pinMode(13,OUTPUT);
  Serial.begin(115200);
  Wire.begin(SLAVE_ADDRESS);

Wire.onRequest(sendData);

Serial.println("Ready!");
}
// loop to cause a delay between each iteration of the program/code
void loop() {
  delay(100);
}
//sendData function that sends the voltage values to the RPI
void sendData() {
  // reads in the voltage value in pin A0 and sets the it to sensorValue
  sensorValue = analogRead(sensorPin);
  // prints out the sensor value onto the serial monitor
  Serial.print(sensorValue);
  Serial.print(' ');
  // sends the sensor value to the RPI
  Wire.write(sensorValue);
}
