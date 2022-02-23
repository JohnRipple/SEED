/*
 * Joshua Lee
 * Exercise 1,2,3,5 and 7
 * this program for these exercise take in a value or array of values from the RPI and then sends back a value or array depending on what was sent from the RPI back to the RPI
 * these programs also include special cases such as different offset values changing the value sent back to the RPI or reversing order of the array sent from the RPI
 */
 // includes the proper Wire library
#include <Wire.h>
// defines the slave address that will be used to send and receive values from the RPI
#define SLAVE_ADDRESS 0x04
// define variables that will be used for the code
int number = 0;
byte data[32];
byte temp[32];
int offset = 2;
// setup block that sets up the serial monitor, wire for the slave address, and when the arduino sends and receives data from the RPI
void setup() {
  pinMode(13,OUTPUT);
  Serial.begin(115200);
  Wire.begin(SLAVE_ADDRESS);

Wire.onReceive(receiveData);
Wire.onRequest(sendData);

Serial.println("Ready!");
}
// loop to cause a delay between each iteration of the program/code
void loop() {
  delay(100);
}

void receiveData(int byteCount) {
  // this is the example code with bus.write_byte
  // while loop so that as long as values are still being sent to the arduino the code is run in the block
  /*while(Wire.available()) {
    // reads in values that is being sent from the RPI
    number = Wire.read();
    // prints out value that was received from RPI
    Serial.print("data received:");
    Serial.println(number);
    // conditionals so that if the values being sent are 1 or 0 it changes the voltage of the 13 pin and changes the state
    if (number == 1) {
      if (state == 0) {
        digitalWrite(13,HIGH);
        state = 1;
      }
      else{
        digitalWrite(13,LOW);
        state = 0;
      }
    }
  }*/
  
  //this is the code for adding 5 to what the RPI sends to arduino
  /*Serial.print("data received, adding 5 to data: ");
  // while loop so that as long as values are still being sent to the arduino the code is run in the block
  while(Wire.available()) {
    // reads in the value that is being sent from the RPI and adds 5 to it
    number = Wire.read();
    number = number + 5;
    // prints out value with 5 added onto it to the serial monitor
    Serial.print(number);
    Serial.print(' ');
  }
  Serial.println(' ');*/

  // code for offset and number from RPI to arduino changing what is sent back to RPI
  /*int i = 0;
  // while loop so that as long as values are still being sent to the arduino the code is run in the block
  while(Wire.available()) {
    // continually reads in values being sent from the RPI and placing it into the data array
    data[i] = Wire.read();
    Serial.print(data[i]);
    Serial.print(' ');
    i++;
  }
  i--;
  // prints out the size of the array and the offset
  Serial.print(i);
  Serial.print(' ');
  Serial.print(offset);
  Serial.println(' ');
  // if more than 1 value was sent to the arduino the first value is the offset and the second value is the number/value
  if (i > 0) {
    offset = data[i-1];
    number = data[i];
  }
  else {
  // if only 1 value is sent then the number is changed and the offset set to neither 1 or 0
    number = data[i];
    offset = 2;
  }
  // if the offset is 0 then add 5 to the number
  if (offset == 0) {
    number = number + 5;
  }
  // if the offset is 1 then add 10 to the number
  else if (offset == 1) {
    number = number + 10;
  }*/

  // code for arrays of integers and bytes
  int i = 0;
  // for loop to reset the values of data and temp back to 0 so that previous values don't leak into future values
  for (int j = 0; j < i;j++) {
    data[j] = 0;
    temp[j] = 0;
  }
  // while loop so that as long as values are still being sent to the arduino the code is run in the block
  while(Wire.available()) {
    // sets the values being sent from the RPI into the array
    temp[i] = Wire.read();
    // printing out the values that are being sent and placed into the array
    Serial.print(temp[i]);
    Serial.print(' ');
    i++;
  }
  // set i to the last element index
  i--;
  Serial.println(' ');
  // setting x as an index counter in reverse
  int x =i;
  // for loop to place all the values that have been sent over into the array data
  for (int j = 0; j <i;j++) {
    // reversing the order of the values in the temp array/ the values that were sent over from the RPI
    data[j] = temp[x];
    // printing out the values in reverse onto the serial monitor
    Serial.print(temp[x]);
    Serial.print(' ');
    x--;
  }
  Serial.println(' ');
  
}

void sendData() {
  // for sending back a single element/piece of data
  //Wire.write(number);
  // for sending back an array of bytes of length/size 32
  Wire.write(data,32);
}
