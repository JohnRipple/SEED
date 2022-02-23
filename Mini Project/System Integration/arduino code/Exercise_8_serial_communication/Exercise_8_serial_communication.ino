/*
 * Joshua Lee
 * Exercise 8
 * this program reads in a value from the RPI and adds 5 to it and sends it back through serial communication
 */
// define variables that will be used for the program
String data;
int add;
bool DataRead;
// sets up the serial address/monitor at a 115200 baud rate
void setup() {
  Serial.begin(115200);
}
// continually loops through this block to see if the arduino is supposed to read something from the RPI
void loop() {
  //if there is data that has been read it runs through this block of code
  if (DataRead) {
    // prints out what the data that was sent from the RPI(this is sent back to the RPI)
    Serial.print("You sent me: ");
    Serial.println(data);
    // prints out the data that has 5 added to it (this is sent back to the RPI)
    Serial.print("I will send you back: ");
    // this changes the value of data to an integer to properly add 5 to it and then changes it back to a string to be sent to the RPI
    add = data.toInt() + 5;
    data = String(add);
    Serial.println(data);
    // sets Dataread to false so it doesn't continually go through the if statement infinitely
    DataRead = false;
  }
}
// function for if something is sent from the RPI to the serial monitor/address
void serialEvent(){
  // if something is sent from the RPI to the serial monitor/address then set the variable data to the data/values that was sent from the RPI
  if (Serial.available() > 0) {
    data = Serial.readStringUntil("\n");
    // set DataRead to true so that the loop will sent values/data back to the RPI
    DataRead = true;
  }
  // flushes the seral monitor/address so that previous values don't leak into future values
  Serial.flush();
}
