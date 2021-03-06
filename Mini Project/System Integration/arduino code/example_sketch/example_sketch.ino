#include <Wire.h>

#define SLAVE_ADDRESS 0x04
int number = 0;
int state = 0;

void setup() {
  pinMode(13,OUTPUT);
  Serial.begin(115200);
  Wire.begin(SLAVE_ADDRESS);

Wire.onReceive(receiveData);
Wire.onRequest(sendData);

Serial.println("Ready!");
}

void loop() {
  delay(100);
}

void receiveData(int byteCount) {
  // this is the example code with bus.write_byte
  while(Wire.available()) {
    number = Wire.read();
    Serial.print("data received:");
    Serial.println(number);

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
  }

  //this is the code for bus.write_byte_data
  /*Serial.print("data received: ");
  while(Wire.available()) {
    number = Wire.read();
    Serial.print(number);
    Serial.print(' ');
  }
  Serial.println(' ');*/
}

void sendData() {
  Wire.write(number);
}
