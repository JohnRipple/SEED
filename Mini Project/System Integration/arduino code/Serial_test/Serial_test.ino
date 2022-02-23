                                                      String data;
bool DataRead;

void setup() {
  Serial.begin(115200);
}
void loop() {
  if (DataRead) {
    Serial.print("You sent me: ");
    Serial.println(data);
    DataRead = false;
  }
}

void serialEvent(){
  if (Serial.available() > 0) {
    data = Serial.readStringUntil("\n");
    DataRead = true;
  }
  Serial.flush();
}
