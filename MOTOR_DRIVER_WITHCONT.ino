int M1Speed = 9;
int M2Speed = 10;
int M1Dir = 7;
int M2Dir = 8;
int n = 1;

//PINS 9 and 10 are MOTOR VOLTAGE

void setup() {
  Serial.begin(19200);
  pinMode(4, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(12, INPUT);

}

void loop() {
if( millis() < 10){
  analogWrite(M1Speed, 0);
}
// PWM frequency is 500Hz, so it's about 2 ms per duty cycle
// Of the actual arduino, FREQ is 490.35, 2.0394 ms duty cycle
// Pins are delivering voltages
// PWM determines how much power is delivered to the motor which in turn can be used to control speed.
// IE, 50% duty cycle would be 50% power  
if ( millis() < 2500){
    if((n*10+1) < millis() && (n*10-1) < millis() ){
    Serial.println(digitalRead(M1Speed));
    Serial.println(digitalRead(M1Dir));
    Serial.println(millis());
    //Serial.println(ANGULARR_VELOCITY);
    }
// CURRENTLY SET AT A 5-6 MS SAMPLING RATE.





if(1000 < millis() && millis() < 1999){
  //DIRECTION
  digitalWrite(M1Dir, HIGH);
  // SPEED FULL POWER
  analogWrite(M1Speed, 255);
    }

    //THIS FLIPS THE DIRECTION
else if(2000 < millis()){
  //DIRECTION
  digitalWrite(M1Dir, LOW);
  // SPEED FULL POWER
  analogWrite(M1Speed, 0);
    }
   
  n++;
}
}
