
/* Authors: Andrew Samson, Michael Klima, Joshua Lee, John Ripple 2/28/2022
 *  
 * Description: This code is used to move the position of a wheel to a specified quadrant 
 * based on a signal from a Rasberry pi.
 * 
 * Hardware: 
 * 
 * connect the clk on the encoder to pins 3 (interrupt pins)
 * connect the dt on the encoder to pin 5
 * connect GND on the encoder to GND on the arduino
 * connect + on encoder to the 5V on the arduino
 * connect pin 
 * 
 */
 #include <Wire.h>
 #include <Encoder.h>
 #define SLAVE_ADDRESS 0x04
 Encoder myEnc(3,5);


int N_in_rotation = 3200; //Clicks in an encoder for full circle
int rotation_made = 0; //Rotations the motor has made since the program began


int M1Speed = 9; //sets some pins
int M2Speed = 10;
int M1Dir = 7;
int M2Dir = 8;
int DIRECTIONM1 = HIGH;
int DIRECTIONM2 = HIGH;
bool STRAIGHT = false;

int ONCE = 0;
double voltage = 0;
double presentVoltage = 8.0; //WHYYYYYYYYYYYYYYYYYYYYYYY
int PWM_value = 0;
double error = 0;
double errorDis = 0;
double curDis = 0.0;
//IN FEET OR WHATEVER THE UNITS NEED TO BE
double RADIUS = 0.05; //IDK WHAT UNITS THIS IS SUPPOSED TO BE IN
double circum = PI*RADIUS*2;
double perClick = circum /3200;

//BELOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOWWWWW
double desired_angle = 0.0; // ENTERED IN DEGREES
double desDis = 0.0; //ENTERED IN FEET
//ABOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVEEEEEEE

//Controller Specifications
//K was initially like 4.1 

// It'll read the desired direction as the input, then it will output speed to reach the desired location
float Kp = 0.289202862842511; //in Volts/Radian
float Ki = 10.8369142120677; //in volts /radian
double I_past = 0;
double I = 0;

struct wheel {
  double theta = 0;  
};

wheel left;

void setup() {
  Serial.begin(115200);
  
  //Initialize the left encoder/wheel
  

  pinMode(4, OUTPUT); //Ensures that the motor will move
  pinMode(M1Dir, OUTPUT); //Sets the Motor 1 direction as an output
  pinMode(M2Dir, OUTPUT); //Sets the Motor 2 direction as an output
  pinMode(M1Speed, OUTPUT); //Sets the Motor 1 speed as an output
  pinMode(M2Speed, OUTPUT); //Sets the Motor 2 speed as an output
  pinMode(12, INPUT); 
  pinMode(13, OUTPUT); //I2C connection 
  digitalWrite(4, HIGH); //Sets pin 4 to high so the motor works
  Wire.begin(SLAVE_ADDRESS); //Sets the 
  Wire.onReceive(receiveData);
  Serial.println("Ready!"); 

}

void loop() {

    desired_angle = PI/180 * desired_angle;
    
    left.theta = myEnc.read()*2*PI /N_in_rotation; //Calculates the theta based on how many clicks have occured in the encoder

    curDis = (left.theta / (2*PI)) * circum; // distance in feet that the encoder wheel has moved
    
    // ALL IN RADS
    // ALL IN RADS
    // ALL IN RADS
    // ALL IN RADS
    // ALL IN RADS


//SAMPLES EVERY 10 ms

    if(ONCE == 0){
      desDis = desDis + curDis;
    }
    ONCE++;
    
    if ( millis() % 10 == 0){
      
      error = (desired_angle - left.theta); // already in radians
      errorDis = (desDis - curDis);
    
      if(error > 0 && STRAIGHT == false){
        DIRECTIONM1 = HIGH; // CW
        DIRECTIONM2 = LOW;
      } else if(error <= 0 && STRAIGHT == false){
        DIRECTIONM1 = LOW; //Sets direction of motor to CCW
        DIRECTIONM2 = HIGH;
      }

      if(errorDis > 0 && STRAIGHT == true){
        DIRECTIONM1 = HIGH; // CW
        DIRECTIONM2 = HIGH;
      } else if (errorDis <= 0 && STRAIGHT == true){
        DIRECTIONM1 = LOW; //Sets direction of motor to CCW
        DIRECTIONM2 = LOW;
      }

      // Ki part
      I = I_past+((error)*0.008);    
      I_past = I;
  
      if(STRAIGHT == false){
        voltage = (Kp * (error)) + (Ki * I);  
      } else{
        voltage = (Kp * (errorDis));    
      }
      PWM_value = ((voltage/presentVoltage))*255;

      if (PWM_value > 255 || PWM_value < -255) {
        PWM_value = 255;
      }
      else{ 
        PWM_value = abs(PWM_value); 
      }


      digitalWrite(M1Dir, DIRECTIONM1);
      analogWrite(M1Speed, PWM_value);
      digitalWrite(M2Dir, DIRECTIONM2);
      analogWrite(M2Speed, PWM_value);
     
      if(error == 0){
        STRAIGHT = true;
        ONCE = 0;  
      }
  
}

  // IDK WHAT THIS IS FORRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR
      if(Serial.available() > 0){ //If there is a '0' inputed to serial input then the current angle of the motor is set as 0 radians.
        if(Serial.read() == 48)
          myEnc.write(0);
        
      }
}



//Prints a bunch of variables separated by a tab '\t'
void tab_sep_print(double arr[], int precision[], int arrSize){ 
  for(int i = 0; i < arrSize; i++){
    Serial.print(arr[i],precision[i]);
    Serial.print('\t');
  }
  Serial.print('\n');
  
}

void receiveData(int byteCount) {
      desired_angle = Wire.read() - 1; //Sets the quadrant to the input from the pi. The -1 converts the signal to the desired quadrant
}
