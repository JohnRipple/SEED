
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
double radius = 0.05; //Wheel radius
int rotation_made = 0; //Rotations the motor has made since the program began


int M1Speed = 9; //sets some pins
int M2Speed = 10;
int M1Dir = 7;
int M2Dir = 8;
int n = 1;
int quadrant = 0;
int DIRECTION = HIGH;
int SPEED = 180;


//Controller Specifications
//K was initially like 4.1 

// It'll read the desired direction as the input, then it will output speed to reach the desired location
float Kp = 0.289202862842511; //in Volts/Radian
float Ki = 10.8369142120677; //in volts /radian
float e_past = 0;
double Tp = 0; //time in ms
double Tc =0; //time in ms
double Radp = 0; //radians past theta
double VELOCITY = 0; //m/s

//OLD WHEEL PRECISION WAS 0.11 and then changed to 0.11/1.2 and then back to 0.08
// any less and it overshoots too much 
double wheel_precision = 0.11/1.2;

struct wheel {
  double theta = 0;  
};

wheel left;

void setup() {
  Serial.begin(115200);
  
  //Initialize the left encoder/wheel
  

  pinMode(4, OUTPUT); //Ensures that the motor will move
  pinMode(7, OUTPUT); //Sets the Motor 1 direction as an output
  pinMode(8, OUTPUT); //Sets the Motor 2 direction as an output
  pinMode(9, OUTPUT); //Sets the Motor 1 speed as an output
  pinMode(10, OUTPUT); //Sets the Motor 2 speed as an output
  pinMode(12, INPUT); 
  pinMode(13, OUTPUT); //I2C connection 
  digitalWrite(4, HIGH); //Sets pin 4 to high so the motor works
  Wire.begin(SLAVE_ADDRESS); //Sets the 
  Wire.onReceive(receiveData);
  Serial.println("Ready!"); 

}

void loop() {
    left.theta = myEnc.read()*2*PI /N_in_rotation; //Calculates the theta based on how many clicks have occured in the encoder
    Tc = millis();
    if ( millis() % 10 == 0){
      double b[] = {(double)millis(), left.theta}; //{(double)digitalRead(M1Speed),(double)digitalRead(M1Dir),(double)millis(), left.theta};
      int p[] = {0,4}; //{0,0,0,3};
      tab_sep_print(b, p, 2); //Prints some stuff
      
      double true_theta = rotation_made * 2*PI + left.theta;
      VELOCITY = (- Radp + true_theta) / ( - Tp + Tc);
      Tp = Tc;
      Radp = true_theta;
      
      double desired_angle = (PI/2)*quadrant; //Sets the angle we are putting the motor to in radians
      
      if(!((desired_angle < (left.theta  + wheel_precision)) && (desired_angle > (left.theta  - wheel_precision)))){
        
        if(desired_angle - left.theta > 0){
           DIRECTION = HIGH; //Sets direction of motor to CW
        }else{
           DIRECTION = LOW; //Sets direction of motor to CCW
        }
        
        if(desired_angle - left.theta < 1 && DIRECTION == HIGH){
            SPEED = 100;
        }else if(desired_angle - left.theta > -1 && DIRECTION == LOW){
          SPEED = 100;
        }else{
          SPEED = 255;
        } 
        
        if(millis() > 250){ //only starts moving the motor 0.25s after the program starts
          if(DIRECTION == HIGH){//for when direction is cw
            if(!(desired_angle - left.theta < 0)){ 
            //DIRECTION
            digitalWrite(M1Dir, DIRECTION);
            analogWrite(M1Speed, SPEED);
            }
          }
          else if(DIRECTION == LOW){//for when direction is ccw
              if(!(desired_angle - left.theta > 0)){ 
              digitalWrite(M1Dir, DIRECTION);
              analogWrite(M1Speed, SPEED);
              }
          }
        }
      }else{
        SPEED = SPEED - 20;
            digitalWrite(M1Dir, DIRECTION);
            // SPEED OFF
            analogWrite(M1Speed, 0);
      }
      
  
   }
      if(Serial.available() > 0){ //If there is a '0' inputed to serial input then the current angle of the motor is set as 0 radians.
        if(Serial.read() == 48)
          myEnc.write(0);
        
      }
//      while(myEnc.read() >= 3200 || myEnc.read() < 0 ){
//        if(myEnc.read() >= 3200){
//          myEnc.write(myEnc.read() - 3200);
//          rotation_made++;
//        }else{
//          myEnc.write(myEnc.read() + 3200);
//          rotation_made--;
//      }
//   }
   n++;
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
      quadrant = Wire.read() - 1; //Sets the quadrant to the input from the pi. The -1 converts the signal to the desired quadrant
}
