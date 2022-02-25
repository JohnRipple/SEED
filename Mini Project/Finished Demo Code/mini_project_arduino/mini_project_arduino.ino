
/* Author: Andrew Samson 2/14/2022
 *  
 * Description: This code is used to calculate and print the position (x,y,phi) of the robot or; velocities, 
 * or angles of wheels on a robot based on mechanical encoder input.
 * 
 * Hardware: 
 * connect the clk on the encoder to pins 2 and 3 (interrupt pins)
 * connect the dt on the encoder to pins 4 and 5
 * connect GND on the encoder to GND on the arduino
 * connect + on encoder to the 5V on the arduino
 * 
 */
 #include <Wire.h>
 #include <Encoder.h>
 #define SLAVE_ADDRESS 0x04
 Encoder myEnc(3,5);


int N_in_rotation = 3200; //Clicks in an encoder for full circle
double radius = 0.05; //Wheel radius
double b = 0.1; //robot width
double x = 0; //position
double y = 0;
double phi = 0;
int rotation_made = 0;
int time_precision = 25;

int M1Speed = 9;
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
float Kp = 0.289202862842511;
float Ki = 10.8369142120677;
float e_past = 0;
double Tp = 0;
double Tc =0;
double Radp = 0;
double VELOCITY = 0;    

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
  

  pinMode(4, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(12, INPUT);
  pinMode(13,OUTPUT);
  digitalWrite(4, HIGH);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Serial.println("Ready!");

}

void loop() { //yet to be tested
    left.theta = myEnc.read()*2*PI /N_in_rotation;
    Tc = millis();
    if ( millis() % 10 == 0){
      double b[] = {(double)millis(), left.theta}; //{(double)digitalRead(M1Speed),(double)digitalRead(M1Dir),(double)millis(), left.theta};
      int p[] = {0,4}; //{0,0,0,3};
      tab_sep_print(b, p, 2);
      //Serial.println(SPEED);
      double true_theta = rotation_made * 2*PI + left.theta;
      VELOCITY = (- Radp + true_theta) / ( - Tp + Tc);
      Tp = Tc;
      Radp = true_theta;
      double desired_angle = (PI/2)*quadrant;
      if(!((desired_angle < (left.theta  + wheel_precision)) && (desired_angle > (left.theta  - wheel_precision)))){
          if(desired_angle - left.theta > 0){
             DIRECTION = HIGH;
          }else{
             DIRECTION = LOW;
          }
          if(desired_angle - left.theta < 1 && DIRECTION == HIGH){
              SPEED = 100;
          } else if(desired_angle - left.theta > -1 && DIRECTION == LOW){
            SPEED = 100;
          } else{
            SPEED = 255;
          } 
        if(millis() > 250){
        if(DIRECTION == HIGH){
            if(!(desired_angle - left.theta < 0)){ //CHECK THIS LOGIC
            //DIRECTION
            digitalWrite(M1Dir, DIRECTION);
            // SPEED FULL POWER
            analogWrite(M1Speed, SPEED);
            }
        }
        else if(DIRECTION == LOW){
              if(!(desired_angle - left.theta > 0)){ //CHECK THIS LOGIC
              //DIRECTION
              digitalWrite(M1Dir, DIRECTION);
              // SPEED LESS THAN HALF
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
      if(Serial.available() > 0){
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
      quadrant = Wire.read() - 1;
}