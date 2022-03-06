/* Author: Andrew Samson 3/02/2022
 *  
 * Description: This code is used to keep track of the position and orientation (radians off of x-axis)
 * of the robot relative to its starting point (0,0)
 * 
 * Hardware: 
 * 
 * connect the clk on the encoder to pins 2 and 3 (interrupt pins)
 * connect the dt on the encoder to pins 5 and 6
 * connect GND on the encoder to GND on the arduino
 * connect + on encoder to the 5V on the arduino
 * 
 */
 //#include <Wire.h>
 #include <Encoder.h>
 //#define SLAVE_ADDRESS 0x04

// ============== Localization Initialization ==============
const double N_per_Rotation = 3200; // in ticks
const double radius = 0.05; //in meters
const double robot_width = 0.1; //in meters (Needs to be measured from wheel to wheel)
const double enc_to_rad = 2*PI/N_per_Rotation; //in radian / tick

double x = 0; //in meters
double y = 0; //in meters
double r = 0; //in meters
double phi = 0; //in radians

struct wheel :  Encoder{
  wheel(int x, int y, int s) : Encoder(x,y){side = s;};  //constructor
  long read(){
    return Encoder::read() * side;
  }
  double theta(){
    return read() * enc_to_rad;
  }
  
  double displacement(){ //returns the displacement made by wheel since it was last called
    int temp = read();
    double tbr = (temp - theta_last) * enc_to_rad * radius;
    theta_last = temp;
    return tbr;
  }
  int side = 1;
  long theta_last = 0;
};

wheel left(2,5,-1);
wheel right(3,6,1);

// ============== END Localization Initialization ==============


// ============== Motor Control Initialization ==============
int M1Speed = 9; //sets some pins
int M2Speed = 10;
int M1Dir = 7;
int M2Dir = 8;
int DIRECTIONM1 = HIGH;
int DIRECTIONM2 = HIGH;
bool STRAIGHT = false;

double meterToFeet = 3.280839895;

int ONCE = 0;
double voltage = 0;
double presentVoltage = 8.0; //WHYYYYYYYYYYYYYYYYYYYYYYY
int PWM_value = 0;
double error = 0;
double errorDis = 0;
double curDis = 0.0;
//IN FEET
double circum = PI*radius*2*meterToFeet;
double perClick = circum /3200;

//BELOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOWWWWW<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
double desired_angle = 40 * PI/180; // ENTERED IN DEGREES
double desDis = 1.5; //ENTERED IN FEET
//ABOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVEEEEEEE<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//Controller Specifications
//K was initially like 4.1 

// It'll read the desired direction as the input, then it will output speed to reach the desired location
float Kp = 0.289202862842511*10; //in Volts/Radian
float Ki = 10.8369142120677; //in volts /radian
double I_past = 0;
double I = 0;
// ============= END Motor Control Initialization =============

void setup() {
  Serial.begin(115200);
  
  pinMode(4, OUTPUT); //Ensures that the motor will move
  pinMode(M1Dir, OUTPUT); //Sets the Motor 1 direction as an output
  pinMode(M2Dir, OUTPUT); //Sets the Motor 2 direction as an output
  pinMode(M1Speed, OUTPUT); //Sets the Motor 1 speed as an output
  pinMode(M2Speed, OUTPUT); //Sets the Motor 2 speed as an output
  pinMode(12, INPUT); 
  pinMode(13, OUTPUT); //I2C connection 
  digitalWrite(4, HIGH); //Sets pin 4 to high so the motor works
  //Wire.begin(SLAVE_ADDRESS); //Sets the 
  //Wire.onReceive(receiveData);
  Serial.println("Ready!");
}
int itt = 0;
void loop() {

    update_position();// UPDATES THE R VALUE WHICH TELLS IT HOW FAR IT HAS DRIVEN  

    if(ONCE == 0){
      //update_position();
      desDis = desDis + r;
    }
    ONCE++;

    if ( millis() % 10 == 0){
      Serial.println(left.read());
      error = (desired_angle - (phi)); // already in radians ****ALSO NEED TO KNOW HOW THE ENCODERS ARE ORIENTED****
      //(desired_angle - (robot_width/(radius*2))*(left.theta()+right.theta())); // already in radians ****ALSO NEED TO KNOW HOW THE ENCODERS ARE ORIENTED****
      //^^^^^^ROTATION ERROR TRACKING
      errorDis = (desDis - r); //FORWARD ERROR TRACKING
      
      //IF THE CONTROLLER IS IN TURN ROBOT MODE>>>>>
      if(error > 0 && STRAIGHT == false){
        DIRECTIONM2 = HIGH; // CW
        DIRECTIONM1 = HIGH;
      } else if(error <= 0 && STRAIGHT == false){
        DIRECTIONM2 = LOW; //Sets direction of motor to CCW
        DIRECTIONM1 = LOW;
      }
      //IF THE CONTROLLER IS IN DRIVE FORWARD MODE>>>>>
      if(errorDis > 0 && STRAIGHT == true){
        DIRECTIONM1 = HIGH; // CW
        DIRECTIONM2 = LOW;
      } else if (errorDis <= 0 && STRAIGHT == true){
        DIRECTIONM1 = LOW; //Sets direction of motor to CCW
        DIRECTIONM2 = HIGH;
      }
      // Ki part
      I = I_past+((error)*0.008);    
      I_past = I;
      
      //CHOOSES BETWEEN THE CONTROLLER WHICH DRIVES STRAIGHT OR TURNS THE ROBOT
      if(STRAIGHT == false){
        voltage = (Kp * (error)) + (Ki * I);  
      } else{
        voltage = (Kp * (errorDis));    
      }
      //SETS THE PWM VALUE
      PWM_value = ((voltage/presentVoltage))*255;

      //SETS THE MOTOR SPEED VIA PWM
      if (PWM_value > 255 || PWM_value < -255) {
        PWM_value = 255;
      }
      else{ 
        PWM_value = abs(PWM_value); 
      }
      //Something should probably be scaled for the minimum PWM needed to move the motor
      //WRITES THE SPEED AND DIRECTIONS TO THE MOTORS
      digitalWrite(M1Dir, DIRECTIONM1);
      analogWrite(M1Speed, PWM_value);//PWM_value
      digitalWrite(M2Dir, DIRECTIONM2);
      analogWrite(M2Speed, PWM_value);//PWM_value
      Serial.println(PWM_value);
      
      //CHECKS TO SEE IF IT CAN GO FORWARD
      if(error < 2  && !STRAIGHT){
        STRAIGHT = true;
        ONCE = 0;  
        Serial.print("Done Turning");
        Serial.print('\n');
      }else if(STRAIGHT && error > 3.5){
        STRAIGHT = false;
        Serial.print("Turning again");
        Serial.print('\n');
      }
  
    }
  if(millis() % 1000 == 0){//values to print for testing, can be deleted if desired
        if(label %12 == 0){
        Serial.print('\n');
        Serial.print("phi");
        Serial.print('\t');
        Serial.print("Error");
        Serial.print('\t');
        Serial.print("Dis");
        Serial.print('\t');
        Serial.print("desDis");
        Serial.print('\t');    
        Serial.print("x");
        Serial.print('\t'); 
        Serial.print("y");
        Serial.print('\t'); 
        Serial.print("PWM");
        Serial.print('\n');       
        }
        Serial.print(phi* 180/PI);
        Serial.print('\t');
        Serial.print(error *180/PI);
        Serial.print('\t');
        Serial.print(r);
        Serial.print('\t');
        Serial.print(desDis);
        Serial.print('\t');
        Serial.print(x);
        Serial.print('\t');
        Serial.print(y);
        Serial.print('\t');
        Serial.println(PWM_value);
        label ++;
    }
}
  


void update_position(){ //Updates position for localization 
  double d_r = right.displacement(); //sets a constant position throughout function
  double d_l = left.displacement();
  
  x = x + cos(phi)*(d_r + d_l) / 2; //updates x postion
  y = y + sin(phi)*(d_r + d_l) / 2; //updates y position
  r = meterToFeet * sqrt(x*x+y*y); // upadates r / distance traveled
  phi = (right.read() - left.read()) * enc_to_rad * radius / robot_width; //updates orientation
}
