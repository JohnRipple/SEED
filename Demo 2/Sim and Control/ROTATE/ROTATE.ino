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
 #include <Wire.h>
 #include <Encoder.h>
 #define SLAVE_ADDRESS 0x04
int label = 0;
// ============== Localization Initialization ==============
const double meterToFeet = 3.28084;
const double N_per_Rotation = 3200; // in ticks
const double radius = meterToFeet*0.145/2; //in meters
const double robot_width = meterToFeet*0.29; //in meters (Needs to be measured from wheel to wheel)
const double enc_to_rad = 2*PI/N_per_Rotation; //in radian / tick

bool duh = false;
int enc_last = 0;
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
bool STRAIGHT = true;

int todo = 0; //0: find tape; 1: center tape; 2: move forward till tape in lower 1/3; 3: 

int ONCE = 0;
//double voltage = 0;
//double presentVoltage = 8.0; //WHYYYYYYYYYYYYYYYYYYYYYYY
int PWM_value_M1 = 0; // DONT FORGET ABOUT THIS _____________________________________________________________________________________________________
int PWM_value_M2 = 0;
// double error = 0;
double errorDis = 0;
double errorPhi = 0;
double curDis = 0.0;
//IN FEET
//double circum = PI*radius*2;
//double perClick = circum /3200;
double lastPT = 0.0;
double lastPO = 0.0;
double angVelOne = 0.0;
double angVelTwo = 0.0;
double errorAngVel = 0.0;
double errorForVel = 0.0;
double errorHorizontal = 0.0;
double desForVel = 0.0;
double desAngVel = 0.0;
double barVoltage = 0.0;
double deltaVoltage = 0.0;
double samplingRate = 0.01;// in seconds, 10 milliseconds
double lastTime = 0;
double hamburger = 0;
double adjustVariable = 0 ;
double oneChange = 0;
double twoChange = 0;
double angStrong = 8;

double horizontalAngle = 0;
double boundingAngle = 0;


bool RotateForever = true;
//BELOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOWWWWW<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
double INPUT_ANGLE = 0;// ENTERED IN DEGREES
double INPUT_DISTANCE = 0.5;//ENTERED IN FEET
//ABOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVEEEEEEE<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//double desired_angle = (INPUT_ANGLE + (INPUT_ANGLE/90)*10)* PI/180; //CHANGE THIS CONTROLLER -(INPUT_ANGLE + (INPUT_ANGLE/90)*33.5
double desired_angle = (INPUT_ANGLE) * PI/180; //CHANGE THIS CONTROLLER -(INPUT_ANGLE + (INPUT_ANGLE/90)*33.5
//double desDis = INPUT_DISTANCE;// - (INPUT_DISTANCE/5)*( 0.105);
double desDis = INPUT_DISTANCE - (INPUT_DISTANCE/5)*( 0.105);

//Controller Specifications
//K was initially like 4.1

// It'll read the desired direction as the input, then it will output speed to reach the desired location
float Kp = 0.289202862842511*2; //in Volts/Radian //was 2
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
  Wire.begin(SLAVE_ADDRESS); //Sets the
  Wire.onReceive(receiveData);
  Serial.println("Ready!");
 delay(1000);
}

void loop() {
  
    update_position();// UPDATES THE R VALUE WHICH TELLS IT HOW FAR IT HAS DRIVEN
       
    if ( millis() % 10 == 0){
      
      intializeAngleVel();
      //desForVel = 3; //THIS IS A RANDOM VALUE, could include a different forward velocity in each function
      
      if(errorHorizontal > 3 || errorHorizontal < -3){
      desForVel = 2;
      alignCenter();
      } else{
      desForVel = 4;
      alignParallel();
      }      
      //This guy makes the thing rotate forever
      if(RotateForever){
        rotate(-1);
      }
      speedDirectionSet(); 
    }
   printTest(); 
    
   //delay(5);
}

void update_position(){ //Updates position for localization
  double d_r = right.displacement(); //sets a constant position throughout function
  double d_l = left.displacement();
 
  x = x + cos(phi)*(d_r + d_l) / 2; //updates x postion
  y = y + sin(phi)*(d_r + d_l) / 2; //updates y position
  r = sqrt(x*x + y*y); // upadates r / distance traveled
  
  phi = (right.read() - left.read()) * enc_to_rad * radius / robot_width; //updates orientation
}

void rotate(int direct){
  PWM_value_M1 = 20*direct;
  PWM_value_M2 = -20*direct;
  
//        if(RotateForever){
//        //if (millis() % 100 == 0){
//        PWM_value_M1 = 50;
//        PWM_value_M2 = 50;
//        if (errorDis < 0.1){
//          RotateForever = false;
//          desired_angle = 90 * PI/180;
//        }
//      } else{
//        if (errorPhi < 0.1)
//        {
//          RotateForever = true;
//          x = 0;
//          y = 0;
//          desDis = desDis + 0.1;
//        }
//      }
     
  
}

void intializeAngleVel(){
      //LEFT WHEEL DEFINED AS THE ONE, RIGHT WHEEL DEFINED AS THE TWO
      hamburger = millis() - lastTime;
      oneChange = left.theta() - lastPO;
      twoChange = right.theta() - lastPT;
      angVelOne = (oneChange) /(hamburger/1000);
      angVelTwo = (twoChange) / (hamburger/1000);
      lastTime = millis();
      lastPT = right.theta();
      lastPO = left.theta();
}

void alignCenter(){
      errorPhi = (0 - (boundingAngle));
           
      desAngVel = errorPhi / samplingRate;  

      errorForVel = desForVel - radius*(angVelOne + angVelTwo)/2;
      errorAngVel = -(desAngVel - radius*(angVelOne + angVelTwo)/(robot_width))*angStrong; //THIS WAS 7 abs(errorDis + errorPhi)

      barVoltage = errorForVel * Kp/2;
      deltaVoltage = errorAngVel * Kp;     
     
      PWM_value_M2 = ((barVoltage + deltaVoltage) / 2);
      PWM_value_M1 = ((barVoltage - deltaVoltage) / 2);
}

void alignParallel(){
      errorHorizontal = (90 - (horizontalAngle));
  
      desAngVel = errorHorizontal / samplingRate;  

      errorForVel = desForVel - radius*(angVelOne + angVelTwo)/2;
      errorAngVel = -(desAngVel - radius*(angVelOne + angVelTwo)/(robot_width))*angStrong; //THIS WAS 7 abs(errorDis + errorPhi)

      barVoltage = errorForVel * Kp/2;
      deltaVoltage = errorAngVel * Kp;     
     
      PWM_value_M2 = ((barVoltage + deltaVoltage) / 2);
      PWM_value_M1 = ((barVoltage - deltaVoltage) / 2);
}

void receiveData(int byteCount) {
      RotateForever = false;
      int arrayOfInputs[4] = {0};
      for(int i = 0; i < 4; i++) {
        arrayOfInputs[i] = Wire.read(); //Sets the quadrant to the input from the pi. The -1 converts the signal to the desired quadrant
      }
      //horiztonal
      horizontalAngle = arrayOfInputs[0] * -1^arrayOfInputs[1];
      //bounding
      boundingAngle = arrayOfInputs[2] * -1^arrayOfInputs[3];
}

void speedDirectionSet(){
  //CHOOSES DIRECTION     
      if(PWM_value_M1 > 0){
        DIRECTIONM1 = HIGH;
      } else{
        DIRECTIONM1 = LOW;
      }
     
      if(PWM_value_M2 > 0){
        DIRECTIONM2 = LOW;
      } else{
        DIRECTIONM2 = HIGH;
      }



      PWM_value_M1 = abs(PWM_value_M1);//-6*(8/INPUT_DISTANCE));
      if (PWM_value_M1 > 255) {
        PWM_value_M1 = 255;
      }
      PWM_value_M2 = abs(PWM_value_M2);
      if (PWM_value_M2 > 255) {
        PWM_value_M2 = 255;
      }
     
      
      //Something should probably be scaled for the minimum PWM needed to move the motor
      //WRITES THE SPEED AND DIRECTIONS TO THE MOTORS
     
      double bound = 100;
      if(PWM_value_M1 != 0) {
        PWM_value_M1 = ((double)PWM_value_M1)/(255)*(255-bound)+bound;
        
      }
      if(PWM_value_M2 != 0) {
        if(STRAIGHT) {
          PWM_value_M2 *= 0.923;
        }
        PWM_value_M2 = ((double)PWM_value_M2)/(255)*(255-bound)+bound;
      }  
     
      //enc_last = left.read();
      digitalWrite(M1Dir, DIRECTIONM1); //
      analogWrite(M1Speed, PWM_value_M1 );//PWM_value, WHEEL ON RIGHT SIDE IF LOOKING FROM BACK
      digitalWrite(M2Dir, DIRECTIONM2);
      analogWrite(M2Speed, PWM_value_M2);//PWM_value, WHEEL ON LEFT SIDE IF LOOKING FROM BACK
      //Serial.println(PWM_value);
     
}

void turnInPlace(double angle){ //TODO: Take input and turn that much
  
}

void moveForward(double ft){ //TODO: Take input and move forward that much
  while(PWM_value_M2 || PWM_value_M1){
    intializeAngleVel();
    
    errorForVel = desForVel - radius*(angVelOne + angVelTwo)/2;
    errorAngVel = -(desAngVel - radius*(angVelOne + angVelTwo)/(robot_width))*angStrong;

    if(errorDis < 0.5){
      angStrong = 1;
    }
    barVoltage = errorForVel * Kp/2;
    deltaVoltage = errorAngVel * Kp;
   
    PWM_value_M2 = ((barVoltage + deltaVoltage) / 2);
    PWM_value_M1 = abs((barVoltage - deltaVoltage) / 2);
    
  }
}

void testPrint(){
  if(millis() % 1000 == 0){//values to print for testing, can be deleted if desired
        if(label %12 == 0){
        Serial.print('\n');
//        Serial.print("phi");
//        Serial.print('\t');
//        Serial.print('\t');
        Serial.print("Error");
        Serial.print('\t');
        Serial.print("Er vel");
        Serial.print('\t');
        Serial.print("Er Ang");
        Serial.print('\t');
        Serial.print("Dis");
        Serial.print('\t');
        Serial.print("desDis");
        Serial.print('\t');    
        Serial.print("x");
        Serial.print('\t');
        Serial.print("y");
        Serial.print('\t');
        Serial.print("PWM L");
        Serial.print('\t');  
        Serial.print("PWM R");
        Serial.print('\t');
        Serial.print("Left");
        Serial.print('\t');
        Serial.print("Right");
        Serial.print('\n');    
        }
         
//        Serial.print(phi* 180/PI);
//        Serial.print('\t');
       
        Serial.print(errorPhi *180/PI);
        Serial.print('\t');
        Serial.print(barVoltage);
        Serial.print('\t');
        Serial.print(deltaVoltage);
        Serial.print('\t');
        Serial.print(r);
        Serial.print('\t');
        Serial.print(desDis);
        Serial.print('\t');
        Serial.print(x);
        Serial.print('\t');
        Serial.print(y);
        Serial.print('\t');
        Serial.print(PWM_value_M1);
        Serial.print('\t');
        Serial.print(PWM_value_M2);
        Serial.print('\t');
        Serial.print(left.read());
        Serial.print('\t');
        Serial.println(right.read());
        //Serial.print('\t');
        //Serial.print('\n');
        label ++;
    }
}
