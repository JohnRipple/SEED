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
double meterToFeet = 3.280839895;
const double N_per_Rotation = 3200; // in ticks
const double radius = 0.05*meterToFeet; //in meters
const double robot_width = 0.1*meterToFeet; //in meters (Needs to be measured from wheel to wheel)
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



int ONCE = 0;
double voltage = 0;
double presentVoltage = 8.0; //WHYYYYYYYYYYYYYYYYYYYYYYY
int PWM_value_M1 = 0;
int PWM_value_M2 = 0;
double error = 0;
double errorDis = 0;
double errorPhi = 0;
double curDis = 0.0;
//IN FEET
double circum = PI*radius*2;
double perClick = circum /3200;
double lastPT = 0.0;
double lastPO = 0.0;
double angVelOne = 0.0;
double angVelTwo = 0.0;
double errorAngVel = 0.0;
double errorForVel = 0.0;
double desForVel = 0.0;
double desAngVel = 0.0;
double barVoltage = 0.0;
double deltaVoltage = 0.0;
double samplingRate = 0.01;// in seconds, 10 milliseconds
//BELOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOWWWWW<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
double desired_angle = 40 * PI/180; // ENTERED IN DEGREES
double desDis = 1.5; //ENTERED IN FEET
//ABOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVEEEEEEE<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//Controller Specifications
//K was initially like 4.1 

// It'll read the desired direction as the input, then it will output speed to reach the desired location
float Kp = 0.289202862842511*15; //in Volts/Radian
float Ki = 10.8369142120677; //in volts /radian
double I_past = 0;
double I = 0;
// ============= END Motor Control Initialization =============


// ============= TEST VARIABLES =============

int label = 0;

// ============= END TEST VARIABLES =============


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

    //THIS IS NOTO NEEDED IN THE CURRENT BUILD, BUT IT IS A SWITCH CASE BETWEEN GOING STRAIGHT AND TURNING.
    if(ONCE == 0){
      //update_position();
      desDis = desDis + r;
    }
    ONCE++;
    //LIST OF POTENTIAL ERRORS:
    // 1. THE RIGHT AND LEFT WHEELS MAY NEED TO BE REASSIGNED AS EITHER ONE OR TWO
    // 2. DUE TO WIRING THE WHEELS ORIENTATION MAY BE FLIPPED
    // 3. WHEN FINDING THE PWM VALUES THE = AND - SIGNS MAY NEED TO BE FLIPPED
    // 4. IT CURRENTLY MAY ROTATE AND DRIVE FORWARD AT THE SAME TIME
    if ( millis() % 10 == 0){
      //RIGHT WHEEL DEFINED AS THE ONE, LEFT WHEEL DEFINED AS THE TWO
      angVelOne = (right.theta() - lastPO) /samplingRate;
      angVelTwo = (left.theta() - lastPT) / samplingRate;
      lastPT = left.theta();
      lastPO = right.theta(); 

      errorPhi = (desired_angle - (phi)); // already in radians ****ALSO NEED TO KNOW HOW THE ENCODERS ARE ORIENTED****
      errorDis = (desDis - r); //FORWARD ERROR TRACKING
      
      desForVel = errorDis / samplingRate; //MIGHT WANT TO ADJUST THE SPEED
      desAngVel = errorPhi / samplingRate;

      errorAngVel = desForVel - radius*(angVelOne + angVelTwo)/2;
      errorForVel = desAngVel - radius*(angVelOne + angVelTwo)/robot_width;

      barVoltage = errorForVel * Kp;
      deltaVoltage = errorAngVel * Kp;

      PWM_value_M1 = (barVoltage + deltaVoltage) / 2; //MIGHT NEED TO FLIP THE PLUS AND MINUS SIGNS ON THESE DEPENDING ON WHETHER YOU SET RIGHT WHEEL AS ONE OR TWO AND VICE VERSA
      PWM_value_M2 = (barVoltage - deltaVoltage) / 2;

      //CHOOSES DIRECTION
      if(PWM_value_M1 > 0){
        M1Dir = HIGH;
      } else{
        M1Dir = LOW;
      }
      
      if(PWM_value_M2 > 0){
        M2Dir = LOW;
      } else{
        M2Dir = HIGH;
      }
      //MAKES SURE THE PWM IS WITHIN THE BOUNDS
      if (PWM_value_M1 > 255 || PWM_value_M1 < -255) {
        PWM_value_M1 = 255;
      }
      else{ 
        PWM_value_M1 = abs(PWM_value_M1); 
      }

      if (PWM_value_M2 > 255 || PWM_value_M2 < -255) {
        PWM_value_M2 = 255;
      }
      else{ 
        PWM_value_M2 = abs(PWM_value_M2); 
      }


      //Something should probably be scaled for the minimum PWM needed to move the motor
      //WRITES THE SPEED AND DIRECTIONS TO THE MOTORS
      digitalWrite(M1Dir, DIRECTIONM1);
      analogWrite(M1Speed, PWM_value_M1);//PWM_value
      digitalWrite(M2Dir, DIRECTIONM2);
      analogWrite(M2Speed, PWM_value_M2);//PWM_value
      //Serial.println(PWM_value);
      
  
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
        Serial.print(errorPhi *180/PI);
        Serial.print('\t');
        Serial.print(r);
        Serial.print('\t');
        Serial.print(desDis);
        Serial.print('\t');
        Serial.print(x);
        Serial.print('\t');
        Serial.print(y);
        Serial.print('\t');
        Serial.println(PWM_value_M1);
        Serial.println(PWM_value_M2);
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
