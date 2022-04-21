/* Author: Andrew Samson, John Ripple, Michael Klima, Josh Lee 3/02/2022
 *  
 * Description: This code controls a robots movement based on angle inputs from a rasberry pi.
 *
 * Hardware:
 *
 * connect the encoder on left wheel to pins 2 and 5 
 * connect the encoder on right wheel to pins 3 and 6 
 * connect GND on the wire splitter to GND on the arduino
 * connect + on wire splitter to the 5V on the arduino
 *
 */
#include <Encoder.h>
// ============== Localization Initialization ==============
const double meterToFeet = 3.28084;
const double N_per_Rotation = 3200; // in ticks
const double radius = meterToFeet*0.145/2; //in meters
const double robot_width = meterToFeet*0.29; //in meters (Needs to be measured from wheel to wheel)
const double enc_to_rad = 2*PI/N_per_Rotation; //in radian / tick
const double toRad = PI/180;

int on = 0;

double x = 0; //in meters
double y = 0; //in meters
double r = 0; //in meters
double phi = 0; //in radians
double distanceTotal = 0; //in feet

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

int Rotate = 1;
int Forward = 0;
int AlignS = 0;

int PWM_value_R = 0; // DONT FORGET ABOUT THIS _____________________________________________________________________________________________________
int PWM_value_L = 0;

double errorDis = 0;
double errorPhi = 0;

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
double oneChange = 0;
double twoChange = 0;
double angStrong = 1;
double setForwardVelocity = 4;

double horizontalAngle = 0;
double shiftAngle = 0;
int stopSig = 0;
double INPUT_DISTANCE = 0.5;
double INPUT_ANGLE = 0;

bool Vision = false;
bool halt = false;
bool useHAngle = false;

//double desired_angle = (INPUT_ANGLE + (INPUT_ANGLE/90)*10)* PI/180; //CHANGE THIS CONTROLLER -(INPUT_ANGLE + (INPUT_ANGLE/90)*33.5
double desired_angle = (INPUT_ANGLE) * PI/180; //CHANGE THIS CONTROLLER -(INPUT_ANGLE + (INPUT_ANGLE/90)*33.5
//double desDis = INPUT_DISTANCE;// - (INPUT_DISTANCE/5)*( 0.105);
double desDis = INPUT_DISTANCE - (INPUT_DISTANCE/5)*( 0.105);

//Controller Specifications

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
  digitalWrite(4, HIGH); //Sets pin 4 to high so the motor works
  delay(1000);
}

void loop() {
    update_position();// UPDATES THE R VALUE WHICH TELLS IT HOW FAR IT HAS DRIVEN
    if (Serial.available() > 0) {
      receiveData();
    }
       
    if ( millis() % 10 == 0){
      intializeAngleVel();
      if(Rotate) {
        rotate(-1);
      }

      if(AlignS) {
        turnToTape(shiftAngle, horizontalAngle);
        
      }
      
      if(Forward) {
        moveForward(setForwardVelocity);
          
      }
      if (!Rotate) {
        PWM_value_L = ((barVoltage + deltaVoltage) / 2); // Used to be /2
        PWM_value_R = ((barVoltage - deltaVoltage) / 2);
      }
      speedDirectionSet(); 
      
    }
}


// Gets Data from Pi
void receiveData() {
      Rotate = false;
      AlignS = true;
      //halt = false;
      // Array of Inputs from Pi
      int arrayOfInputs[5] = {0};
      String data = Serial.readStringUntil('\n');
      int start = 0;
      int count = 0;
      Serial.print("You Sent me: ");
      for(int i = 0; i < data.length(); i++){
       if(data[i] == ' ') {
         arrayOfInputs[count] = data.substring(start, i).toInt();
         start = i;
         count++;
       }
      }
     
     Serial.println(data); 
     
      stopSig = arrayOfInputs[4];
      if(stopSig == 0) {
        // The Tape is in sight
        //Set Horizontal Angle
        horizontalAngle = arrayOfInputs[0] * toRad * pow(-1,arrayOfInputs[1]); // Need to add toRad back in ///////////////* pow(-1,arrayOfInputs[1]) 
        //Set Shift Angle
        shiftAngle = arrayOfInputs[2] * pow(-1,arrayOfInputs[3])* toRad;

        Vision = true;
        Rotate = false;
        
      } else if (stopSig == 1){
        // Stopping at cross
        if(Vision && r > 1){
          halt = true;
        }
        Vision = false;
        
      } else if (stopSig == 2) {
        // No tape seen
        Vision = false;
        Rotate = true;
      }
     
}

void speedDirectionSet(){
      //CHOOSES DIRECTION     
      if(PWM_value_R > 0){
        DIRECTIONM1 = HIGH;
      } else{
        DIRECTIONM1 = LOW;
      }
     
      if(PWM_value_L > 0){
        DIRECTIONM2 = LOW;
      } else{
        DIRECTIONM2 = HIGH;
      }

      //Makes sure PWM is within 0 to 255
      PWM_value_R = constrain(abs(PWM_value_R),0,255);
      
      PWM_value_L = constrain(abs(PWM_value_L),0,255);
      
     
      // Scales PWM to lowest moving motor value
      double bound = 70;
      if (Rotate) {
        bound = 70;
      }
      if(PWM_value_R != 0) {
        PWM_value_R = ((double)PWM_value_R)/(255)*(255-bound)+bound;
      }
      if(PWM_value_L != 0) {
        PWM_value_L = ((double)PWM_value_L)/(255)*(255-bound)+bound;
      }  

     if(halt){
        int tmp = r+0;
        while (tmp - r > 0) {
          update_position();
        }

      PWM_value_R = 0;
      PWM_value_L = 0;
     }

      // Writes values to motors
      digitalWrite(M1Dir, DIRECTIONM1); //
      analogWrite(M1Speed, PWM_value_R);//PWM_value_, WHEEL ON RIGHT SIDE IF LOOKING FROM BACK
      digitalWrite(M2Dir, DIRECTIONM2);
      analogWrite(M2Speed, PWM_value_L);//PWM_value_, WHEEL ON LEFT SIDE IF LOOKING FROM BACK
  
      if(halt && on == 0){
        victoryScreech();
        on++;
        while(halt) {
          
        }
      }
     
}

void turnToTape(double angle, double angleH){ //TODO: Take input and turn that much
  if(abs(angleH) < 20*toRad || useHAngle) {
    angle = (-90*toRad+abs(angleH))/3.46;
    if(distanceTotal > 3) {
      useHAngle = true;
    }
    if (abs(angleH) > 60*toRad) {
      useHAngle = false;
    }
  }
 
  if(abs(angle) > 1*toRad) { 
    update_position();
    // Set errorPhi
    errorPhi = angle;
    
    // Set Desired Velocities
    desAngVel = errorPhi / samplingRate; 
  
    // Set error values
    errorAngVel = -(desAngVel - radius*(angVelOne + angVelTwo)/(robot_width))/2;
  
    barVoltage = 0;
    deltaVoltage = errorAngVel * Kp * angStrong;     
    
  } else {
    Forward = true;
    angStrong = 5; //was 8
  }
}

void moveForward(double ft){
  errorDis = (desDis - r);
  desForVel = errorDis / samplingRate;
  errorForVel = desForVel - radius*(angVelOne + angVelTwo)/4;
  barVoltage = errorForVel * Kp/2;
  if (Vision){
       desDis = r + ft; //should only happen once until it reaches its place of rest
  }
}


///Everything below is down pat

void rotate(int direct){
  PWM_value_R = 1*direct;
  PWM_value_L = -1*direct;
}

void update_position(){ //Updates position for localization
  double d_r = right.displacement(); //sets a constant position throughout function
  double d_l = left.displacement();

  double xOld = x;
  double yOld = y;
  x = cos(phi)*(d_r + d_l) / 2; //updates x postion
  y = sin(phi)*(d_r + d_l) / 2; //updates y position
 
  distanceTotal += sqrt(x*x + y*y);
  
  phi = (right.read() - left.read()) * enc_to_rad * radius / robot_width; //updates orientation
  x = x + xOld;
  y = y + yOld;
  r = sqrt(x*x + y*y); // upadates r / distance away from origin
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
      int cornerAngleBound = 90;
      if (abs(horizontalAngle) < (cornerAngleBound * toRad)){ // Makes the forward velocity proportional with the error from horizontal angle
        setForwardVelocity = 4*(abs(horizontalAngle)/(cornerAngleBound * toRad *2));
      } else{
        setForwardVelocity = 4;
      }
      if (setForwardVelocity < .1) {
        setForwardVelocity = 0.1;
      }
}

void victoryScreech(){
  int qua = 500;
  int trip = qua/3;

//   int notes[] = {196,264, 330, 400, 524,660,785,660,   0, 0,
//     264,315,415,524,622,831,622, 0,0,
//     293, 350, 466, 587, 698, 932, 932, 932, 932, 1046};
//   int durr[] = {trip,trip,trip,trip,trip,trip,qua,qua/2, qua/2, trip, 
//     trip, trip, trip, trip, trip, qua, qua/2, qua/2, trip,
//     trip, trip, trip, trip, trip, qua,trip, trip ,trip , 3*qua};
  int notes[] = {261, 350, 440, 350, 440, 392, 350, 293, 261, 261, 
    350, 440, 350, 440, 392, 523, 440,
    523, 440, 524, 440, 350, 261, 293, 350, 350, 293, 261, 261,
    350, 440, 350, 440, 392, 350};
  int durr[] = {qua, 2*qua, qua/2, qua/2, 2*qua, qua, 2*qua, qua, 2*qua, qua,
    2*qua, qua/2, qua/2, 2*qua, qua, 5*qua, qua,
    1.5*qua, qua/2, qua/2, qua/2, qua*2, qua, 1.5*qua, qua/2, qua/2, qua/2, 2*qua, qua,
    2*qua, qua/2, qua/2, qua*2, qua, qua*5};
  for(int i = 0; i < 35; i++){
      tone(11, notes[i], durr[i]);
      delay(durr[i]);
    }
}
