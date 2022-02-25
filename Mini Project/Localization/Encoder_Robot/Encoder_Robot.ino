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

 #include <Encoder.h>

 Encoder myEnc(3,5);
 
int N_in_rotation = 3200; //Clicks in an encoder for full circle
double radius = 0.05; //Wheel radius
double b = 0.1; //robot width
double x = 0; //position
double y = 0;
double phi = 0;
int time_precision = 25;

int M1Speed = 9;
int M2Speed = 10;
int M1Dir = 7;
int M2Dir = 8;
int n = 1;
int quadrant = 3;

double wheel_precision = 0.11;

struct wheel {
  double theta = 0;  
};

wheel left;

void setup() {
  Serial.begin(19200);
  
  //Initialize the left encoder/wheel
  

  pinMode(4, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(12, INPUT);

  digitalWrite(4, HIGH);

}

void loop() { //yet to be tested
  left.theta = myEnc.read()*2*PI /N_in_rotation;
 

  if ( millis() % 250 == 0){
    double b[] = {(double)millis(), left.theta}; //{(double)digitalRead(M1Speed),(double)digitalRead(M1Dir),(double)millis(), left.theta};
    int p[] = {0,4}; //{0,0,0,3};
    tab_sep_print(b, p, 2);
    //if((n*10+1) < millis() && (n*10-1) < millis() ){ //What is this?
      
//      Serial.println(digitalRead(M1Speed));
//      Serial.println(digitalRead(M1Dir));
//      Serial.print(millis());
//      Serial.print("\t");
//      Serial.println(ANGULARR_VELOCITY);
//      Serial.println(left.theta); 

    //}

  //TODO:
  //DETERMINE WHETHER OR NOT IT SHOULD SPIN
//    double desired_angle = PI/2*quadrant;
//    
//    if(!(left.theta  < desired_angle + wheel_precision && left.theta  > desired_angle - wheel_precision)){ //CHECK THIS LOGIC
//  //DIRECTION
//      digitalWrite(M1Dir, HIGH);
//  // SPEED FULL POWER
//      analogWrite(M1Speed, 100);
//    }
//
//    //THIS FLIPS THE DIRECTION
//    else if(2000 < millis()){
//  //DIRECTION
//      digitalWrite(M1Dir, LOW);
//  // SPEED FULL POWER
//      analogWrite(M1Speed, 0);
//    }
  }
 
   // ---------Same as above just outside of the if statemt--------
  double desired_angle = PI/2*quadrant;
    
    if(!(left.theta  < desired_angle + wheel_precision && left.theta  > desired_angle - wheel_precision)){ //CHECK THIS LOGIC
  //DIRECTION
      digitalWrite(M1Dir, HIGH);
  // SPEED FULL POWER
      analogWrite(M1Speed, 100);
    }else{
      digitalWrite(M1Dir, LOW);
  // SPEED FULL POWER
      analogWrite(M1Speed, 0);
    }

    
    
  if(left.theta >= 2*PI || (Serial.available() > 0 && Serial.read() == 48)){
    myEnc.write(0);
  }
  
  if((Serial.available() > 0 && Serial.read() == 49)){
    quadrant = (quadrant+1) % 4;
    Serial.println("BUTTS");
  }
  
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
