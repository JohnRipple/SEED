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
 
int N_in_rotation = 64; //Clicks in an encoder for full circle
double radius = 0.05; //Wheel radius
double b = 0.1; //robot width
double x = 0; //position
double y = 0;
double phi = 0;
unsigned long time_last = 0;

struct wheel {
  int pos = 0;
  int a_last = 0;
  int b_last = 0;
  int clk_pin;
  int dt_pin;
  double theta = 0;  
  double theta_last = 0;
  unsigned long last_int = 0;
};

wheel left;
wheel right;

void setup() {
  Serial.begin(9600);
  
  //Initialize the left encoder/wheel
  pinMode(3, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  left.clk_pin = 3;
  left.dt_pin = 5;
  left.a_last = digitalRead(left.clk_pin);
  left.b_last = digitalRead(left.dt_pin);
  attachInterrupt(digitalPinToInterrupt(left.clk_pin), rotary_left, LOW);

  //Initialize the Right encoder/wheel
  pinMode(2, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  right.clk_pin = 2;
  right.dt_pin = 4;
  right.a_last = digitalRead(right.clk_pin);
  right.b_last = digitalRead(right.dt_pin);
  attachInterrupt(digitalPinToInterrupt(right.clk_pin), rotary_right, LOW); //adjust pins that this is used in code
}

void loop() { //yet to be tested
  if (Serial.available() == 48) { //if serial input is '0' then it will set all the thetas to 0
    left.theta = 0;
    right.theta = 0;
  }
}

void rotary_left(){
  if(left.last_int + 50 < millis())
    rotary(left);
  
}

void rotary_right(){
  if(right.last_int + 50 < millis())
    rotary(right);
  
}

void find_vars(){
  
  double d_r = (right.theta - right.theta_last)*radius;
  double d_l = (left.theta - left.theta_last)*radius;
  
  Serial.println(left.theta); 
 
  right.theta_last = right.theta;
  left.theta_last = left.theta;

  unsigned long time_now = millis();
  
  //This section is used to calculate the time, left, and right velocities
  if(time_now != time_last){
    
    double v_r = d_r /(double)(time_now - time_last)*1000; //dont forget time now is in ms, so multiply by 1000 to convert to /s
    double v_l = d_l /(double)(time_now - time_last)*1000; //Caluclates velocity of left and right based on prior postition and time
                                                                //compared to current position and time
    //double b[] = {(double)time_now/1000, v_l, v_r};
    //int print_precision[] = {3,6,6};
    //tab_sep_print(b,print_precision, 3); //print it     
                                                         
  }
  
  // This section  calculates and prints the current position and angle orientation of the bot
//  x = x + cos(phi)*(d_r + d_l)/2;
//  y = y + sin(phi)*(d_r+d_l)/2;
//  phi = phi + radius/b*(d_l-d_r);
//
//  Serial.println("x y phi:");
//  double b[] = {x,y,phi};
//  int print_precision[] = {3,3,3};

  time_last = time_now;
}

void rotary(wheel &whe){
  int a_now = digitalRead(whe.clk_pin); 
  int b_now = digitalRead(whe.dt_pin); 
    
  if(a_now == b_now ){
      whe.pos--;
  }else{
     whe.pos++;    
  }   
      
  whe.theta = (double)whe.pos*2*3.14159/(double)N_in_rotation; //2pi*pos/N_in_rotation;//puts boundary of 50 ms to reduce jitter
  find_vars();
  whe.last_int = millis();
}

//Prints a bunch of variables separated by a tab '\t'
void tab_sep_print(double arr[], int precision[], int arrSize){ 
  for(int i = 0; i < arrSize; i++){
    Serial.print(arr[i],precision[i]);
    Serial.print('\t');
  }
  Serial.print('\n');
  
}
