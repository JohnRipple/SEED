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
#include <Encoder.h>

const double N_per_Rotation = 3200; // in ticks
const double radius = 0.05; //in meters
const double robot_width = 0.1; //in meters (Needs to be measured from wheel to wheel)
const double enc_to_rad = 2*PI/N_per_Rotation; //in radian / tick

double x = 0; //in meters
double y = 0; //in meters
double r = 0; //in meters
double phi = 0; //in radians

struct wheel :  Encoder{
  wheel(int clk, int dt) : Encoder(clk,dt){};  //constructor
  double displacement(){ //returns the displacement made by wheel since it was last called
    int temp = read();
    double tbr = (temp - theta_last) * enc_to_rad * radius;
    theta_last = temp;
    return tbr;
  }
  long theta_last = 0;
};

wheel left(2,5);
wheel right(3,6);

void setup() {
  Serial.begin(9600);
  
}

void loop() {
  update_position();

}

void update_position(){
  double d_r = right.displacement(); //sets a constant position throughout function
  double d_l = left.displacement();

  x = x + cos(phi)*(d_r + d_l) / 2; //updates x postion
  y = y + sin(phi)*(d_r + d_l) / 2; //updates y position
  r = sqrt(x*x+y*y); // upadates r / distance traveled
  phi = phi + radius / robot_width * (d_l - d_r); //updates orientation
}
