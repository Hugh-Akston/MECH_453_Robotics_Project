// Written By: Dr. Eric Markvicka
// Modified By: Luke Freyhof
// Purpose: This file serves as a guideline for implementing inverse kinematics in the robot arm for MECH 453/853

#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Robotic arm parameters.
// These will need to be modified based on your arm
////// Theta 1 //////
#define J0_min  88   // min pulselength
#define J0_max  484   // max pulselength
#define d0_min  -90   // corresponding limit in degrees (min)
#define d0_max  90    // max degrees

////// Theta 2 //////
#define J1_min  112    // min pulselength
#define J1_max  488   // max pulselength
#define d1_min  0     // corresponding limit in degrees (min)
#define d1_max  180    // max degrees

////// Theta 3 //////
#define J2_min  112   // min pulselength
#define J2_max  412   // max pulselength
#define d2_min  -90     // corresponding limit in degrees (min)
#define d2_max  90    // max degrees

////// End Effector //////
#define J3_min  304   // pulselength in open position
#define J3_max  412   // pulselength in closed position
#define d3_min  0     // corresponding min distance in mm
#define d3_max  10    // max distance in mm


char incomingByte = 0; // for incoming serial data

// Reference configuration of the robotic arm
float x = 255.0; // mm
float y = 0.0;   // mm
float z = 110.0; //mm

// variables used to calculate the inverse kinematics
float t1 = 0.0;
float d1 = 0.0;

float t2 = 0.0;
float d2 = 0.0;

float t3 = 0.0;
float d3 = 0.0;
float t3cos = 0.0;

// float d1 = 110.0;
// float l2 = 105.0;
// float l3 = 150.0;

// Whiteboard Points
float points[][4] = {
  {1, 77.5, 0.0, 305.0},
  {2, 67.5, 0.0, 315.0},
  {3, 67.5, 0.0, 295.0},
  {4, 57.5, 0.0, 305.0},
  {5, 47.5, 0.0, 335.0},
  {6, 47.5, 0.0, 305.0},
  {7, 47.5, 0.0, 275.0},
  {8, 17.5, 0.0, 335.0},
  {9, 17.5, 0.0, 305.0},
  {10, 17.5, 0.0, 275.0},
  {11, 7.5, 0.0, 280.0},
  {12, 7.5, 0.0, 275.0},
  {13, 2.5, 0.0, 280.0},
  {14, 2.5, 0.0, 275.0},
  {15, 7.5, 0.0, 305.0},
};

// Character Points
int plus_index[] = {1, 4, 15, 2, 3};
int minus_index[] = {1, 4};
int zero_index[] = {5, 6, 7, 10, 9, 8};
int one_index[] = {8, 9, 10};
int two_index[] = {5, 8, 9, 6, 7, 10};
int three_index[] = {5, 8, 9, 6, 9, 10, 7};
int four_index[] = {5, 6, 9, 8, 9, 10};
int five_index[] = {8, 5, 6, 9, 10, 7};
int six_index[] = {8, 5, 6, 7, 10, 9, 6}; // Points index for 6
int seven_index[] = {5, 8, 9, 10};
int eight_index[] = {5, 6, 7, 10, 9, 8, 5, 6, 9};
int nine_index[] = {9, 6, 5, 8, 9, 10};
int period_index[] = {11, 12, 14, 13};

float char_index_size = sizeof(two_index) / sizeof(two_index[1]);

int grip = 400;

void setup() {
  Serial.begin(115200);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

  // Move the robotic arm to the reference configuration
  pwm.setPWM(0, 0, map(0,d0_min,d0_max,J0_min,J0_max));
  pwm.setPWM(1, 0, map(0,d1_min,d1_max,J1_min,J1_max));
  pwm.setPWM(2, 0, map(0,d2_min,d2_max,J2_min,J2_max));
  pwm.setPWM(3, 0, grip);

}

void loop() {

  // send data only when you receive data: 
  if (Serial.available() > 0) { 
    // read the incoming byte: 
    incomingByte = Serial.read();

    // say what you got: 
    Serial.println(incomingByte);

    // print the current x, y, z value
    Serial.print("x: "); Serial.print(x);
    Serial.print(", y: "); Serial.print(y);
    Serial.print(", z: "); Serial.println(z);
    switch (incomingByte) {
      case 'q':  // move in x
        x += 5;
        break;
      case 'a':
        x -= 5;
        break;
        
      case 'w':  // move in y
        y += 5;
        break;
      case 's':
        y -= 5;
        break; 
        
      case 'e':  // move in z
        z += 5;
        break;
      case 'd':
        z -= 5;
        break;

      case 'r':  // open/close the end effector
        grip += 5;
        break;
      case 'f':
        grip -= 5;
        break;   
        
      case 'h':  // move the robot to the reference configuration 
        x = 255; y = 0; z = 110; grip = 400;
        break;  
    }
    // print the modified x, y, z value
    Serial.print("x: "); Serial.print(x);
    Serial.print(", y: "); Serial.print(y);
    Serial.print(", z: "); Serial.println(z);


    /////////////// Compute the inverse kinematics of the robotic arm ///////////////
    ////// inverse kinematics (joint 1) //////
    t1 = atan2(y, x);                // [radians]
    d1 = t1*180/3.1415;   // [degrees]
    Serial.print("t1: "); Serial.println(d1);
    // map degrees to pulselength and send value to robotic arm
    pwm.setPWM(0, 0, map(d1,d0_min,d0_max,J0_min,J0_max));    

    ////// inverse kinematics (joint 2) //////
    t2 = atan2(z - 110, sqrt(sq(x) + sq(y))) + acos((sq(x) + sq(y) + sq(z) - 220*z + 625)/(210*sqrt(sq(x) + sq(y) + sq(z - 110))));                // [radians]
    d2 = t2*180/3.1415;   // [degrees]
    Serial.print("t2: "); Serial.println(d2);
    // map degrees to pulselength and send value to robotic arm
    pwm.setPWM(1, 0, map(d2,d1_min,d1_max,J1_min,J1_max));

    ////// inverse kinematics (joint 3) //////
    t3 = acosf(sq(x)/31500.0 + sq(y)/31500.0 + sq(z)/31500.0 - ((11.0*z)/1575.0) - (857.0/1260.0));     // [radians]
    d3 = t3*180.0/3.1415;   // [degrees]
    Serial.print("t3: "); Serial.println(t3);
    // map degrees to pulselength and send value to robotic arm
    pwm.setPWM(2, 0, map(d3,d2_min,d2_max,J2_min,J2_max));

    /////// inverse kinematics (joint 3) //////
    // send pulselength value to the end effector
    pwm.setPWM(3, 0, grip);
  }

void copy_array(int destination_index[], int source_array[], int source_array_length) {
    memcpy(destination_array, source_array, sizeof(source_array));
}

copy_array(char_index, two_index, (sizeof(two_index) / sizeof(two_index[0])))
  // Serial.println(points);
  
  for(int i = 0; i < 6; i++)
{
  Serial.println(char_index[i]);
}
  //Serial.println("two_index: "); Serial.println(two_index);
  Serial.println("char_index_size: ");  Serial.println(char_index_size);

// float char_point_setup(points[][], char_index[], char_index_size) {
//   float char_points[char_index_size][3];
//   for (int i{0}; i < char_index_size; i++) {
//     char_points[i][0] = points[char_index[i]][1];
//     char_points[i][1] = points[char_index[i]][2];
//     char_points[i][2] = points[char_index[i]][3];
//   }
//   return char_points;
// }


}
