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
float x = 255; // mm
float y = 0;   // mm
float z = 110; //mm

// variables used to calculate the inverse kinematics
float t1 = 0;
float d1 = 0;

float t2 = 0;
float d2 = 0;

float t3 = 0;
float d3 = 0;
float t3cos = 0;

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

}
