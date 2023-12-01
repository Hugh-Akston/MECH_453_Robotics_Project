#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// Robotic arm parameters.
// These will need to be modifed based on your arm
////// Theta 1 //////
#define J0_min  105   // min pulselength
#define J0_max  465   // max pulselength
#define d0_min  -90   // coresponding limit in degrees (min)
#define d0_max  90    // max degrees

////// Theta 2 //////
#define J1_min  85    // min pulselength
#define J1_max  270   // max pulselength
#define d1_min  0     // coresponding limit in degrees (min)
#define d1_max  90    // max degrees

////// Theta 3 //////
#define J2_min  302   // min pulselength
#define J2_max  475   // max pulselength
#define d2_min  0     // coresponding limit in degrees (min)
#define d2_max  90    // max degrees

////// End Effector //////
#define J3_min  140   // pulselength in open position
#define J3_max  295   // pulselength in closed position
#define d3_min  0     // coresponding min distance in mm
#define d3_max  10    // max distance in mm

char incomingByte = 0; // for incoming serial data

// current position of the robitc arm
double x = 255; // mm
double y = 0;   // mm
double z = 110; //mm

double D1 = 110;
double L1 = 105;
double L3 = 150;

double start_time = 0;
double current_time = 0;
bool run_bool = false;
bool set_start_time = true;

////// Trajectory generation vars //////
double t1_init = 0;
double t2_init = 0;
double t3_init = 0;
double t1_final = 0;
double t2_final = 0;
double t3_final = 0;
double t_velocity_max = 0;
double tf = 0;
double tb = 0;

////// Reference Configuration of the robotic arm //////
double x_reference = 255; // mm
double y_reference = 0;   // mm
double z_reference = 110; // mm

////// Reference Configuration of the robotic arm in joint space //////
double t1_reference = 0; // degrees
double t2_reference = 0;   // degrees
double t3_reference = 0; // degrees

// variables used to calculate the inverse kinematics
double t1 = 0;
int d1 = 0;

double beta = 0;
double t2 = 0;
int d2 = 0;

double c3 = 0;
double s3 = 0;
double t3 = 0;
int d3 = 0;

int grip = 200;

void setup() {
  Serial.begin(115200);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

  // Move the robotic arm to the reference configuration (starting position)
  pwm.setPWM(0, 0, map(t1_reference,d0_min,d0_max,J0_min,J0_max));
  pwm.setPWM(1, 0, map(t2_reference,d1_min,d1_max,J1_min,J1_max));
  pwm.setPWM(2, 0, map(t3_reference,d2_min,d2_max,J2_min,J2_max));
  pwm.setPWM(3, 0, grip);  
}

void loop() {

  // send data to start or end the trajectory
  if (Serial.available() > 0) { 
    // read the incoming byte: 
    incomingByte = Serial.read();

    // say what you got: 
    Serial.println(incomingByte);

    // print the current x, y, z value
    switch (incomingByte) {
      case 's':  // start trajectory
        run_bool = true;
        set_start_time = true;
        break;
      case 'e': // end trajectory
        run_bool = false;
        break;
        
      case 'h':  // move the robot to the reference configuration 
        x = x_reference; y = y_reference; z = z_reference; grip = 200;
        break;  
    }
  }

  if (run_bool == true){
    if (set_start_time == true){
      start_time = millis();
      set_start_time = false;
    }

    current_time = millis()-start_time;
    if (current_time <= tb){
      d1 = ;
      d2 = ;
      d3 = ;
    }
    if ((current_time > tb) && (current_time <= tf-tb)){
      d1 = ;
      d2 = ;
      d3 = ;
    }
    if ((current_time > tf-tb) && (current_time <= tf)){
      d1 = ;
      d2 = ;
      d3 = ;
    }

    pwm.setPWM(0, 0, map(d1,d0_min,d0_max,J0_min,J0_max)); 
    pwm.setPWM(1, 0, map(d2,d1_min,d1_max,J1_min,J1_max));
    pwm.setPWM(2, 0, map(d3,d2_min,d2_max,J2_min,J2_max));
  } 

}
