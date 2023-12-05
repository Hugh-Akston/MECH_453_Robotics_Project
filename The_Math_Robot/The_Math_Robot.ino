// Written By: Dr. Eric Markvicka
// Modified By: Luke Freyhof
// Purpose: This file serves as a guideline for implementing inverse kinematics in the robot arm for MECH 453/853

#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define some functions to help find the sizes of arrays
#define ArrayHeight(array) (sizeof(array) / sizeof(array[0]))
#define ArrayWidth(array) (sizeof(array[0]) / sizeof(array[0][0]))
#define VectorLength(vector) (sizeof(vector) / sizeof(vector[0]))

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

float l1 = 110.0;
float l2 = 105.0;
float l3 = 150.0;

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

// float original_array[4][4];
// float new_array[4][4];
const int array_height = 5; //ArrayHeight(points);
const int array_width = 4; //ArrayWidth(points);
const int vector_length = 6; //VectorLength(two_index);

int grip = 400;

void setup() {
  Serial.begin(115200);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);

  // Move the robotic arm to the reference configuration
  // pwm.setPWM(0, 0, map(0,d0_min,d0_max,J0_min,J0_max));
  // pwm.setPWM(1, 0, map(0,d1_min,d1_max,J1_min,J1_max));
  // pwm.setPWM(2, 0, map(0,d2_min,d2_max,J2_min,J2_max));
  // pwm.setPWM(3, 0, grip);

  // float points_width = ArrayWidth(points);
  // float points_height = ArrayHeight(points);
  // float char_index_size = VectorLength(two_index);

  // Serial.print("Char_Index_Size: ");
  // Serial.println(char_index_size);
  // Serial.print("Point_Width: ");
  // Serial.println(points_width);
  // Serial.print("Point_Height: ");
  // Serial.println(points_height);

  int points_height = ArrayHeight(points);
  int points_width = ArrayWidth(points);
  int two_length = VectorLength(two_index);
  Serial.print("Point_Width: ");
  Serial.println(points_width);
  Serial.print("Point_Height: ");
  Serial.println(points_height);

  ////// This tests the array copy functionality //////
  // Create a new array to copy into
  float newArray[points_height][3];

  // Call the function to copy the array
  copyArray4to3(points, newArray, points_height);

  // Print the original array
  Serial.println("Original Array:");
  printArray4(points, points_height);

  delay(100);

  // Print the copied array
  Serial.println("Copied Array:");
  printArray3(newArray, points_height);
  ////////////////////////////////////////////////////

  delay(100);

  ////// This tests the vector copy functionality //////
  // Create a new array to copy into
  int newVector[two_length];

  // Call the function to copy the array
  copyVectorInt6(two_index, newVector);

  // Print the original array
  Serial.println("Original Vector:");
  printVectorInt(two_index, VectorLength(two_index));

  delay(100);

  // Print the copied array
  Serial.println("Copied Vector:");
  printVectorInt(newVector, VectorLength(newVector));
  //////////////////////////////////////////////////////

  ////// Create char_index vector //////
  int index_length = VectorLength(two_index);
  int char_index[index_length] = {0};
  copyVectorInt6(two_index, char_index);
  printVectorInt(char_index, index_length);
  //////////////////////////////////////

  numToPulselength(index_length, char_index, points);

  // ////// Char Point Setup //////
  // float char_points[index_length][3] = {0.0};
  // // // printArray3(char_points, index_length);
  // // for (int i = 0; i < index_length; ++i) {
  // //   // newVector[i] = originalVector[i];
  // //   char_points[i][0] = points[char_index[i] - 1][1];
  // //   Serial.println(" ");
  // //   Serial.println(char_index[i]);
  // //   Serial.println(points[char_index[i]][1]);
  // //   char_points[i][1] = points[char_index[i] - 1][2];
  // //   char_points[i][2] = points[char_index[i] - 1][3];
  // // }
  // // Serial.println("New char_points array: ");
  // // printArray3(char_points, index_length);
  // //////////////////////////////

  // // Test charPointSetup
  // charPointSetup(char_points, char_index, points, index_length);
  // //


  // ////// Waypoint Array Setup //////
  // int n_wp = 15;
  // int char_waypoints_height = index_length + (n_wp - 2)*(index_length - 1);
  // float char_waypoints[char_waypoints_height][3] = {0.0};
  // Serial.println(char_waypoints_height);
  // printArray3(char_waypoints, char_waypoints_height);
  // for (int i = 0; i < index_length; ++i) {
  //   // newVector[i] = originalVector[i];
  //   char_waypoints[i + (i)*(n_wp - 2)][0] = char_points[i][0];
  //   char_waypoints[i + (i)*(n_wp - 2)][1] = char_points[i][1];
  //   char_waypoints[i + (i)*(n_wp - 2)][2] = char_points[i][2];
  // }
  // printArray3(char_waypoints, char_waypoints_height);

  // Serial.println(char_waypoints[0][0]);
  // //////////////////////////////////

  // ////// Waypoint Array Assignment //////
  // float x_increment[index_length - 1] = {0.0};
  // float y_increment[index_length - 1] = {0.0};
  // float z_increment[index_length - 1] = {0.0};
  // for (int i = 0; i < (index_length - 1); ++i) {
  //   x_increment[i] = (char_points[i + 1][0] - char_points[i][0])/(n_wp - 1);
  //   y_increment[i] = (char_points[i + 1][1] - char_points[i][1])/(n_wp - 1);
  //   z_increment[i] = (char_points[i + 1][2] - char_points[i][2])/(n_wp - 1);
  // }
  // for (int i = 0; i < (index_length - 1); ++i) {
  //   for (int j = 0; j < (n_wp - 1); ++j) {
  //     char_waypoints[i + (i)*(n_wp - 2) + j][0] = char_points[i][0] + j*(x_increment[i]);
  //   }
  //   for (int j = 0; j < (n_wp - 1); ++j) {
  //     char_waypoints[i + (i)*(n_wp - 2) + j][1] = char_points[i][1] + j*(y_increment[i]);
  //   }
  //   for (int j = 0; j < (n_wp - 1); ++j) {
  //     char_waypoints[i + (i)*(n_wp - 2) + j][2] = char_points[i][2] + j*(z_increment[i]);
  //   }
  // }
  // printVector(x_increment, VectorLength(x_increment));
  // printArray3(char_waypoints, char_waypoints_height);
  // // Serial.println(char_points[0][0] + 1*(x_increment[0]));
  // // Serial.println(char_waypoints[1][0]);
  // ///////////////////////////////////////

  // ////// IK of char_waypoints //////
  // float x_values[char_waypoints_height] = {0.0};
  // float y_values[char_waypoints_height] = {0.0};
  // float z_values[char_waypoints_height] = {0.0};
  // float t3_values[char_waypoints_height] = {0.0};
  // float t2_values[char_waypoints_height] = {0.0};
  // float t1_values[char_waypoints_height] = {0.0};

  // for (int i = 0; i < char_waypoints_height; ++i) {
  //   x_values[i] = char_waypoints[i][0];
  // }
  // for (int i = 0; i < char_waypoints_height; ++i) {
  //   y_values[i] = char_waypoints[i][1];
  // }
  // for (int i = 0; i < char_waypoints_height; ++i) {
  //   z_values[i] = char_waypoints[i][2];
  // }
  
  // printVector(x_values, VectorLength(x_values));

  // for (int i = 0; i < char_waypoints_height; ++i) {
  //   t1_values[i] = atan2(y_values[i], x_values[i]);                // [radians]
  //   t1_values[i] = t1_values[i]*180/3.1415;   // [degrees];
  // }
  
  // printVector(t1_values, VectorLength(t1_values));

  // for (int i = 0; i < char_waypoints_height; ++i) {
  //   t2_values[i] = atan2(z_values[i] - 110, sqrt(sq(x_values[i]) + sq(y_values[i]))) + acos((sq(x_values[i]) + sq(y_values[i]) + sq(z_values[i]) - 220*z_values[i] + 625)/(210*sqrt(sq(x_values[i]) + sq(y_values[i]) + sq(z_values[i] - 110))));                // [radians]
  //   // t2_values[i] = atan2(z_values[i] - l1, sqrt(sq(x_values[i]) + sq(y_values[i])) + acos((sq(l1) - 2*l1*z_values[i] + sq(l2) - sq(l3) + sq(x_values[i]) + sq(y_values[i]) + sq(z_values[i]))/(2*l2*sqrt(sq(x_values[i]) + sq(y_values[i]) + sq(l1 - z_values[i]))))); // This is the inverse kinematics formula that Dr. M had in his path planning example code.
  //   t2_values[i] = t2_values[i]*180/3.1415;   // [degrees];
  // }

  // for (int i = 0; i < char_waypoints_height; ++i) {
  //   t3_values[i] = acosf(sq(x_values[i])/31500.0 + sq(y_values[i])/31500.0 + sq(z_values[i])/31500.0 - ((11.0*z_values[i])/1575.0) - (857.0/1260.0));                // [radians]
  //   t3_values[i] = t3_values[i]*180/3.1415;   // [degrees];
  // }
  // Serial.println("Theta Values: ");
  // printVector(t1_values, VectorLength(t1_values));
  // printVector(t2_values, VectorLength(t2_values));
  // printVector(t3_values, VectorLength(t3_values));

  // //////////////////////////////////

  // ////// Convert Angles to Pulselengths //////
  // float pl1_values[char_waypoints_height] = {0.0};
  // float pl2_values[char_waypoints_height] = {0.0};
  // float pl3_values[char_waypoints_height] = {0.0};
  // for (int i = 0; i < char_waypoints_height; ++i) {
  //   // pwm.setPWM(0, 0, map(d1,d0_min,d0_max,J0_min,J0_max));
  //   pl1_values[i] = map(t1_values[i],d0_min,d0_max,J0_min,J0_max);
  //   pl2_values[i] = map(t2_values[i],d1_min,d1_max,J1_min,J1_max);
  //   pl3_values[i] = map(t3_values[i],d2_min,d2_max,J2_min,J2_max);
  // }
  // Serial.println("Pulse Length Values: ");
  // printVector(pl1_values, VectorLength(pl1_values));
  // printVector(pl2_values, VectorLength(pl2_values));
  // printVector(pl3_values, VectorLength(pl3_values));
  
  // ////////////////////////////////////////////

  // ////// Move Robot Arm //////
  // pwm.setPWM(0, 0, pl1_values[0]);
  // pwm.setPWM(1, 0, pl2_values[0]);
  // pwm.setPWM(2, 0, pl3_values[0]);
  // delay(3000);

  // for (int i = 1; i < char_waypoints_height; ++i) {
  //   pwm.setPWM(0, 0, pl1_values[i]);
  //   pwm.setPWM(1, 0, pl2_values[i]);
  //   pwm.setPWM(2, 0, pl3_values[i]);
  //   delay(50);
  // }
  ////////////////////////////
  Serial.println(" ");
  Serial.println("Setup finished");
}


void loop() {

if (Serial.available() > 0) { 

    String input = Serial.readStringUntil('\n'); 
    input.trim(); // Remove leading and trailing whitespaces 

    // Evaluate the mathematical expression 
    double result = evaluateExpression(input); 

    // Print the original result 
    Serial.print("Original Result: "); 
    Serial.println(result);

    // Interpret the result into a vector 
    int numDigits = countDigits(result); 
    int* resultVector = new int[numDigits]; 
    interpretResult(result, resultVector, numDigits); 

    // Print the interpreted vector 
    Serial.println("Interpreted Result Vector:");
    for (int i = 0; i < numDigits; ++i) { 

      Serial.print(resultVector[i]); 
      Serial.print("\t"); 

    } 
    Serial.println();

    // Deallocate memory 

    delete[] resultVector; 
    Serial.println("Enter a mathematical expression:"); 

  } 
//   // send data only when you receive data: 
//   if (Serial.available() > 0) { 
//     // read the incoming byte: 
//     incomingByte = Serial.read();

//     // say what you got: 
//     Serial.println(incomingByte);

//     // print the current x, y, z value
//     Serial.print("x: "); Serial.print(x);
//     Serial.print(", y: "); Serial.print(y);
//     Serial.print(", z: "); Serial.println(z);
//     switch (incomingByte) {
//       case 'q':  // move in x
//         x += 5;
//         break;
//       case 'a':
//         x -= 5;
//         break;
        
//       case 'w':  // move in y
//         y += 5;
//         break;
//       case 's':
//         y -= 5;
//         break; 
        
//       case 'e':  // move in z
//         z += 5;
//         break;
//       case 'd':
//         z -= 5;
//         break;

//       case 'r':  // open/close the end effector
//         grip += 5;
//         break;
//       case 'f':
//         grip -= 5;
//         break;   
        
//       case 'h':  // move the robot to the reference configuration 
//         x = 255; y = 0; z = 110; grip = 400;
//         break;  
//     }
//     // print the modified x, y, z value
//     Serial.print("x: "); Serial.print(x);
//     Serial.print(", y: "); Serial.print(y);
//     Serial.print(", z: "); Serial.println(z);


//     /////////////// Compute the inverse kinematics of the robotic arm ///////////////
//     ////// inverse kinematics (joint 1) //////
//     t1 = atan2(y, x);                // [radians]
//     d1 = t1*180/3.1415;   // [degrees]
//     Serial.print("t1: "); Serial.println(d1);
//     // map degrees to pulselength and send value to robotic arm
//     pwm.setPWM(0, 0, map(d1,d0_min,d0_max,J0_min,J0_max));    

//     ////// inverse kinematics (joint 2) //////
//     t2 = atan2(z - 110, sqrt(sq(x) + sq(y))) + acos((sq(x) + sq(y) + sq(z) - 220*z + 625)/(210*sqrt(sq(x) + sq(y) + sq(z - 110))));                // [radians]
//     d2 = t2*180/3.1415;   // [degrees]
//     Serial.print("t2: "); Serial.println(d2);
//     // map degrees to pulselength and send value to robotic arm
//     pwm.setPWM(1, 0, map(d2,d1_min,d1_max,J1_min,J1_max));

//     ////// inverse kinematics (joint 3) //////
//     t3 = acosf(sq(x)/31500.0 + sq(y)/31500.0 + sq(z)/31500.0 - ((11.0*z)/1575.0) - (857.0/1260.0));     // [radians]
//     d3 = t3*180.0/3.1415;   // [degrees]
//     Serial.print("t3: "); Serial.println(t3);
//     // map degrees to pulselength and send value to robotic arm
//     pwm.setPWM(2, 0, map(d3,d2_min,d2_max,J2_min,J2_max));

//     /////// inverse kinematics (joint 3) //////
//     // send pulselength value to the end effector
//     pwm.setPWM(3, 0, grip);
//   }

// // void copy_array(int destination_index[], int source_array[], int source_array_length) {
// //     memcpy(destination_array, source_array, sizeof(source_array));
// // }

// // copy_array(char_index, two_index, (sizeof(two_index) / sizeof(two_index[0])))
// //   // Serial.println(points);
  
// //   for(int i = 0; i < 6; i++)
// // {
// //   Serial.println(char_index[i]);
// // }
// //   //Serial.println("two_index: "); Serial.println(two_index);
// //   Serial.println("char_index_size: ");  Serial.println(char_index_size);

// // float char_point_setup(points[][], char_index[], char_index_size) {
// //   float char_points[char_index_size][3];
// //   for (int i{0}; i < char_index_size; i++) {
// //     char_points[i][0] = points[char_index[i]][1];
// //     char_points[i][1] = points[char_index[i]][2];
// //     char_points[i][2] = points[char_index[i]][3];
// //   }
// //   return char_points;
// // }


}

void copyArray4(float originalArray[][4], float newArray[][4], int array_height) { // This copies an array that is 4 elements wide
  for (int i = 0; i < array_height; ++i) {
    for (int j = 0; j < 4; ++j) {
      newArray[i][j] = originalArray[i][j];
    }
  }
}

void copyArray4to3(float originalArray[][4], float newArray[][3], int array_height) { // This copies an array that is 4 elements wide
  for (int i = 0; i < array_height; ++i) {
    for (int j = 1; j < 4; ++j) {
      newArray[i][j - 1] = originalArray[i][j];
    }
  }
}

void copyVector(float originalVector[], float newVector[]) { // This function decides which copyVector function to use based on the length of the input vector.
  // switch // Use a switch case statement to determine which copyVector function to use based on the length of the input vector.
  // case
  // case
}

////// Copy Vector Float //////

void copyVector9(float originalVector[9], float newVector[9]) { // This copies a vector that is 9 elements long
  for (int i = 0; i < 9; ++i) {
    newVector[i] = originalVector[i];
  }
}

void copyVector8(float originalVector[8], float newVector[8]) { // This copies a vector that is 8 elements long
  for (int i = 0; i < 8; ++i) {
    newVector[i] = originalVector[i];
  }
}

void copyVector7(float originalVector[7], float newVector[7]) { // This copies a vector that is 7 elements long
  for (int i = 0; i < 7; ++i) {
    newVector[i] = originalVector[i];
  }
}

void copyVector6(float originalVector[6], float newVector[6]) { // This copies a vector that is 6 elements long
  for (int i = 0; i < 6; ++i) {
    newVector[i] = originalVector[i];
  }
}

void copyVector5(float originalVector[5], float newVector[5]) { // This copies a vector that is 5 elements long
  for (int i = 0; i < 5; ++i) {
    newVector[i] = originalVector[i];
  }
}

void copyVector4(float originalVector[4], float newVector[4]) { // This copies a vector that is 4 elements long
  for (int i = 0; i < 4; ++i) {
    newVector[i] = originalVector[i];
  }
}

void copyVector3(float originalVector[3], float newVector[3]) { // This copies a vector that is 3 elements long
  for (int i = 0; i < 3; ++i) {
    newVector[i] = originalVector[i];
  }
}

void copyVector2(float originalVector[2], float newVector[2]) { // This copies a vector that is 2 elements long
  for (int i = 0; i < 2; ++i) {
    newVector[i] = originalVector[i];
  }
}

////////////////////////////////////////////

////// Copy Vector Int ///////

void copyVectorInt9(int originalVector[9], int newVector[9]) { // This copies a vector that is 9 elements long
  for (int i = 0; i < 9; ++i) {
    newVector[i] = originalVector[i];
  }
}

void copyVectorInt8(int originalVector[8], int newVector[8]) { // This copies a vector that is 8 elements long
  for (int i = 0; i < 8; ++i) {
    newVector[i] = originalVector[i];
  }
}

void copyVectorInt7(int originalVector[7], int newVector[7]) { // This copies a vector that is 7 elements long
  for (int i = 0; i < 7; ++i) {
    newVector[i] = originalVector[i];
  }
}

void copyVectorInt6(int originalVector[6], int newVector[6]) { // This copies a vector that is 6 elements long
  for (int i = 0; i < 6; ++i) {
    newVector[i] = originalVector[i];
  }
}

void copyVectorInt5(int originalVector[5], int newVector[5]) { // This copies a vector that is 5 elements long
  for (int i = 0; i < 5; ++i) {
    newVector[i] = originalVector[i];
  }
}

void copyVectorInt4(int originalVector[4], int newVector[4]) { // This copies a vector that is 4 elements long
  for (int i = 0; i < 4; ++i) {
    newVector[i] = originalVector[i];
  }
}

void copyVectorInt3(int originalVector[3], int newVector[3]) { // This copies a vector that is 3 elements long
  for (int i = 0; i < 3; ++i) {
    newVector[i] = originalVector[i];
  }
}

void copyVectorInt2(int originalVector[2], int newVector[2]) { // This copies a vector that is 2 elements long
  for (int i = 0; i < 2; ++i) {
    newVector[i] = originalVector[i];
  }
}

//////////////////////////////////////////

void printVector(float vector[], int size) {
  for (int i = 0; i < size; ++i) {
    Serial.print(vector[i]);
    Serial.print("\t");
  }
  Serial.println();
}

void printVectorInt(int vector[], int size) {
  for (int i = 0; i < size; ++i) {
    Serial.print(vector[i]);
    Serial.print("\t");
  }
  Serial.println();
}

void printArray4(float array[][4], int array_height) {
  for (int i = 0; i < array_height; ++i) {
    for (int j = 0; j < 4; ++j) {
      Serial.print(array[i][j]);
      Serial.print("\t");
    }
    Serial.println();
  }
  Serial.println();
}

void printArray3(float array[][3], int array_height) {
  for (int i = 0; i < array_height; ++i) {
    for (int j = 0; j < 3; ++j) {
      Serial.print(array[i][j]);
      Serial.print("\t");
    }
    Serial.println();
  }
  Serial.println();
}

  // ////// Create char_index vector //////
  // int index_length = VectorLength(two_index);
  // int char_index[index_length] = {0};
  // copyVectorInt6(two_index, char_index);
  // printVectorInt(char_index, index_length);
  // //////////////////////////////////////

////// numToPulselength //////
void numToPulselength(int index_length, int char_index[], float points[][4]){
    ////// Char Point Setup //////
  float char_points[index_length][3] = {0.0};
  // printArray3(char_points, index_length);
  for (int i = 0; i < index_length; ++i) {
    // newVector[i] = originalVector[i];
    char_points[i][0] = points[char_index[i] - 1][1];
    Serial.println(" ");
    Serial.println(char_index[i]);
    Serial.println(points[char_index[i]][1]);
    char_points[i][1] = points[char_index[i] - 1][2];
    char_points[i][2] = points[char_index[i] - 1][3];
  }
  Serial.println("New char_points array: ");
  printArray3(char_points, index_length);
  ////////////////////////////

  ////// Waypoint Array Setup //////
  int n_wp = 15;
  int char_waypoints_height = index_length + (n_wp - 2)*(index_length - 1);
  float char_waypoints[char_waypoints_height][3] = {0.0};
  Serial.println(char_waypoints_height);
  printArray3(char_waypoints, char_waypoints_height);
  for (int i = 0; i < index_length; ++i) {
    // newVector[i] = originalVector[i];
    char_waypoints[i + (i)*(n_wp - 2)][0] = char_points[i][0];
    char_waypoints[i + (i)*(n_wp - 2)][1] = char_points[i][1];
    char_waypoints[i + (i)*(n_wp - 2)][2] = char_points[i][2];
  }
  printArray3(char_waypoints, char_waypoints_height);

  Serial.println(char_waypoints[0][0]);
  //////////////////////////////////

  ////// Waypoint Array Assignment //////
  float x_increment[index_length - 1] = {0.0};
  float y_increment[index_length - 1] = {0.0};
  float z_increment[index_length - 1] = {0.0};
  for (int i = 0; i < (index_length - 1); ++i) {
    x_increment[i] = (char_points[i + 1][0] - char_points[i][0])/(n_wp - 1);
    y_increment[i] = (char_points[i + 1][1] - char_points[i][1])/(n_wp - 1);
    z_increment[i] = (char_points[i + 1][2] - char_points[i][2])/(n_wp - 1);
  }
  for (int i = 0; i < (index_length - 1); ++i) {
    for (int j = 0; j < (n_wp - 1); ++j) {
      char_waypoints[i + (i)*(n_wp - 2) + j][0] = char_points[i][0] + j*(x_increment[i]);
    }
    for (int j = 0; j < (n_wp - 1); ++j) {
      char_waypoints[i + (i)*(n_wp - 2) + j][1] = char_points[i][1] + j*(y_increment[i]);
    }
    for (int j = 0; j < (n_wp - 1); ++j) {
      char_waypoints[i + (i)*(n_wp - 2) + j][2] = char_points[i][2] + j*(z_increment[i]);
    }
  }
  printVector(x_increment, VectorLength(x_increment));
  printArray3(char_waypoints, char_waypoints_height);
  // Serial.println(char_points[0][0] + 1*(x_increment[0]));
  // Serial.println(char_waypoints[1][0]);
  ///////////////////////////////////////

  ////// IK of char_waypoints //////
  float x_values[char_waypoints_height] = {0.0};
  float y_values[char_waypoints_height] = {0.0};
  float z_values[char_waypoints_height] = {0.0};
  float t3_values[char_waypoints_height] = {0.0};
  float t2_values[char_waypoints_height] = {0.0};
  float t1_values[char_waypoints_height] = {0.0};

  for (int i = 0; i < char_waypoints_height; ++i) {
    x_values[i] = char_waypoints[i][0];
  }
  for (int i = 0; i < char_waypoints_height; ++i) {
    y_values[i] = char_waypoints[i][1];
  }
  for (int i = 0; i < char_waypoints_height; ++i) {
    z_values[i] = char_waypoints[i][2];
  }
  
  printVector(x_values, VectorLength(x_values));

  for (int i = 0; i < char_waypoints_height; ++i) {
    t1_values[i] = atan2(y_values[i], x_values[i]);                // [radians]
    t1_values[i] = t1_values[i]*180/3.1415;   // [degrees];
  }
  
  printVector(t1_values, VectorLength(t1_values));

  for (int i = 0; i < char_waypoints_height; ++i) {
    t2_values[i] = atan2(z_values[i] - 110, sqrt(sq(x_values[i]) + sq(y_values[i]))) + acos((sq(x_values[i]) + sq(y_values[i]) + sq(z_values[i]) - 220*z_values[i] + 625)/(210*sqrt(sq(x_values[i]) + sq(y_values[i]) + sq(z_values[i] - 110))));                // [radians]
    // t2_values[i] = atan2(z_values[i] - l1, sqrt(sq(x_values[i]) + sq(y_values[i])) + acos((sq(l1) - 2*l1*z_values[i] + sq(l2) - sq(l3) + sq(x_values[i]) + sq(y_values[i]) + sq(z_values[i]))/(2*l2*sqrt(sq(x_values[i]) + sq(y_values[i]) + sq(l1 - z_values[i]))))); // This is the inverse kinematics formula that Dr. M had in his path planning example code.
    t2_values[i] = t2_values[i]*180/3.1415;   // [degrees];
  }

  for (int i = 0; i < char_waypoints_height; ++i) {
    t3_values[i] = acosf(sq(x_values[i])/31500.0 + sq(y_values[i])/31500.0 + sq(z_values[i])/31500.0 - ((11.0*z_values[i])/1575.0) - (857.0/1260.0));                // [radians]
    t3_values[i] = t3_values[i]*180/3.1415;   // [degrees];
  }
  Serial.println("Theta Values: ");
  printVector(t1_values, VectorLength(t1_values));
  printVector(t2_values, VectorLength(t2_values));
  printVector(t3_values, VectorLength(t3_values));

  //////////////////////////////////

  ////// Convert Angles to Pulselengths //////
  float pl1_values[char_waypoints_height] = {0.0};
  float pl2_values[char_waypoints_height] = {0.0};
  float pl3_values[char_waypoints_height] = {0.0};
  for (int i = 0; i < char_waypoints_height; ++i) {
    // pwm.setPWM(0, 0, map(d1,d0_min,d0_max,J0_min,J0_max));
    pl1_values[i] = map(t1_values[i],d0_min,d0_max,J0_min,J0_max);
    pl2_values[i] = map(t2_values[i],d1_min,d1_max,J1_min,J1_max);
    pl3_values[i] = map(t3_values[i],d2_min,d2_max,J2_min,J2_max);
  }
  Serial.println("Pulse Length Values: ");
  printVector(pl1_values, VectorLength(pl1_values));
  printVector(pl2_values, VectorLength(pl2_values));
  printVector(pl3_values, VectorLength(pl3_values));
  
  ////////////////////////////////////////////
}
//////////////////////////////

double evaluateExpression(String expression) { 
  // Replace 'x' with '*' for multiplication 
  expression.replace("x", "*"); 

  // Evaluate the expression 
  int index = 0; 
  double result = parseTerm(expression, index); 
  while (index < expression.length()) { 

    char op = expression.charAt(index); 
    index++; 
    double operand = parseTerm(expression, index); 

    // Perform the operation 
    if (op == '+') { 
      result += operand; 
    } else if (op == '-') { 
      result -= operand; 
    } else { 
      Serial.println("Invalid operator");
      return 0; 
    } 

  } 
  return result; 
}

double parseTerm(String expression, int &index) { 
  double result = parseFactor(expression, index);
  while (index < expression.length()) { 
    char op = expression.charAt(index); 
    if (op == '*' || op == '/') { 
      index++; 
      double operand = parseFactor(expression, index); 
      // Perform the operation 
      if (op == '*') { 
        result *= operand; 
      } else if (op == '/') { 
        if (operand != 0) { 
          result /= operand; 
        } else { 
          Serial.println("Division by zero"); 
          return 0; 
        } 

      } 

    } else { 

      break; 
    } 

  }
  return result; 
}

double parseFactor(String expression, int &index) { 
  double result = 0; 
  // Check for parentheses 
  if (expression.charAt(index) == '(') { 
    index++; 
    result = evaluateExpression(expression); 
    index++; // Skip the closing parenthesis 
  } else { 

    // Read the number 
    result = expression.substring(index).toFloat(); 
    while (index < expression.length() && 
           (isdigit(expression.charAt(index)) || expression.charAt(index) == '.')) { 
      index++; 
    } 

  }
  return result; 
}

int countDigits(double num) { 
  // Count the number of digits in the number 
  int count = 0; 
  long long intPart = static_cast<long long>(num); 
  while (intPart != 0) { 
    intPart /= 10; 
    count++; 
  } 
  return count; 

}

void interpretResult(double result, int resultVector[], int numDigits) { 
  // Convert the double result to a string 
  String resultString = String(result, 10); 

  // Initialize the result vector 
  for (int i = 0; i < numDigits; ++i) { 
    resultVector[i] = 0; 
  } 

  // Interpret the result into the vector 
  int vecIndex = 0; 
  for (int i = 0; i < resultString.length() && vecIndex < numDigits; ++i) { 
    if (isdigit(resultString.charAt(i))) { 
      resultVector[vecIndex] = resultString.charAt(i) - '0'; 
      vecIndex++; 
    } 

  }
}