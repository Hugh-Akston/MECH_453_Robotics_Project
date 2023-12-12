// Inspired By Code From: Dr. Eric Markvicka
// Written/Modified By: Luke Freyhof, Elliott Rankin, and Christian Dahlman
// The Math Robot
// December 11, 2023
// MECH 453/853

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
#define J0_min  100   // min pulselength
#define J0_max  476   // max pulselength
#define d0_min  -90   // corresponding limit in degrees (min)
#define d0_max  90    // max degrees

////// Theta 2 //////
#define J1_min  124    // min pulselength
#define J1_max  480   // max pulselength
#define d1_min  0     // corresponding limit in degrees (min)
#define d1_max  180    // max degrees

////// Theta 3 //////
#define J2_min  104   // min pulselength
#define J2_max  448   // max pulselength
#define d2_min  -110     // corresponding limit in degrees (min)
#define d2_max  110    // max degrees

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
int zero_index[] = {5, 6, 7, 10, 9, 8, 5};
int one_index[] = {8, 9, 10};
int two_index[] = {5, 8, 9, 6, 7, 10};
int three_index[] = {5, 8, 9, 6, 9, 10, 7};
int four_index[] = {5, 6, 9, 8, 9, 10};
int five_index[] = {8, 5, 6, 9, 10, 7};
int six_index[] = {8, 5, 6, 7, 10, 9, 6}; // Points index for 6
int seven_index[] = {5, 8, 9, 10};
int eight_index[] = {5, 6, 7, 10, 9, 8, 5, 6, 9};
int nine_index[] = {9, 6, 5, 8, 9, 10};
int decimal_index[] = {11, 12, 14, 13};

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

  Serial.println(" ");
  Serial.println("Setup finished");
}


void loop() {

if (Serial.available() > 0) { 

    String input = Serial.readStringUntil('\n'); 
    input.trim(); // Remove leading and trailing whitespaces 
    String inputCheck = "9+10";

    Serial.print("Original Input:");
    Serial.println(input);

    bool joke = false;

    if (input.compareTo(inputCheck) == 0) {
         joke = true;
         Serial.println("I hope you like jokes ;)");
         float shift_distance = 30.0;
         float x_shift = 0.0;
         Serial.print("x_shift");
         Serial.println(x_shift);

         Serial.println("The number is 2");
         ////// Create char_index vector //////
         int index_length = VectorLength(two_index);
         int char_index[index_length] = {0};
         copyVectorInt6(two_index, char_index);
         ////// Draw the number //////
         numToPulselength(index_length, char_index, points, x_shift);

         Serial.println("The number is 1");
         x_shift = shift_distance;
         ////// Create char_index vector //////
         index_length = VectorLength(one_index);
         char_index[index_length] = {0};
         copyVectorInt3(one_index, char_index);
         ////// Draw the number //////
         numToPulselength(index_length, char_index, points, x_shift);

         input = ' ';
    }

    // Evaluate the mathematical expression 
    double result = evaluateExpression(input); 

    // Print the original result 
    Serial.print("Original Result: "); 
    Serial.println(result);

    // Interpret the result into a vector 
    int numDigits = 3; //countDigits(result); 
    Serial.println(numDigits);
    int* resultVector = new int[numDigits]; 
    interpretResult(result, resultVector, numDigits); 

    // Print the interpreted vector 
    Serial.println("Interpreted Result Vector:");
    for (int i = 0; i < numDigits; ++i) { 

      Serial.print(resultVector[i]); 
      Serial.print("\t"); 

    } 
    Serial.println();

    char charVector[numDigits];
    floatToVector(result, charVector, numDigits);
    Serial.print("Converted vector: ");
    Serial.println(charVector);
    Serial.print("First Element of Converted vector: ");
    Serial.println(charVector[0]);

    for (int i = 0; i < numDigits; ++i) { 
      float shift_distance = 30.0;
      float x_shift = shift_distance*i;
      Serial.print("x_shift");
      Serial.println(x_shift);
      switch (charVector[i]) { // This switch/case structure decides which decides which symbols to draw.
        
        case '+':
        if (joke == true){
          break;
        }
        {Serial.println("+ sign");
        ////// Create char_index vector //////
        int index_length = VectorLength(plus_index);
        int char_index[index_length] = {0};
        copyVectorInt5(plus_index, char_index);
        ////// Draw the number //////
        numToPulselength(index_length, char_index, points, x_shift);
        break;}

        case '-':
        if (joke == true){
          break;
        }
        {Serial.println("- sign");
        ////// Create char_index vector //////
        int index_length = VectorLength(minus_index);
        int char_index[index_length] = {0};
        copyVectorInt2(minus_index, char_index);
        ////// Draw the number //////
        numToPulselength(index_length, char_index, points, x_shift);
        break;}

        case '.':
        if (joke == true){
          break;
        }
        {Serial.println(".");
        ////// Create char_index vector //////
        int index_length = VectorLength(decimal_index);
        int char_index[index_length] = {0};
        copyVectorInt4(decimal_index, char_index);
        ////// Draw the number //////
        numToPulselength(index_length, char_index, points, x_shift);
        break;}

        case '0':
        if (joke == true){
          break;
        }
        {Serial.println("The number is 0");
        ////// Create char_index vector //////
        int index_length = VectorLength(zero_index);
        int char_index[index_length] = {0};
        copyVectorInt6(zero_index, char_index);
        ////// Draw the number //////
        numToPulselength(index_length, char_index, points, x_shift);
        break;}

        case '1':
        if (joke == true){
          break;
        }
        {Serial.println("The number is 1");
        ////// Create char_index vector //////
        int index_length = VectorLength(one_index);
        int char_index[index_length] = {0};
        copyVectorInt3(one_index, char_index);
        ////// Draw the number //////
        numToPulselength(index_length, char_index, points, x_shift);
        break;}

        case '2':
        if (joke == true){
          break;
        }
        {Serial.println("The number is 2");
        ////// Create char_index vector //////
        int index_length = VectorLength(two_index);
        int char_index[index_length] = {0};
        copyVectorInt6(two_index, char_index);
        ////// Draw the number //////
        numToPulselength(index_length, char_index, points, x_shift);
        break;}

        case '3':
        if (joke == true){
          break;
        }
        {Serial.println("The number is 3");
        ////// Create char_index vector //////
        int index_length = VectorLength(three_index);
        int char_index[index_length] = {0};
        copyVectorInt7(three_index, char_index);
        ////// Draw the number //////
        numToPulselength(index_length, char_index, points, x_shift);
        break;}

        case '4':
        if (joke == true){
          break;
        }
        {Serial.println("The number is 4");
        ////// Create char_index vector //////
        int index_length = VectorLength(four_index);
        int char_index[index_length] = {0};
        copyVectorInt6(four_index, char_index);
        ////// Draw the number //////
        numToPulselength(index_length, char_index, points, x_shift);
        break;}

        case '5':
        if (joke == true){
          break;
        }
        {Serial.println("The number is 5");
        ////// Create char_index vector //////
        int index_length = VectorLength(five_index);
        int char_index[index_length] = {0};
        copyVectorInt6(five_index, char_index);
        ////// Draw the number //////
        numToPulselength(index_length, char_index, points, x_shift);
        break;}

        case '6':
        if (joke == true){
          break;
        }
        {Serial.println("The number is 6");
        ////// Create char_index vector //////
        int index_length = VectorLength(six_index);
        int char_index[index_length] = {0};
        copyVectorInt7(six_index, char_index);
        ////// Draw the number //////
        numToPulselength(index_length, char_index, points, x_shift);
        break;}

        case '7':
        if (joke == true){
          break;
        }
        {Serial.println("The number is 7");
        ////// Create char_index vector //////
        int index_length = VectorLength(seven_index);
        int char_index[index_length] = {0};
        copyVectorInt4(seven_index, char_index);
        ////// Draw the number //////
        numToPulselength(index_length, char_index, points, x_shift);
        break;}

        case '8':
        if (joke == true){
          break;
        }
        {Serial.println("The number is 8");
        ////// Create char_index vector //////
        int index_length = VectorLength(eight_index);
        int char_index[index_length] = {0};
        copyVectorInt8(eight_index, char_index);
        ////// Draw the number //////
        numToPulselength(index_length, char_index, points, x_shift);
        break;}

        case '9':
        if (joke == true){
          break;
        }
        {Serial.println("The number is 9");
        ////// Create char_index vector //////
        int index_length = VectorLength(nine_index);
        int char_index[index_length] = {0};
        copyVectorInt6(nine_index, char_index);
        ////// Draw the number //////
        numToPulselength(index_length, char_index, points, x_shift);
        break;}

      }
    }

    // Deallocate memory 

    delete[] resultVector; 
    Serial.println("Enter a mathematical expression:"); 

  }

}

////// Copy Array Functions //////
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
//////////////////////////////////

////// Copy Vector Functions //////
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
///////////////////////////////////

////// Print Vector and Array Functions //////
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
//////////////////////////////////////////////

////// Functions for drawing numbers and working with the mathematical expressions //////
////// numToPulselength //////
void numToPulselength(int index_length, int char_index[], float points[][4], float x_shift){
    ////// Char Point Setup //////
  float char_points[index_length][3] = {0.0};
  // printArray3(char_points, index_length);
  for (int i = 0; i < index_length; ++i) {
    // newVector[i] = originalVector[i];
    char_points[i][0] = points[char_index[i] - 1][1] + 60; // Added 40 to shift all the numbers to the right.
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

  Serial.println("Before Shift:");
  printArray3(char_waypoints, char_waypoints_height);

  for (int i = 0; i < char_waypoints_height; ++i) {
    char_waypoints[i][0] = char_waypoints[i][0] - x_shift;
  }
  printVector(x_increment, VectorLength(x_increment));
  Serial.println("After Shift:");
  printArray3(char_waypoints, char_waypoints_height);

  int off_board = 0;
  if (char_waypoints[0][0] > 0) {
    off_board = off_board + 40;
    Serial.println("off_board = 40");
  }
  else if (char_waypoints[0][0] < 0) {
    off_board = off_board - 40;
    Serial.println("off_board = -40");
  }

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
  printVector(y_values, VectorLength(y_values));
  printVector(z_values, VectorLength(z_values));

  for (int i = 0; i < char_waypoints_height; ++i) {
    t1_values[i] = atan2(y_values[i], x_values[i]);                // [radians]
    t1_values[i] = t1_values[i]*180/3.1415;   // [degrees];
  }
  Serial.println("Theta 1 Values");
  printVector(t1_values, VectorLength(t1_values));

  for (int i = 0; i < char_waypoints_height; ++i) {
    t2_values[i] = atan2(z_values[i] - 110, sqrt(sq(x_values[i]) + sq(y_values[i]))) + acos((sq(x_values[i]) + sq(y_values[i]) + sq(z_values[i]) - 220*z_values[i] + 625)/(210*sqrt(sq(x_values[i]) + sq(y_values[i]) + sq(z_values[i] - 110))));                // [radians]
    // t2_values[i] = atan2(z_values[i] - l1, sqrt(sq(x_values[i]) + sq(y_values[i])) + acos((sq(l1) - 2*l1*z_values[i] + sq(l2) - sq(l3) + sq(x_values[i]) + sq(y_values[i]) + sq(z_values[i]))/(2*l2*sqrt(sq(x_values[i]) + sq(y_values[i]) + sq(l1 - z_values[i]))))); // This is the inverse kinematics formula that Dr. M had in his path planning example code.
    t2_values[i] = t2_values[i]*180/3.1415;   // [degrees];
  }
  Serial.println("Theta 2 Values");
  printVector(t2_values, VectorLength(t2_values));

  for (int i = 0; i < char_waypoints_height; ++i) {
    t3_values[i] = acosf(sq(x_values[i])/31500.0 + sq(y_values[i])/31500.0 + sq(z_values[i])/31500.0 - ((11.0*z_values[i])/1575.0) - (857.0/1260.0));                // [radians]
    t3_values[i] = t3_values[i]*180/3.1415;   // [degrees];
  }
  Serial.println("Theta 3 Values");
  printVector(t3_values, VectorLength(t3_values));

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

  ////// Move Robot Arm //////
  Serial.println("Shift off board:");
  int shift_off_board = off_board + 288;
  Serial.println(shift_off_board);
  pwm.setPWM(0, 0, 288 + off_board); // The 40 is added to allow the robot arm to move away from the whiteboard.
  pwm.setPWM(1, 0, pl2_values[0]);
  pwm.setPWM(2, 0, pl3_values[0]);
  delay(3000);

  for (int i = 1; i < char_waypoints_height; ++i) {
    pwm.setPWM(0, 0, 288); 
    pwm.setPWM(1, 0, pl2_values[i]);
    pwm.setPWM(2, 0, pl3_values[i]);
    delay(100);
  }
  pwm.setPWM(0, 0, 288 + off_board); // The 40 is added to allow the robot arm to move away from the whiteboard.
  //////////////////////////
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
      return ;//0; 
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

void floatToVector(float num, char charVector[], int numDigits) {
  // Convert the floating-point number to a string
  String strNum = String(num, 6);  // Adjust the precision as needed

  // Copy the characters into the charVector
  strNum.toCharArray(charVector, numDigits);

  // Uncomment the following lines if you want to print the result
  Serial.print("Original float: ");
  Serial.println(num, 6);
  Serial.print("Converted vector: ");
  Serial.println(charVector);
}
/////////////////////////////////////////////////////////////////////////////////////////