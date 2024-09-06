/**
 ******************************************************************************
 * @file   main.cpp
 * @author Jakob Frabosilio, jakob [dot] frabosilio [at] gmail [dot] com
 * @brief  Main program for running the Fo-SHIP.
 * 
 * This program is the main code for running my thesis's ground-truth
 * positioning system, the Four-Stacked Hexapod Isometric Positioner, or
 * Fo-SHIP for short (pronounced faux-ship). This code mainly builds off of
 * Nicolas Jeanmonod's implementation of a single hexapod platform controller.
 * I've added some custom functions (like being able to move slowly from the
 * current position to a new position, and calculating the position of each of
 * the platforms), but his code is the backbone of this implementation. You can
 * find his GitHub and website at the links below.
 * 
 * @see https://github.com/NicHub/stewart-platform-esp32/tree/master?tab=readme-ov-file
 * @see https://ouilogique.com/plateforme-de-stewart-esp32/
 * 
 * The main.cpp file uses a few helper files and functions. 
 * 
 * For the configuration of the hexapod platforms (where you specify the 
 * parameters of the physical model like linkage length, offsets for all the 
 * servos, etc.), look at Hexapod_Config_1.h.
 *
 * For some functions that can be used for demonstrations (like moving the
 * platforms in a circle, or testing the full range of motion), check out
 * Hexapod_Demo.h and Hexapod_Demo.cpp.
 * 
 * For helper functions related to the inverse kinematics solver and definitions
 * of custom structs like angle_t and platform_t, look at Hexapod_Kinematics.h
 * and Hexapod_Kinematics.cpp.
 * 
 * For the three algorithms used to calculate the inverse kinematics for a
 * single platform, check out Hexapod_KinematicsCalcServoAnglesAlgo1.h,
 * Hexapod_KinematicsCalcServoAnglesAlgo2.h, and
 * Hexapod_KinematicsCalcServoAnglesAlgo3.h. This implementation uses Algo3, as
 * the other two algorithms were giving me some weird issues.
 * 
 * For functions related to the servo motors and getting them to set to the
 * correct angles given a PWM value, see Hexapod_Servo.h and Hexapod_Servo.cpp.
 * 
 * You can find my master's thesis here:
 * @see https://youtube.com/watch?v=dQw4w9WgXcQ
 * 
 * (placeholder until I get it published)
 * 
 ******************************************************************************
*/

// External libs
#include <Arduino.h>
#include <PCA9685.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Hexapod libs
#include <Hexapod_Kinematics.h>
#include <Hexapod_Servo.h>
#include <Hexapod_Demo.h>

// -------------------------------- DEFINTIONS ---------------------------------
// macro used for counting the number of rows in a platform_t struct
#define COUNT_OF(x) ((sizeof(x) / sizeof(0 [x])) / ((size_t)(!(sizeof(x) % sizeof(0 [x])))))

// GPIO pins connected to AD0 pins of each IMU
#define IMU0 16
#define IMU1 4
#define IMU2 18
#define IMU3 2
#define IMU4 15

// GPIO pin for button and onboard LED
#define BUTTON 36
#define LED 2

// software Serial pins for communicating with STM32
#define TX 12
#define RX 14

// -------------------------------- VARIABLES ---------------------------------
// characters used for message verification with STM32
const char START_MARKER = '<';
const char END_MARKER = '>';
const int CHECKSUM_MODULUS = 256;

// for storing accelerometer data from each IMU
float IMU0_AccelData[] = {0, 0, 0};
float IMU1_AccelData[] = {0, 0, 0};
float IMU2_AccelData[] = {0, 0, 0};
float IMU3_AccelData[] = {0, 0, 0};
float IMU4_AccelData[] = {0, 0, 0};

// for storing the current orientation of each IMU
float IMU0_Angles[] = {0, 0, 0};
float IMU1_Angles[] = {0, 0, 0};
float IMU2_Angles[] = {0, 0, 0};
float IMU3_Angles[] = {0, 0, 0};
float IMU4_Angles[] = {0, 0, 0};

// for storing the rotation matrix defined by the current orientation of each IMU
float IMU0_RotMat[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
float IMU1_RotMat[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
float IMU2_RotMat[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
float IMU3_RotMat[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
float IMU4_RotMat[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

// for storing the transposed rotation matrices for each IMU (used to compute relative rotmats) 
float IMU0trans_RotMat[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
float IMU1trans_RotMat[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
float IMU2trans_RotMat[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
float IMU3trans_RotMat[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

// for storing the rotation matrices defining relative rotation between IMUs
float IMU1rel0_RotMat[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
float IMU2rel1_RotMat[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
float IMU3rel2_RotMat[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
float IMU4rel3_RotMat[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

// tracks the current position of the platform (X, Y, Z, A, B, C) (mm for XYZ, rad for ABC)
platform_t currentPos = {0, 0, 0, 0, 0, 0};

// tracks the true position of the center of the iSBL array (for verifying acoustic position estimate)
float true_x, true_y, true_z;

// setpoints for startup sequence
platform_t home_coords[] = {0, 0, 0, 0, 0, 0};
platform_t bow_coords[] = {0, 0, 0, 0, -radians(15), 0};

// set to true to print accelerometer / orientation data from each IMU
bool printMPU = false;

// for setting the angle for each servo on each platform - 0 is bottom, 3 is top
angle_t servo_angles0[6];
angle_t servo_angles1[6];
angle_t servo_angles2[6];
angle_t servo_angles3[6];

// for controlling the platforms
Hexapod_Servo hx_servo;
Hexapod_Demo hx_demo;

// for initializing the IMUs
Adafruit_MPU6050 mpu;
const unsigned short fifoRate = 20U; // IMU refresh rate in Hz.
HardwareSerial SerialPort(1);  // using UART1

// for tracking which axes to move about
bool translational_axes = false;
const char* axes[3]; // Declare as an array of pointers to const char


// -------------------------------- FUNCTION DECLARATIONS ---------------------------------
// for reading from IMU and converting to orientation
void initMPU(int IMU_NUM);
void readMPU(int IMU_NUM, int num_avgs=5);
void getEulerFromMPU(float *IMU_AccelData, float *IMU_Angles);
void getRotMatFromEuler(float *IMU_Angles, float *IMU_RotMat);
void getEulerFromRotMat(float *IMU_RotMat, float *IMU_Angles);

// matrix math helper functions
void multiply3x3Matrices(float *matA, float *matB, float *result);
void transposeMatrix(float *mat, float *result);
void multiply3x3MatrixVector(float *mat, float *vec, float *result);

// for getting the true position of the end effector / center of iSBL array
void getEndEffectorCoords(platform_t *coords1, platform_t *coords2, platform_t *coords3, platform_t *coords4, bool USE_IMU=false);

// for moving the platform
void moveSlowly(platform_t *startCoords, platform_t *endCoords, uint16_t num_steps, uint8_t which_platforms=15);

// for communication with STM32
String readSerialString();
int binaryToDecimal(int n);
void sendRobustFloat(float value);

// -------------------------------- SETUP ---------------------------------
void setup()
{
  // initialize the IMU AD0 pins and the button input pin
  pinMode(IMU0, OUTPUT);
  pinMode(IMU1, OUTPUT);
  pinMode(IMU2, OUTPUT);
  pinMode(IMU3, OUTPUT);
  pinMode(IMU4, OUTPUT);
  pinMode(BUTTON, INPUT_PULLDOWN);

  // small delay to let Serial initialize
  delay(1000);

  // start the Serial connection to the laptop for data collection
  Serial.begin(115200);
  Serial.println("HelloWorld!");

  // start the Serial connection to the STM32
  SerialPort.begin(9600, SERIAL_8N1, RX, TX);
  hx_servo.setupServo();

  // initialize the IMUs
  initMPU(IMU0);
  initMPU(IMU1);
  initMPU(IMU2);
  initMPU(IMU3);
  initMPU(IMU4);

  // make the top platform bow its head to show respect
  delay(5000);
  moveSlowly(&home_coords[0], &bow_coords[0], 25, binaryToDecimal(1));
  moveSlowly(&bow_coords[0], &home_coords[0], 25, binaryToDecimal(1));

  // turn off the LED and wait for a button press before entering the main loop
  digitalWrite(LED, LOW);
  while (!digitalRead(BUTTON)){}
  digitalWrite(LED, HIGH);
  delay(1000);

  // --- DEMONSTRATIONS ---

  // minmax demonstration
  hx_demo.demoMov_MinMaxAllAxis();
  delay(1000);

  // turn off the LED and wait for a button press before entering the main loop
  digitalWrite(LED, LOW);
  while (!digitalRead(BUTTON)){}
  digitalWrite(LED, HIGH);
  delay(1000);

  // circle demonstration
  float cir_radius = 15;
  platform_t circle_coords[] = {0, 0, 0, radians(cir_radius), 0, 0};
  moveSlowly(&home_coords[0], &circle_coords[0], 25, binaryToDecimal(1111));
  hx_demo.demoMov_circles(1, cir_radius);
  moveSlowly(&circle_coords[0], &home_coords[0], 25, binaryToDecimal(1111));
  delay(1000);
  
  // turn off the LED and wait for a button press before entering the main loop
  digitalWrite(LED, LOW);
  while (!digitalRead(BUTTON)){}
  digitalWrite(LED, HIGH);
  delay(1000);


  // // active transmitter tracking demonstration (BLOCKING FUNCTION)
  // hx_demo.lookAtTX();
  
}

// -------------------------------- MAIN LOOP ---------------------------------
void loop()
{
  // move_str defines which platforms should be moving, using binary notation.
  // 1111 corresponds to all the platforms moving. 1000 corresponds to only the
  // bottom (first) platform moving. 101 corresponds to the second and fourth
  // platforms moving. don't include the leading 0!
  int move_str = 1111;

  // flag that tracks if the loop should break; we want to break the loop when
  // the button is pressed (resetting to home)
  bool shouldBreak = false;

  // step increments for different axes of motion; linear corresponds to XYZ
  // motion, angular corresponds to ABC motion
  float linear_step_increment = .5;
  float angular_step_increment = radians(0.25);

  // sometimes it was helpful to test only the translation axes, and then only
  // the rotational axes. this flip-flop switches after each entry in axes[] is
  // tested.
  if (translational_axes) 
  {
    axes[0] = "X";
    axes[1] = "Y";
    axes[2] = "Z";
  }
  else 
  {
    axes[0] = "Z";
    axes[1] = "B";
    axes[2] = "C";
  }

  // these track the current orientation of the iSBL / STM32 array
  float stm_roll = 0.0;
  float stm_pitch = 0.0;
  float stm_yaw = 0.0;

  // this tells us how many axes to iterate over
  const int num_axes = sizeof(axes) / sizeof(axes[0]);

  // these two flags were helpful for running different kinds of tests.
  // if stationary is set to 1, then the Fo-SHIP stays still. if it's set to
  // 0, it will iterate through its axes. if gradual is set to 1, the motion
  // is small and slow. if it's set to 0, the motion is large and rapid. this
  // was nice when testing out the system's resilience to jerky motion
  int stationary = 0;
  int gradual = 1;

  if (!gradual){
    linear_step_increment = 2;
    angular_step_increment = radians(2);
  }

  // main loop for iterating through different axes
  for (int axis = 0; axis < num_axes; axis++) {

    float start_val, end_val, step_increment;
    
    // set the start and end values based on the current axis
    // absolute max/min values are located in Hexapod_Config_1.h
    if (axes[axis] == "X") {
      start_val = 5;
      end_val = -5;
      step_increment = linear_step_increment;
    } else if (axes[axis] == "Y") {
      start_val = 5;
      end_val = -5;
      step_increment = linear_step_increment;
    } else if (axes[axis] == "Z") {
      start_val = 16;
      end_val = -16;
      step_increment = linear_step_increment;
    } else if (axes[axis] == "A") {
      start_val = radians(5);
      end_val = radians(-5);
      step_increment = angular_step_increment;
    } else if (axes[axis] == "B") {
      start_val = radians(0.5);
      end_val = radians(-1.5);
      step_increment = angular_step_increment;
    } else if (axes[axis] == "C") {
      start_val = radians(2);
      end_val = radians(-2);
      step_increment = angular_step_increment;
    }

    // this flag tracks if the platform is at its first or last value for its current axis
    int first_last = 0;

    // if the platform is stationary, this for() loop will not increment and it stays at the zero position
    for (float current_val = start_val*(1-stationary); current_val >= end_val; current_val -= step_increment*(1-stationary)) {
      platform_t newPos = {0, 0, 0, 0, 0, 0};
      
      // update the appropriate axis
      if (axes[axis] == "X") newPos.hx_x = current_val;
      else if (axes[axis] == "Y") newPos.hx_y = current_val;
      else if (axes[axis] == "Z") newPos.hx_z = current_val;
      else if (axes[axis] == "A") newPos.hx_a = current_val;
      else if (axes[axis] == "B") newPos.hx_b = current_val;
      else if (axes[axis] == "C") newPos.hx_c = current_val;

      // if this is the first value in this axis, discard the next datapoint. when moving to a new axis,
      // the true position of the platform for its new position will be incorrect. for the next position,
      // it will stay in its current location and calculate its position again.
      // i know this is not the "proper" way to solve this issue, but it works and is reliable.
      if (!first_last && !stationary && (current_val == start_val)){
        Serial.println("DISCARD");
        current_val = start_val + step_increment;
        first_last = 1;
      }

      // similarly, if this is the last position, rerun the same position again. because of the order that
      // data is transmitted between the ESP32 and the STM32 (next position that it's going to, then the 
      // results of that next position) the last datapoint doesn't get printed. this ensures that it does.
      // again, like above, not the right way to do it but it works and is reliable. and i'm tired
      if ((first_last == 1) && !stationary && (current_val == end_val)){
        current_val = end_val + step_increment;
        first_last = 0;
      }

      // in this current implementation, i'm measuring what the true orientation of the iSBL array was (by
      // trusting that the IMU running on it is accurate) and using that orientation to correct the forward
      // kinematics of the Fo-SHIP. basically, we take the orientation of the platform from the last datapoint,
      // add the change in position from the last datapoint to the next (aka, how far has the platform moved
      // since the last datapoint), and use that to calculate what the "true" orientation of each platform is.

      // i assume that, based on the forward kinematics model, the current angle of each platform about any axis
      // is the current angle of the end effector divided by 4 (four platforms). while this may seem like an
      // oversimplification, it changes the position accuracy from +/- 2cm to +/- 1cm. the only issue is that
      // the yaw value tends to drift quite a bit, but since the yaw axis only affects the position estimate by
      // a small amount compared to the rest of the platform (due to the small lever arm of 110mm from the center
      // of the platform to the center of the iSBL array), it's an acceptable error.
      platform_t modPos = newPos;
      float rollDiff = modPos.hx_a - currentPos.hx_a;
      float pitchDiff = modPos.hx_b - currentPos.hx_b;
      float yawDiff = modPos.hx_c - currentPos.hx_c;
      modPos.hx_a = radians(stm_roll/4) + rollDiff;
      modPos.hx_b = radians(stm_pitch/4) + pitchDiff;
      modPos.hx_c = radians(stm_yaw/4) + yawDiff;

      // once we've accounted for the "true" orientation of the platform, we can recalcuate the end effector
      // (aka the iSBL array center) XYZ coordinates for the upcoming datapoint, which are stored in true_x, 
      // true_y, and true_z
      getEndEffectorCoords(&modPos, &modPos, &modPos, &modPos, false);

      // now, we send the true_x coordinate of the upcoming position to the STM32 so that it can use it for
      // calculating the "depth" reading. you can read more about it in my thesis or the STM32 code, but 
      // essentially we need to know the x-distance from the transmitter to the receiver array. in a real
      // submarine, this would be handled by a depth pressure sensor, but since we don't have that, we 
      // simulate a "depth" reading by subtracting the known x-coordinate of the transmitter from the
      // x-coordinate of the iSBL array (origin is at the base of the Fo_SHIP). this gives us a depth
      // reading that we can work with
      char message[15];
      float next_val = true_x / 1000;

      // we'll try and send the message up to three times; this tracks how many times it takes
      int good_send = 0;

      // this structure will be quite common in the code below. essentially it's a waiting block;
      // here, we wait until the serial port is available (which happens after the STM32 sends a new
      // datapoint, which is later in this code). we want to be able to break out of this loop whenever
      // we need by pressing the button, and this is the easiest way to do it without using multitasking.
      while (!SerialPort.available()){

        // if the button is pressed,
        if (digitalRead(BUTTON)){

          // turn the LED on
          digitalWrite(LED, HIGH);

          // move back to home
          moveSlowly(&currentPos, &home_coords[0], 20, binaryToDecimal(move_str));
          currentPos = {0, 0, 0, 0, 0, 0};
          
          // break out of all the loops
          shouldBreak = true;

          // turn the LED back off
          digitalWrite(LED, LOW);
          Serial.println("BUTTON PRESSED, 1");

          // wait until the button is pressed again to resume testing (just holds here, then restarts main)
          while (!digitalRead(BUTTON)){}
          digitalWrite(LED, HIGH);
          delay(1000);
          break;
        }
      }

      if (shouldBreak) {
        break;
      }

      // loop until our send has been received or until we've tried three times
      while(!good_send){
        
        // send the x-coordinate of the next position over UART to the STM32
        sendRobustFloat(next_val);

        // wait in this loop until the STM32 replies, or break if the button is pressed
        while (!SerialPort.available() && !shouldBreak){
          if (digitalRead(BUTTON)){
            digitalWrite(LED, HIGH);
            moveSlowly(&currentPos, &home_coords[0], 20, binaryToDecimal(move_str));
            shouldBreak = true;
            digitalWrite(LED, LOW);
            Serial.println("BUTTON PRESSED, 2");
            currentPos = {0, 0, 0, 0, 0, 0};
            while (!digitalRead(BUTTON)){}
            digitalWrite(LED, HIGH);
            delay(1000);
            break;
          }
        }

        // decode the message from the STM32 and parse it for a reply
        // a reply of "1" means that it was received successfully
        // a reply of "0" means it was not received successfully
        // a reply of "-1" means it took more than three tries and that it should just discard
        // this datapoint and try again next time
        String receivedString = readSerialString();
        sscanf(receivedString.c_str(), "%i", &good_send);
        if (good_send == -1){
          Serial.println("DISCARD");
          break;
        }
      }

      if (shouldBreak) {
        break;
      }

      // now that we've sent the STM32 the next position it will reach, we need to move to that position
      // (assuming we're not stationary). note that the gradual flag affects how quickly the platform
      // moves to its new position.
      if (!stationary){
        
        // if moving to a new axis (aka, first reading for this axis), go slow because it's a big change
        if (current_val == start_val + step_increment){
          moveSlowly(&currentPos, &newPos, 20 + gradual*4, binaryToDecimal(move_str));
        }

        // otherwise, go normal speed
        else{
          moveSlowly(&currentPos, &newPos, 6 + gradual*10, binaryToDecimal(move_str));
        }
      }

      // update the current position
      currentPos = newPos;

      // if the MPU6050s were not defective, this is where we'd read from them and calculate
      // the true end effector position more accurately
      // delay(500);
      // readMPU(IMU0);
      // delay(100);
      // readMPU(IMU1);
      // delay(100);
      // readMPU(IMU2);
      // delay(100);
      // readMPU(IMU3);
      // delay(100);
      // readMPU(IMU4);
      // getEndEffectorCoords(&currentPos, &currentPos, &currentPos, &currentPos, true);

      // wait for the STM32 to send its datapoint for the new position, or break if the button is pressed
      while (!SerialPort.available()){
        digitalWrite(2, LOW);
        if (digitalRead(BUTTON)){
          digitalWrite(2, HIGH);
          moveSlowly(&newPos, &home_coords[0], 20, binaryToDecimal(move_str));
          shouldBreak = true;
          digitalWrite(2, LOW);
          Serial.println("BUTTON PRESSED, 3");
          currentPos = {0, 0, 0, 0, 0, 0};
          while (!digitalRead(BUTTON)){}
          digitalWrite(2, HIGH);
          delay(1000);
          break;
        }
      }
      
      if (shouldBreak) {
        break;
      }

      // once the STM32 sends its data, parse it and put the data into floats
      String receivedString = readSerialString();
      float x_kf, y_kf, z_kf, x_r_isbl, y_r_isbl, z_r_isbl, x_isbl, y_isbl, z_isbl;
      sscanf(receivedString.c_str(), "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", 
             &x_kf, &y_kf, &z_kf, &x_r_isbl, &y_r_isbl, &z_r_isbl, 
             &x_isbl, &y_isbl, &z_isbl, &stm_roll, &stm_pitch, &stm_yaw);

      // once we have the new orientation of the platform, use it to recalculate the true end effector pos
      modPos = currentPos;
      modPos.hx_a = radians(stm_roll/4);
      modPos.hx_b = radians(stm_pitch/4);
      modPos.hx_c = radians(stm_yaw/4);
      getEndEffectorCoords(&modPos, &modPos, &modPos, &modPos, false);

      // since XYZ and ABC have different units, we need to format the current value differently
      float printval;
      printval = degrees(current_val*4);
      if (translational_axes) printval = current_val*4;
      
      // print a single datapoint / row
      Serial.println(String(axes[axis]) + ", " + String(printval)+ ": "
                  + String(x_kf) + ", " + String(y_kf) + ", " + String(z_kf) + "; " 
                  + String(x_r_isbl) + ", " + String(y_r_isbl) + ", " + String(z_r_isbl) + "; "
                  + String(x_isbl) + ", " + String(y_isbl) + ", " + String(z_isbl) + "; "
                  + String(true_x) + ", " + String(true_y) + ", " + String(true_z) + "; "
                  + String(stm_roll) + ", " + String(stm_pitch) + ", " + String(stm_yaw) + ";");

      // --- CODE FOR TRANSMITTING LONG DATA STRINGS, USUALLY ALL 2048 ROWS OF AN ADC RECORDING ---
      // Serial.println("DATA HERE");
      // while (!digitalRead(BUTTON)) {
      //   while (!SerialPort.available());
      //   String micData = readSerialString();
      //   if (micData == "END") {
      //     break;
      //   }
      //   Serial.println(micData);
      // } 
      // Serial.println("END_MIC_DATA");
      // Serial.println("");

    }

    if (shouldBreak) {
      break;
    }
  }
  translational_axes = !translational_axes;
}

/**
 * @brief Initializes the specified MPU6050 IMU
 * 
 * This function initializes the IMU specifed by IMU_NUM, which is
 * the GPIO pin associated with that IMU's AD0 pin. We pull that MPU's
 * AD0 pin low, run the begin() method for that IMU from the Adafruit
 * MPU6050 library, and set it according to the ranges below.
 * 
 * This function must be run for each individual MPU on the Fo-SHIP.
 * 
 * @param IMU_NUM the GPIO pin of the IMU you want to initialize
 */
void initMPU(int IMU_NUM) {

  // pull the AD0 pin low
  digitalWrite(IMU0, HIGH);
  digitalWrite(IMU1, HIGH);
  digitalWrite(IMU2, HIGH);
  digitalWrite(IMU3, HIGH);
  digitalWrite(IMU4, HIGH);
  digitalWrite(IMU_NUM, LOW);

  // run the begin() method
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }
}

/**
 * @brief Reads accelerometer data from specified IMU, converts to Euler angles
 * 
 * This function reads from an individual MPU. It pulls that MPU's AD0 pin low
 * to change the I2C address (so we only talk to the one we want), reads from the
 * accelerometer a set number of times, and averages out the readings to reduce 
 * noise. Then, it processes the accelerometer data.
 * 
 * First, it converts the accelerometer readings into Euler angles using the function
 * @see getEulerFromMPU(). Then, it converts the Euler angles into their rotation matrix
 * representation, and additionally computes the transpose of that rotation matrix. For
 * all IMUs attached to moving parts of the Fo-SHIP (aka, all except the base, IMU0), it
 * then multiplies the current IMU's rotation matrix by the transpose of the rotation
 * matrix for the IMU below it. This gives us the difference in orientation between the
 * two IMUs, which we compare to the setpoint for each of the levels. For example, if the 
 * first and second platform both set to a roll angle of 10 degrees, we wouldn't want to
 * compare the second platform's true orientation (as measured by IMU) with the setpoint,
 * since the true orientation would be 20 degrees.
 * 
 * For all IMUs but IMU0, we extract the Euler angles from these relative rotation
 * matrices and store them in their respective structs (or print, if debug mode is on).
 * 
 * @param IMU_NUM the GPIO pin of the IMU you want to read from
 * @param num_avgs how many readings to average from the accelerometer
 */
void readMPU(int IMU_NUM, int num_avgs) {

  // pull the AD0 pin low
  digitalWrite(IMU0, HIGH);
  digitalWrite(IMU1, HIGH);
  digitalWrite(IMU2, HIGH);
  digitalWrite(IMU3, HIGH);
  digitalWrite(IMU4, HIGH);
  digitalWrite(IMU_NUM, LOW);
  
  // take multiple readings from the accelerometer and calculate the average
  sensors_event_t a, g, temp;
  float avg_accel_x = 0, avg_accel_y = 0, avg_accel_z = 0;
  
  for (int i = 0; i < num_avgs; i++) {
    mpu.getEvent(&a, &g, &temp);
    avg_accel_x += a.acceleration.x;
    avg_accel_y += a.acceleration.y;
    avg_accel_z += a.acceleration.z;
  }
  
  avg_accel_x /= num_avgs;
  avg_accel_y /= num_avgs;
  avg_accel_z /= num_avgs;
  
  switch (IMU_NUM) {
    case IMU0:
      IMU0_AccelData[0] = avg_accel_x;
      IMU0_AccelData[1] = avg_accel_y;
      IMU0_AccelData[2] = avg_accel_z;
      getEulerFromMPU(IMU0_AccelData, IMU0_Angles);
      getRotMatFromEuler(IMU0_Angles, IMU0_RotMat);
      transposeMatrix(IMU0_RotMat, IMU0trans_RotMat);
      if (printMPU){
        Serial.print("IMU0 Roll: ");
        Serial.print(RAD_TO_DEG * IMU0_Angles[0]);
        Serial.print(" deg, Pitch: ");
        Serial.print(RAD_TO_DEG * IMU0_Angles[1]);
        Serial.print(" deg, Yaw: ");
        Serial.print(RAD_TO_DEG * IMU0_Angles[2]);
        Serial.println(" deg");
      }
      break;
    case IMU1:
      IMU1_AccelData[0] = avg_accel_x;
      IMU1_AccelData[1] = avg_accel_y;
      IMU1_AccelData[2] = avg_accel_z;
      getEulerFromMPU(IMU1_AccelData, IMU1_Angles);
      getRotMatFromEuler(IMU1_Angles, IMU1_RotMat);
      transposeMatrix(IMU1_RotMat, IMU1trans_RotMat);
      multiply3x3Matrices(IMU0trans_RotMat, IMU1_RotMat, IMU1rel0_RotMat);
      getEulerFromRotMat(IMU1rel0_RotMat, IMU1_Angles);
      if (printMPU){
        Serial.print("IMU1 Roll: ");
        Serial.print(RAD_TO_DEG * IMU1_Angles[0]);
        Serial.print(" deg, Pitch: ");
        Serial.print(RAD_TO_DEG * IMU1_Angles[1]);
        Serial.print(" deg, Yaw: ");
        Serial.print(RAD_TO_DEG * IMU1_Angles[2]);
        Serial.println(" deg");
      }
      break;
    case IMU2:
      IMU2_AccelData[0] = avg_accel_x;
      IMU2_AccelData[1] = avg_accel_y;
      IMU2_AccelData[2] = avg_accel_z;
      getEulerFromMPU(IMU2_AccelData, IMU2_Angles);
      getRotMatFromEuler(IMU2_Angles, IMU2_RotMat);
      transposeMatrix(IMU2_RotMat, IMU2trans_RotMat);
      multiply3x3Matrices(IMU1trans_RotMat, IMU2_RotMat, IMU2rel1_RotMat);
      getEulerFromRotMat(IMU2rel1_RotMat, IMU2_Angles);
      if (printMPU){
        Serial.print("IMU2 Roll: ");
        Serial.print(RAD_TO_DEG * IMU2_Angles[0]);
        Serial.print(" deg, Pitch: ");
        Serial.print(RAD_TO_DEG * IMU2_Angles[1]);
        Serial.print(" deg, Yaw: ");
        Serial.print(RAD_TO_DEG * IMU2_Angles[2]);
        Serial.println(" deg");
      }
      break;
    case IMU3:
      IMU3_AccelData[0] = avg_accel_x;
      IMU3_AccelData[1] = avg_accel_y;
      IMU3_AccelData[2] = avg_accel_z;
      getEulerFromMPU(IMU3_AccelData, IMU3_Angles);
      getRotMatFromEuler(IMU3_Angles, IMU3_RotMat);
      transposeMatrix(IMU3_RotMat, IMU3trans_RotMat);
      multiply3x3Matrices(IMU2trans_RotMat, IMU3_RotMat, IMU3rel2_RotMat);
      getEulerFromRotMat(IMU3rel2_RotMat, IMU3_Angles);
      if (printMPU){
        Serial.print("IMU3 Roll: ");
        Serial.print(RAD_TO_DEG * IMU3_Angles[0]);
        Serial.print(" deg, Pitch: ");
        Serial.print(RAD_TO_DEG * IMU3_Angles[1]);
        Serial.print(" deg, Yaw: ");
        Serial.print(RAD_TO_DEG * IMU3_Angles[2]);
        Serial.println(" deg");
      }
      break;
    case IMU4:
      IMU4_AccelData[0] = avg_accel_x;
      IMU4_AccelData[1] = avg_accel_y;
      IMU4_AccelData[2] = avg_accel_z;
      getEulerFromMPU(IMU4_AccelData, IMU4_Angles);
      getRotMatFromEuler(IMU4_Angles, IMU4_RotMat);
      multiply3x3Matrices(IMU3trans_RotMat, IMU4_RotMat, IMU4rel3_RotMat);
      getEulerFromRotMat(IMU4rel3_RotMat, IMU4_Angles);
      if (printMPU){
        Serial.print("IMU4 Roll: ");
        Serial.print(RAD_TO_DEG * IMU4_Angles[0]);
        Serial.print(" deg, Pitch: ");
        Serial.print(RAD_TO_DEG * IMU4_Angles[1]);
        Serial.print(" deg, Yaw: ");
        Serial.print(RAD_TO_DEG * IMU4_Angles[2]);
        Serial.println(" deg");
      }
      break;
  }
}

/**
 * @brief Converts accelerometer readings into Euler angles
 * 
 * This function converts the accelerometer readings into Euler angles using
 * a method from Michael Wrona's blog, "Roll and Pitch Angles From Accelerometer 
 * Sensors". We transform the coordinate frame of the IMUs into our global frame,
 * then compute the normalized values of each axis reading. We can then use some
 * trigonometry to compute what the pitch and roll angles for each IMU are. Since
 * we are only reading accelerometer data and not magnetometer data, we don't know
 * where North is, and thus cannot compute yaw. For simplicity, we assume yaw is 0.
 * 
 * @see https://mwrona.com/posts/accel-roll-pitch/
 * 
 * @param IMU_AccelData pointer to accelerometer data struct
 * @param IMU_Angles pointer to Euler angles struct
 */
void getEulerFromMPU(float *IMU_AccelData, float *IMU_Angles){

  // IMU_AccelData = {-a_y, -a_z, a_x}, m/s2
  // IMU_Angles = {pitch, roll, yaw}, rad
  float a_x = -IMU_AccelData[1];
  float a_y = -IMU_AccelData[0];
  float a_z = IMU_AccelData[2];
  float mag = sqrtf(POW(a_x,2) + POW(a_y,2) + POW(a_z,2));
  float a_x_norm = a_x / mag;
  float a_y_norm = a_y / mag;
  float a_z_norm = a_z / mag;
  float pitch = -asinf(a_x_norm);
  float roll = atan2f(a_y_norm, a_z_norm);
  IMU_Angles[0] = roll;
  IMU_Angles[1] = pitch;
  IMU_Angles[2] = 0; // yaw, unmeasurable
}

/**
 * @brief Converts Euler angles into a rotation matrix
 * 
 * This function converts Euler angles into their corresponding rotation matrix
 * using a method from Gregory G. Slabaugh of the Queen Mary University of London,
 * "Computing Euler angles from a rotation matrix". We manually fill the entries
 * of the matrix according to the deriation in the paper.
 * 
 * @see https://eecs.qmul.ac.uk/~gslabaugh/publications/euler.pdf
 * 
 * @param IMU_Angles pointer to Euler angles struct
 * @param IMU_RotMat pointer to rotation matrix struct
 */
void getRotMatFromEuler(float *IMU_Angles, float *IMU_RotMat){
  float pitch = IMU_Angles[0];
  float roll = IMU_Angles[1];
  float yaw = IMU_Angles[2];
  IMU_RotMat[0] = cosf(roll) * cosf(yaw);
  IMU_RotMat[1] = sinf(pitch) * sinf(roll) * cosf(yaw) - cosf(pitch) * sinf(yaw);
  IMU_RotMat[2] = cosf(pitch) * sinf(roll) * cosf(yaw) + sinf(pitch) * sinf(yaw);
  IMU_RotMat[3] = cosf(roll) * sinf(yaw);
  IMU_RotMat[4] = sinf(pitch) * sinf(roll) * sinf(yaw) + cosf(pitch) * cosf(yaw);
  IMU_RotMat[5] = cosf(pitch) * sinf(roll) * sinf(yaw) - sinf(pitch) * cosf(yaw);
  IMU_RotMat[6] = -sinf(roll);
  IMU_RotMat[7] = sinf(pitch) * cosf(roll);
  IMU_RotMat[8] = cosf(pitch) * cosf(roll);
}

/**
 * @brief Converts a rotation matrix into Euler angles
 * 
 * This function converts a rotation matrix into its corresponding Euler angles
 * using a method from Gregory G. Slabaugh of the Queen Mary University of London,
 * "Computing Euler angles from a rotation matrix". We manually fill the entries
 * of the matrix according to the deriation in the paper.
 * 
 * @param IMU_RotMat pointer to rotation matrix struct
 * @param IMU_Angles pointer to Euler angles struct
 */
void getEulerFromRotMat(float *IMU_RotMat, float *IMU_Angles){
  IMU_Angles[0] = atan2f(IMU_RotMat[7], IMU_RotMat[8]); // pitch
  IMU_Angles[1] = -asin(IMU_RotMat[6]); // roll
  IMU_Angles[2] = atan2f(IMU_RotMat[3] / cosf(IMU_Angles[1]), IMU_RotMat[0] / cosf(IMU_Angles[1])); // yaw
}

/**
 * @brief Multiplies two 3x3 matrices
 * 
 * This function multiplies two 3x3 matrices represented by a float of size [9].
 * It carries out the multiplication operation for matrices of this size.
 * 
 * @param matA pointer to 3x3 matrix A
 * @param matB pointer to 3x3 matrix B
 * @param result pointer to 3x3 matrix result of A*B
 */
void multiply3x3Matrices(float *matA, float *matB, float *result) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      float sum = 0;
      for (int k = 0; k < 3; k++) {
        sum += matA[i * 3 + k] * matB[k * 3 + j];
      }
      result[i * 3 + j] = sum;
    }
  }
}

/**
 * @brief Transposes a 3x3 matrix
 * 
 * This function transposes a 3x3 matrix represented by a float of size [9].
 * 
 * @param mat pointer to input 3x3 matrix A
 * @param result pointer to output 3x3 matrix A^T
 */
void transposeMatrix(float *mat, float *result) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      result[i * 3 + j] = mat[j * 3 + i];
    }
  }
}

/**
 * @brief Multiplies a 3x3 matrix by a 3x1 column vector
 * 
 * This function multiplies a 3x3 matrix by a 3x1 column vector represented by
 * floats of size [9] and [3], respectively.
 * 
 * @param mat pointer to 3x3 matrix A
 * @param vec pointer to 3x1 column vector B
 * @param result pointer to 3x3 result matrix A*B
 */
void multiply3x3MatrixVector(float *mat, float *vec, float *result) {
  result[0] = mat[0] * vec[0] + mat[1] * vec[1] + mat[2] * vec[2];
  result[1] = mat[3] * vec[0] + mat[4] * vec[1] + mat[5] * vec[2];
  result[2] = mat[6] * vec[0] + mat[7] * vec[1] + mat[8] * vec[2];
}

/**
 * @brief Computes the coordinates of the iSBL receiver array center
 * 
 * This function takes in the setpoint of the four platforms of the Fo-SHIP and computes the XYZ coordinate of the iSBL
 * receiver array center. This is used to validate the results of the acoustic positioning method. Optionally, the input
 * USE_IMU can be set to true if you want to use the on-board IMUs to correct the setpoint of the Fo-SHIP.
 * 
 * I highly recommend reading section 2.5 of my thesis, linked at the top, for visual guides for this function.
 * 
 * First, two vectors are defined: the offset between the base of each platform and the center of rotation for each of the
 * servos, and the offset between the top platform's end effector and the center of the iSBL array. The first offset is a 
 * constant value across all platforms in the Fo-SHIP. The second offset was measured using calipers and the CAD model of the
 * adapter that connects the end effector plate to the iSBL array.
 * 
 * Next, we create a translational vector for each platform that defines the XYZ offset between the center of that platform's 
 * servo array and the center of the top plate for that platform; essentially, what is the XYZ position of the end effector
 * of each platform relative to its base? Then, we create the Euler angles for each platform bsaed on the setpoint. These
 * angles are converted to rotation matrices, which are then multiplied due to the serial/stacked nature of the Fo-SHIP - 
 * the current platform's orientation depends on the one beneath it. We then work from the base up, determining the following:
 *  - What is the vector between the base and top of the current platform in the platform's frame?
 *  - What is the orientation of the platform in the global frame?
 *  - How does the vector shift when rotating from the platform frame to the global frame?
 * 
 * We simply stack these vectors on top of each other to arrive at the end effector coordinates, and finally, we add the
 * offset between the end effector and the iSBL receiver array's center, which is stored in the floats true_x, true_y, and
 * true_z.
 * 
 * Because of the nature of the Fo-SHIP's mechanical design, it has a bit of slop in the system - this is primarily caused
 * by the servos used. Setting a servo to a certain angle doesn't guarantee that it will hold that angle exactly; due to
 * the forces acting on the servo arm and the way that the servos "hold" a position, the true orientation of the arm can be
 * multiple degrees off of the true angle. Because I don't have encoders attached to the servos, the true angle of each servo
 * is technically unknown and the system might be at a different position that it's set to be. However, we can account for
 * this using IMUs on each level.
 * 
 * By measuring the accelerometer data of each IMU attached to each level of the Fo-SHIP, we can read what the true angle is
 * for each level. If USE_IMU is set to true, it replaces the angles of each platform_t struct setpoint with the actual angles
 * measured by the IMUs. This assumes that slop in the system does not affect the translation vector between each platform,
 * and that it only affects the rotation vector. This turns out to be a good-enough assumption for real-life. Then, the
 * function performs the same procedure described above, just with the updated angles.
 * 
 * @param coords1 pointer to platform_t struct with coordinates for platform 1
 * @param coords2 pointer to platform_t struct with coordinates for platform 2
 * @param coords3 pointer to platform_t struct with coordinates for platform 3
 * @param coords4 pointer to platform_t struct with coordinates for platform 4
 * @param USE_IMU bool to specify if the on-board MPU6050 IMUs should be used to correct the angle setpoint
 */
void getEndEffectorCoords(platform_t *coords1, platform_t *coords2, platform_t *coords3, platform_t *coords4, bool USE_IMU){
  float d[3] = {0, 0, -34};
  float TP[3] = {80 + 30, 0, -78.38 - 4};
  float zhome = static_cast<float>(Z_HOME);

  float T1[3] = {static_cast<float>(coords1->hx_x), static_cast<float>(coords1->hx_y), static_cast<float>(coords1->hx_z) + zhome};
  float T2[3] = {static_cast<float>(coords2->hx_x), static_cast<float>(coords2->hx_y), static_cast<float>(coords2->hx_z) + zhome};
  float T3[3] = {static_cast<float>(coords3->hx_x), static_cast<float>(coords3->hx_y), static_cast<float>(coords3->hx_z) + zhome};
  float T4[3] = {static_cast<float>(coords4->hx_x), static_cast<float>(coords4->hx_y), static_cast<float>(coords4->hx_z) + zhome};

  float eul1[3] = {0, 0, static_cast<float>(coords1->hx_c)};
  float eul2[3] = {0, 0, static_cast<float>(coords2->hx_c)};
  float eul3[3] = {0, 0, static_cast<float>(coords3->hx_c)};
  float eul4[3] = {0, 0, static_cast<float>(coords4->hx_c)};

  if (USE_IMU) {
    eul1[0] = IMU1_Angles[0];
    eul1[1] = IMU1_Angles[1];
    eul2[0] = IMU2_Angles[0];
    eul2[1] = IMU2_Angles[1];
    eul3[0] = IMU3_Angles[0];
    eul3[1] = IMU3_Angles[1];
    eul4[0] = IMU4_Angles[0];
    eul4[1] = IMU4_Angles[1];
  } else {
    eul1[0] = static_cast<float>(coords1->hx_a);
    eul1[1] = static_cast<float>(coords1->hx_b);
    eul2[0] = static_cast<float>(coords2->hx_a);
    eul2[1] = static_cast<float>(coords2->hx_b);
    eul3[0] = static_cast<float>(coords3->hx_a);
    eul3[1] = static_cast<float>(coords3->hx_b);
    eul4[0] = static_cast<float>(coords4->hx_a);
    eul4[1] = static_cast<float>(coords4->hx_b);
  }
  
  float R0[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  float R1[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  float R2[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  float R3[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  float R4[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  
  float R1R0[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  float R2R1R0[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  float R3R2R1R0[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  float R4R3R2R1R0[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  
  getRotMatFromEuler(eul1, R1);
  getRotMatFromEuler(eul2, R2);
  getRotMatFromEuler(eul3, R3);
  getRotMatFromEuler(eul4, R4);
  
  multiply3x3Matrices(R1, R0, R1R0);
  multiply3x3Matrices(R2, R1R0, R2R1R0);
  multiply3x3Matrices(R3, R2R1R0, R3R2R1R0);
  multiply3x3Matrices(R4, R3R2R1R0, R4R3R2R1R0);
  
  float x0[3] = {0, 0, -30};
  float dT1[3] = {d[0] + T1[0], d[1] + T1[1], d[2] + T1[2]};
  float x1[3];
  multiply3x3MatrixVector(R0, dT1, x1);
  x1[0] += x0[0];
  x1[1] += x0[1];
  x1[2] += x0[2];
  
  float dT2[3] = {d[0] + T2[0], d[1] + T2[1], d[2] + T2[2]};
  float x2[3];
  multiply3x3MatrixVector(R1R0, dT2, x2);
  x2[0] += x1[0];
  x2[1] += x1[1];
  x2[2] += x1[2];
  
  float dT3[3] = {d[0] + T3[0], d[1] + T3[1], d[2] + T3[2]};
  float x3[3];
  multiply3x3MatrixVector(R2R1R0, dT3, x3);
  x3[0] += x2[0];
  x3[1] += x2[1];
  x3[2] += x2[2];
  
  float dT4[3] = {d[0] + T4[0], d[1] + T4[1], d[2] + T4[2]};
  float x4[3];
  multiply3x3MatrixVector(R3R2R1R0, dT4, x4);
  x4[0] += x3[0];
  x4[1] += x3[1];
  x4[2] += x3[2];

  float xP[3];
  multiply3x3MatrixVector(R4R3R2R1R0, TP, xP);
  xP[0] += x4[0];
  xP[1] += x4[1];
  xP[2] += x4[2];

  true_x = xP[0];
  true_y = xP[1];
  true_z = xP[2];

  // --- DEBUG MESSAGES FOR PRINTING EACH PLATFORM CENTER'S XYZ COORDINATE ---
  // Serial.print("x0: ");
  // Serial.print(x0[0]); Serial.print(", ");
  // Serial.print(x0[1]); Serial.print(", ");
  // Serial.println(x0[2]);

  // Serial.print("x1: ");
  // Serial.print(x1[0]); Serial.print(", ");
  // Serial.print(x1[1]); Serial.print(", ");
  // Serial.println(x1[2]);

  // Serial.print("x2: ");
  // Serial.print(x2[0]); Serial.print(", ");
  // Serial.print(x2[1]); Serial.print(", ");
  // Serial.println(x2[2]);

  // Serial.print("x3: ");
  // Serial.print(x3[0]); Serial.print(", ");
  // Serial.print(x3[1]); Serial.print(", ");
  // Serial.println(x3[2]);

  // Serial.print("x4: ");
  // Serial.print(x4[0]); Serial.print(", ");
  // Serial.print(x4[1]); Serial.print(", ");
  // Serial.println(x4[2]);

  // Serial.print("xP: ");
  // Serial.print(xP[0]); Serial.print(", ");
  // Serial.print(xP[1]); Serial.print(", ");
  // Serial.println(xP[2]);
}

/**
 * @brief Moves specified platforms from their current setpoint to a new setpoint by interpolating between the two
 * 
 * This function instructs the Fo-SHIP to move a specified number of platforms from the start setpoint to the end
 * setpoint over a number of steps, interpolating beween the two. All platforms move in the same way, so care should
 * be taken that all the platforms are starting at the same setpoint.
 * 
 * @param startCoords pointer to platform_t struct with the platform's current setpoint
 * @param endCoords pointer to platform_t struct with the platform's new setpoint
 * @param num_steps number of steps to interpolate between the start and end setpoints
 * @param which_platform a binary representation of which platforms to move, binaryToDecimal() should be used here
 */
void moveSlowly(platform_t *startCoords, platform_t *endCoords, uint16_t num_steps, uint8_t which_platform) {
    platform_t startPos = *startCoords;
    platform_t endPos = *endCoords;
    int8_t movOK = -1;

    for (uint16_t step = 0; step < num_steps; step++) {
        double t = ((double)step) / (double)num_steps;

        // linearly interpolate between startPos and endPos
        platform_t interpPos;
        interpPos.hx_x = startPos.hx_x + t * (endPos.hx_x - startPos.hx_x);
        interpPos.hx_y = startPos.hx_y + t * (endPos.hx_y - startPos.hx_y);
        interpPos.hx_z = startPos.hx_z + t * (endPos.hx_z - startPos.hx_z);
        interpPos.hx_a = startPos.hx_a + t * (endPos.hx_a - startPos.hx_a);
        interpPos.hx_b = startPos.hx_b + t * (endPos.hx_b - startPos.hx_b);
        interpPos.hx_c = startPos.hx_c + t * (endPos.hx_c - startPos.hx_c);

        // calculate servo angles using the interpolated positions
        movOK = 0;
        if (which_platform & (1 << 3)) movOK += hx_servo.calcServoAngles(interpPos, servo_angles0, 0);
        if (which_platform & (1 << 2)) movOK += hx_servo.calcServoAngles(interpPos, servo_angles1, 1);
        if (which_platform & (1 << 1)) movOK += hx_servo.calcServoAngles(interpPos, servo_angles2, 2);
        if (which_platform & (1 << 0)) movOK += hx_servo.calcServoAngles(interpPos, servo_angles3, 3);

        hx_servo.updateServos(movOK);

        delay(50);
    }

    // move to the final position
    movOK = 0;
    if (which_platform & (1 << 3)) movOK += hx_servo.calcServoAngles(endPos, servo_angles0, 0);
    if (which_platform & (1 << 2)) movOK += hx_servo.calcServoAngles(endPos, servo_angles1, 1);
    if (which_platform & (1 << 1)) movOK += hx_servo.calcServoAngles(endPos, servo_angles2, 2);
    if (which_platform & (1 << 0)) movOK += hx_servo.calcServoAngles(endPos, servo_angles3, 3);
    hx_servo.updateServos(movOK);
}

/** 
 * @brief Modified readSerial() function for parsing data strings
 * 
 * This function should be called once data is read to be read in the serial bus. It looks for
 * new line or carriage return characters and breaks once they are received, and trims the
 * received line.
 * 
 * @return the received string, trimmed
 */
String readSerialString() {
  String receivedString = "";
  while (SerialPort.available()) {
    char c = SerialPort.read();
    if (c == '\r' || c == '\n') {
      break;
    }
    receivedString += c;
    delay(2);
  }
  receivedString.trim();
  return receivedString;
}

/**
 * @brief Converts a binary value to a decimal value, both as ints
 * 
 * This function converts an int represented as a binary value into its decimal equivalent.
 * For example, an input of 1111 would return 15, and an input of 101 would return 5.
 * 
 * This function was created by geeksforgeeks.org in their article "Program for Binary To 
 * Decimal Conversion".
 * 
 * @see https://www.geeksforgeeks.org/program-binary-decimal-conversion/
 * 
 * @param n binary int input
 * @return 
 */
int binaryToDecimal(int n)
{
    int num = n;
    int dec_value = 0;
 
    // Initializing base value to 1, i.e 2^0
    int base = 1;
 
    int temp = num;
    while (temp) {
        int last_digit = temp % 10;
        temp = temp / 10;
 
        dec_value += last_digit * base;
 
        base = base * 2;
    }
 
    return dec_value;
}

/**
 * @brief Sends a float over Serial to the STM32 with checksums
 * 
 * This function is a robust way to send a float over Serial to the STM32 microcontroller.
 * It uses checksums (which are the sum of the values of each character decimal representation)
 * to ensure that the message was not corrupted. Additionally, start and end markers are used
 * to aid in decoding.
 * 
 * For my setup, messages sent to the STM32 were frequently corrupted, but messages received 
 * were rarely corrupted.
 * 
 * @param value float to be sent over Serial
 */
void sendRobustFloat(float value) {
  char message[50];
  int messageLength = snprintf(message, sizeof(message), "%.4f", value);
  
  int checksum = 0;
  for (int i = 0; i < messageLength; i++) {
    checksum += message[i];
  }
  checksum %= CHECKSUM_MODULUS;

  SerialPort.print(START_MARKER);
  SerialPort.print(message);
  SerialPort.print(',');
  SerialPort.print(checksum);
  SerialPort.print(END_MARKER);
  SerialPort.println();  

  // ensure transmission is complete
  SerialPort.flush();
}