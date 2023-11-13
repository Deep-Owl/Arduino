/*
 *  K.I.S.S. (KEEP IT SIMPLE STUPID)
 *  Author: Devin Powell
 *  Version: October 24 2023
 *
 * IMPORTANT NOTES:
 *  - Some ultrasonic sensors are giving different readings with the same code/hardware, this should be accounted for later on.
 *  - OSEPP Motor Shield uses digital pins 3, 4, 5, 6, 7, 8, 11, and 12 to drive motors. They cannot be remapped.
 *  - Accelerometer component uses designated SCL and SDA pins on Uno and Mega
 *  - Playing with baud rate could affect the performance of modules and processing speed:
 *      - As of Sep 5th we are using 115200 because it is used in many examples with accelerometers and motor encoders
 *  - Deactivating and removing the WDT (watchdog timer) could solve issues with complex libraries
 *
 *  ========================================================================================================================= 
 *  |                                                      Information:                                                     |
 *  =========================================================================================================================      
 *
 *  This code controls the arduino for the cyclops robot. 
 *  In this version neck movement is dependent on data provided by an ADXL 345 accelerometer and 2 Quadrature Hall-Effect Motor Encoders.
 *  The core behavior of this code is that the robot will use data from sensors to explore its surroundings.
 *  Objects and obstacles in the environment are detected via ultrasonic and IR sensors.
 *
 *  ADXL 345:
 *  ---------
 *    - Accelerometer is used to move neck and reset it to a nuetral position
 *    - In order for ADXL to work, the neck must be in a nueetral position (upright) before running this program
 *
 *  Watchdog Timer:
 *  ---------------
 *    - Software component that enables a hardware reset trigged by hardware/software errors
 *    - Resets chip when the battery voltage is insufficient / unexpected program error / infinite loop
 *    - If wdt_reset() is not called within the specified threshold the hardware will reset
 *    - Current Watchdog timer threshold: 8 seconds
 *    - TODO: Find a good timing threshold, 8 seconds may be too long. Original code used 1 second.
 *        - Options are: ______________________ TODO: find the timing options for WTD
 *
 *
 *  OSEPP & Specialized Libraries:
 *  ------------------------------
 *   - To use libraries: drag and drop the folder with name of desired library into Arduino>libraries
 *   - Any libraries which are not provided by Arduino IDE by default have been uploaded to google Drive folder "Libraries"
 *   - OSEPP Libraries can also be downloaded by visiting https://www.osepp.com/downloads/kits/OSEPP-Mechanical-Kit-Codes.zip
 *   - IRremote library is not the version provided by Arduino IDE library database, instead it comes from ELEGOO's Super Starter Kit
 *
 *  Ultrasonic Sensor Data Handling:
 *  --------------------------------
 *   - If needed the library can be changed or removed for the sake of simplicity
 *   - How it works (sensorsDRV version):
 *      1.  sonicAvg is called which returns the average of the sonic data over 5 pings
 *      2a. If no detection occurs: sonicAvg will return the value 5000
 *      2b. If detection occurs:    sonicAvg will return a value in the range of [3000, 3500]
 *         i.  A flag is triggered marking that something has been detected (currently not used)
 *      3.  If the value is less than 3100, an obstacle flag is triggered
 *
 *
 *  Sharp IR Sensor Data Handling:
 *  --------------------------------
 *   - No library is needed for the sharp IR sensors
 *   1. Sharp IR sensor data is pull via analogRead() statements
 *   2. The data ranges from [0, 650]
 *      i.  High values: Object is close to the sensor
 *      ii. Low values:  Environment clear of obstacles
 *   3. If sensor data >= 250, an obstacle flag is triggered
 *
 *
 * =========================================================
 * |                      Hardware:                        |
 * =========================================================
 *
 *       - 1 Arduino Mega 2560 R3
 *       - 1 OSEPP Motor Shield - 6612 (TB6612 IC chip)
 *       - 4 OSEPP LS-00041 High Torque Electric Motors
 *       - 2 OSEPP Hall Effect Encoders
 *            - A3144 Sensors (2 per encoder)
 *            - 11 Pole Magnetic Wheel (assumably)
 *       - ADXL 345 Accelerometer
 *       - KY-022 Infrared Reciever Module
 *
 *
 * ================================================================
 * |                            Pins:                             |
 * ================================================================
 *
 * ADXL Power/Ground: 
 *   - Brown stripe: SCL 
 *   - Orange stripe: SDA
 *   - Solid orange: 3V
 *   - Solid brown: GRND
 *
 *          Analog:                              Digital:         
 *  -----------------------           -----------------------------
 *  A0:  ----AVAILABLE----            0   (Rx):  ----AVAILABLE---- 
 *  A1:  ----AVAILABLE----            1   (Tx):  ----AVAILABLE----
 *  A2:  GP2Y0A Sharp IR 1            2 (INTR):  ----AVAILABLE---- <- This could be IR Reciever (currently here 10/24/23)
 *  A3:  GP2Y0A Sharp IR 2            3  (PWM):  Motor 2 PWM  
 *  A4:  ----AVAILABLE----            4  (PWM):  Motor 4 Direction
 *  A5:  ----AVAILABLE----            5  (PWM):  Motor 4 PWM
 *  A6:  ----AVAILABLE----            6  (PWM):  Motor 3 PWM
 *  A7:  ----AVAILABLE----            7  (PWM):  Motor 7 Direction
 *  A8:  ----AVAILABLE----            8  (PWM):  Motor 2 Direction
 *  A9:  ----AVAILABLE----            9  (PWM):  Ultrasonic Trigger
 *  A10: ----AVAILABLE----            10 (PWM):  Ultrasonic Echo
 *  A11: ----AVAILABLE----            11 (PWM):  Motor 1 PWM
 *  A12: ----AVAILABLE----            12 (PWM):  Motor 1 Direction
 *  A13: ----AVAILABLE----            13 (PWM):  ----AVAILABLE---- <- This could be IR Reciever
 *  A14: ----AVAILABLE----            14  (Tx):  ----AVAILABLE---- 
 *  A15: ----AVAILABLE----            15  (Rx):  ----AVAILABLE---- 
 *                                    16  (Tx):  ----AVAILABLE---- 
 *                                    17  (Rx):  ----AVAILABLE---- 
 *                                    18(INTR):  Motor 1 Encoder A
 *                                    19(INTR):  Motor 2 Encoder A
 *                                    20 (SDA):  ADXL345 SDA
 *                                    21 (SLC):  ADXL345 SLC
 *                                    22 - 53 : ----AVAILABLE---- <- This could be IR Reciever
 *
 *
 *
 *
 * ====================================================================
 * |                         Project Diagram                          |
 * ====================================================================
 *
 *  KEY:
 *  ------------------------------
 *  +- : Power Supply
 *  Ar : Arduino Mega + Motor Shield
 *  M1 : Motor 1 + Encoder A
 *  M2 : Motor 2 + Encoder B
 *
 *                          Front = 1
 *                             -Y
 *  
 *                            +- USB
 *                           -------         
 *                        //   IR1   \\
 *                      /               \
 *         Left = 4    |                 |    Right = 2
 *            +X      ||    M2  Ar  M1   ||      -X
 *                     |                 |
 *                      \               /
 *                        \\   IR2   //
 *                           -------
 *
 *                          Back = 3
 *                             +Y
 */


//+=+-+=+=+-+=+=+-+=+=+-+=+=+-+=+=+-+=+=+-+=+=+-+=+=+-+=+=+-+=+=+-+=+//
//  DEBUG MODE: When true, prints all variables as they are updated  //
//+=+-+=+=+-+=+=+-+=+=+-+=+=+-+=+=+-+=+=+-+=+=+-+=+=+-+=+=+-+=+=+-+=+//

//bool debugMode = true;  //  ON
bool debugMode = false;  //  OFF

//+=+-+=+=+-+=+=+-+=+=+-+=+=+-+=+=+-+=+=+-+=+=+-+=+=+-+=+=+-+=+=+-+=+//



//============================================================================================================================//
//                                                             Libraries                                                      //
//============================================================================================================================//

#include "TBMotor.h"       // OSEPP Motor Library: Each motor requires 2 digital pins for PWM control and directional control
#include "Arduino.h"       // Hall Effect Encoder Library
#include "pins_arduino.h"  // Hall Effect Encoder Library
#include "sensorsDRV.h"    // OSEPP Sensor Library:                      Ultrasonic
#include <SPI.h>           // Serial Peripheral Interface Library:       Sharp IR
#include <avr/wdt.h>       // Watchdog timer (WDT) library
#include <RH_ASK.h>        // Radio Library
#include <Wire.h>          // I2C Library (ADXL345)
#include <ADXL345_WE.h>    // Accelerometer Library
#include <IRremote.h>      // IR Send/Recieve Library


//=========================================================================================================//
//                              KY-022 Infrared Reciever Variables                                         //
//=========================================================================================================//

const int IR_rx_pin = 2;  // Using 13 offers PWM which could be useful, however may be unessecary.
                          // could be that any digital pin will work for this project.

IRrecv irrecv(IR_rx_pin);  // IR reciever object

decode_results results;  // Object that translates data to hex value for debugging

const unsigned long TIMEOUT = 200;  // Adjust this value if needed (not sure what this does yet)



// Number that corresponds to robot behavior
// should be reset every time action is finished
int IR_code = -1;  // -1 by default

//Variable for storing behavior types for mapping to IR remote code
/*enum behaviorType {
  rest,        //  0
  explore,     //  1
  wiggle,      //  2
  rotateNeck,  //  3
  wagY,        //  4
  wagX,        //  5
  spinCW,      //  6
  spinCCW      //  7
};*/

int actionID = 0;
int randNum = -1;  // Should be generated by calling: randNum = random(X); where X = desired range + 1

//=========================================================================================================//
//                                 ADXL 345 Accelerometer Variables                                        //
//=========================================================================================================//


#define ADXL345_I2CADDR 0x53                          // Set I2C communication address
ADXL345_WE adxlSensor = ADXL345_WE(ADXL345_I2CADDR);  // Create ADXL345 Object with ADXL345_WE library
xyzFloat adxlAngles;                                  // Variable for storing ADXL345 Data

// ADXL Limit variables, used for controlling how far the neck should move
float yMax = 85.00,
      yMin = -85.00,
      xMax = 85.00,
      xMin = -85.00;

//=========================================================================================================//
//                                           Motor Variables                                               //
//=========================================================================================================//

// OSEPP motor object (pwm pin, direction pin)
OseppTBMotor Motor1(12, 11);  // Neck 1
OseppTBMotor Motor2(8, 3);    // Neck 2
OseppTBMotor Motor3(7, 6);    // Left wheel
OseppTBMotor Motor4(4, 5);    // Right wheel

//  Driving speed variables (MIN: 0, MAX: 255)
int leftSpeed = 0;
int rightSpeed = 0;
int driveSpeed = 100;
int neckSpeed = 90;  //  50 seems good, 70 is good

int neckPos = 0;
//            0 = center
//            1 = forward
//            2 = forwardRight
//            3 = right
//            4 = backwardRight
//            5 = backward
//            6 = backwardLeft
//            7 = left
//            8 = forwardLeft

// Parameters for controlling motor direction for both neck and wheel motors
// NOTE: could create separate set of directions for neck and wheels, maybe unnecessary
enum moveDir {
  forward,
  forwardLeft,
  forwardRight,
  rotateCW,
  rotateCCW,
  backward,
  backwardLeft,
  backwardRight,
  left,
  right
};


//=========================================================================================================//
//                                    Hall Effect Encoder Variables                                        //
//=========================================================================================================//

//  TODO: RENAME AND COMMENT THESE GLOBAL VARIABLES, CHANGE INT TO LONG(maybe)
#define encoder_1 18
#define encoder_2 19
volatile int count_1 = 0;
volatile int count_2 = 0;
volatile int countRot_1 = 0;
volatile int countRot_2 = 0;
volatile int totalRot_M1 = 0;
volatile int totalRot_M2 = 0;


//=========================================================================================================//
//                                          Ultrasonic Variables                                           //
//=========================================================================================================//

Ultrasonic ults(9, 10);              // (trig, echo)
const float sonicObstThresh = 3100;  // Threshold for obstacle detection: 3100 seems good - may be a bit high
int sonicData;                       // Variable holds ultrasonic data
bool sonicDetection = false;         // Flag: Something detected by sensor - may not be an obstacle (Unused for now)
bool obstacleUS = false;             // Flag: Obstacle (to be avoided) detected by ultrasonic sensor


//=========================================================================================================//
//                                        GP2Y0A Sharp IR Variables                                        //
//=========================================================================================================//

const int sharpIR1 = A2,
          sharpIR2 = A3;

const int distThresh = 230;  // Threshold for obstacle detection

int frontIR = 0,  // Variables hold IR data
  backIR = 0;

bool obstacleF = false,  // Front IR Obstacle switch
  obstacleB = false;     // Rear IR Obstacle switch


//=========================================================================================================//
//                                               Main Setup                                                //
//=========================================================================================================//

void setup() {
  //  Reset chip when the battery voltage is insufficient / unexpected program error / infinite loop
  //  Setup  watchdog with reset threshold of 8 seconds
  wdt_enable(WDTO_8S);
  wdt_reset();

  Wire.begin();          //  Enable Accelerometer SCL & SDA for data transfer
  Serial.begin(115200);  //  115200 used in OSEPP code, 38400 may work, 9600 as well - testing needed

  Serial.println("\n");
  Serial.println("MEGA 2560 R3 SETUP");
  Serial.println("==================\n");

  //=============================//
  //    ADXL345 Accelerometer    //
  //=============================//

  if (!adxlSensor.init()) Serial.print("ADXL345: ERROR");  //Verify ADXL is working
  else Serial.print("ADXL345: INITIALIZED");

  Serial.println("\nCalibrating Neck: Ensure neck is in nuetral position");

  delay(3000);  // 3 Second delay allows for manually reseting neck

  //adxlSensor.setCorrFactors(-260, 285, -280, 255, -120, 201);  // Calibrate offset for robot 2
  //adxlSensor.setCorrFactors(min x, max x, min y, max y, min z, max z)
  /*  Max and Min calibration values for the cyclops as of September 5th
   *  setCorrFactors calibrates the slope and assumes the zero point is (min+max)/2.
   *  These values will be different when a new robot iteration is constructed
   */

  adxlSensor.measureAngleOffsets();               // Measure angle offset
  adxlSensor.setDataRate(ADXL345_DATA_RATE_100);  // Set Data rate to 50 Hz (Options: 50, 100, 200 400, 800, 1600, 3200)
  adxlSensor.setRange(ADXL345_RANGE_8G);          // Set acceleration sesnitivity (8G or 16G)
  Serial.println("Calibration Complete");


  //================================//
  //            Sharp IR            //
  //================================//

  // Sharp IR Setup:
  pinMode(sharpIR1, INPUT);
  pinMode(sharpIR2, INPUT);

  //================================//
  //         Motors & Encoder       //
  //================================//

  // Adjust PWM timer frequency for controlling the motors and low frequency interference
  TCCR2B &= ~7;
  TCCR2B |= 1;

  pinMode(encoder_1, INPUT);
  pinMode(encoder_2, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoder_1), pulse_1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_2), pulse_2, RISING);

  sei();  // enable global interrupts


  irrecv.enableIRIn();         // Enable IR Reciever
  randomSeed(analogRead(A0));  // Create random generator based on analog input
  Serial.println();
}

//=========================================================================================================//
//                                               Main Loop                                                 //
//=========================================================================================================//

void loop() {
  wdt_reset();      //  Reset Watchdog Timer
  delay(3000);      //  Delay (usually 1000)
  getData();        //  Retrieve data
  //adxlResetNeck();  //  Reset Neck
  rotateNeckAndSpin(10000);
  //debug();
}

void rotateNeckAndSpin(long milliseconds) {
  wdt_reset();  //  Reset Watchdog Timer
  neckPos = findNeckPos();
  long performTime = milliseconds;
  unsigned long startTime;        // initialize start time
  unsigned long elapsedTime = 0;  // declare elsaped time

  if (neckPos == 1) {                   //  Neck is forward, start right
    startTime = millis();               //  Declare start time
    wheelDrive(rotateCW, driveSpeed);  //  Begin Spinning

    while (elapsedTime < performTime) {  //  While Loop

      resetEncoders();
      moveNeck(right, neckSpeed);
      while (adxlAngles.x > -60 && count_1 < 450) {
        getData();
        wdt_reset();
      }

      elapsedTime = millis() - startTime;
      if (elapsedTime > performTime) { break; }

      resetEncoders();
      moveNeck(backward, neckSpeed);
      while (adxlAngles.y < 50 && count_1 < 450) {
        getData();
        wdt_reset();
      }

      elapsedTime = millis() - startTime;
      if (elapsedTime > performTime) { break; }

      resetEncoders();
      moveNeck(left, neckSpeed);
      while (adxlAngles.x < 80 && count_1 < 450) {
        getData();
        wdt_reset();
      }

      elapsedTime = millis() - startTime;
      if (elapsedTime > performTime) { break; }

      resetEncoders();
      moveNeck(forward, neckSpeed);
      while (adxlAngles.y > -70 && count_1 < 450) {
        getData();
        wdt_reset();
      }

      elapsedTime = millis() - startTime;
    }
  }

  else if (neckPos == 5) {              //  Neck is backward, start left
    startTime = millis();               //  Declare start time
    wheelDrive(rotateCW, driveSpeed);  //  Begin Spinning

    while (elapsedTime < performTime) {  //  While Loop

      resetEncoders();
      moveNeck(left, neckSpeed);
      while (adxlAngles.x < 80 && count_1 < 450) {
        getData();
        wdt_reset();
      }

      elapsedTime = millis() - startTime;
      if (elapsedTime > performTime) { break; }

      resetEncoders();
      moveNeck(forward, neckSpeed);
      while (adxlAngles.y > -70 && count_1 < 450) {
        getData();
        wdt_reset();
      }

      elapsedTime = millis() - startTime;
      if (elapsedTime > performTime) { break; }

      resetEncoders();
      moveNeck(right, neckSpeed);
      while (adxlAngles.x > -60 && count_1 < 450) {
        getData();
        wdt_reset();
      }

      elapsedTime = millis() - startTime;
      if (elapsedTime > performTime) { break; }

      resetEncoders();
      moveNeck(backward, neckSpeed);
      while (adxlAngles.y < 50 && count_1 < 450) {
        getData();
        wdt_reset();
      }

      elapsedTime = millis() - startTime;
    }
  }

  else {  //Neck is probably nuetral, move forward and start right
    resetEncoders();
    moveNeck(forward, neckSpeed);
    while (adxlAngles.y > yMin && count_1 < 450) {
      getData();
      wdt_reset();
    }
    startTime = millis();               //  Declare start time
    wheelDrive(rotateCW, driveSpeed);  //  Begin Spinning

    while (elapsedTime < performTime) {  //  While Loop
      elapsedTime = millis() - startTime;

      resetEncoders();
      moveNeck(right, neckSpeed);
      while (adxlAngles.x > -60 && count_1 < 450) {
        getData();
        wdt_reset();
      }

      elapsedTime = millis() - startTime;
      if (elapsedTime > performTime) { break; }

      resetEncoders();
      moveNeck(backward, neckSpeed);
      while (adxlAngles.y < 50 && count_1 < 450) {
        getData();
        wdt_reset();
      }

      elapsedTime = millis() - startTime;
      if (elapsedTime > performTime) { break; }

      resetEncoders();
      moveNeck(left, neckSpeed);
      while (adxlAngles.x < 80 && count_1 < 450) {
        getData();
        wdt_reset();
      }

      elapsedTime = millis() - startTime;
      if (elapsedTime > performTime) { break; }

      resetEncoders();
      moveNeck(forward, neckSpeed);
      while (adxlAngles.y > -70 && count_1 < 450) {
        getData();
        wdt_reset();
      }

      elapsedTime = millis() - startTime;
    }
  }

  stopWheels();
  adxlResetNeck();
}

/* Function makes robot spin its neck in a circle based on random input seeds
 *  - Randomly looks forward or backwards
 *  - Randomly spins clockwise or counter-clockwise
 *  - Completes a random number of full rotations (between 1 and 3)
 *  - Resets neck upon completion
 */
void randRotateNeck() {
  wdt_reset();  //  Reset Watchdog Timer
  int rotateNum = random(3);
  randNum = random(2);  // Set random - Force 0 or 1 to debug

  if (randNum == 0) {     //  Rotate CW
    randNum = random(2);  //  Reset Random - Force 0 or 1 to debug

    if (randNum == 0) {  //  Start Front

      resetEncoders();
      moveNeck(forward, neckSpeed);
      while (adxlAngles.y > yMin && count_1 < 450) {
        getData();
        wdt_reset();
      }
      debug();

      for (int i = 0; i <= rotateNum; i++) {

        resetEncoders();
        moveNeck(right, neckSpeed);
        while (adxlAngles.x > -60 && count_1 < 450) {
          getData();
          wdt_reset();
        }
        //debug();

        resetEncoders();
        moveNeck(backward, neckSpeed);
        while (adxlAngles.y < 50 && count_1 < 450) {
          getData();
          wdt_reset();
        }
        //debug();

        resetEncoders();
        moveNeck(left, neckSpeed);
        while (adxlAngles.x < 80 && count_1 < 450) {
          getData();
          wdt_reset();
        }
        //debug();

        resetEncoders();
        moveNeck(forward, neckSpeed);
        while (adxlAngles.y > -70 && count_1 < 450) {
          getData();
          wdt_reset();
        }
        //debug();
      }
    }

    else if (randNum == 1) {  //  Start Back
      resetEncoders();
      moveNeck(backward, neckSpeed);
      while (adxlAngles.y < yMax && count_1 < 450) {
        getData();
        wdt_reset();
      }
      debug();

      for (int i = 0; i <= rotateNum; i++) {
        resetEncoders();
        moveNeck(left, neckSpeed);

        while (adxlAngles.x < 80 && count_1 < 450) {
          getData();
          wdt_reset();
        }
        //debug();

        resetEncoders();
        moveNeck(forward, neckSpeed);
        while (adxlAngles.y > -70 && count_1 < 450) {
          getData();
          wdt_reset();
        }
        //debug();

        resetEncoders();
        moveNeck(right, neckSpeed);
        while (adxlAngles.x > -60 && count_1 < 450) {
          getData();
          wdt_reset();
        }
        //debug();

        resetEncoders();
        moveNeck(backward, neckSpeed);
        //while (adxlAngles.y < xMax && count_1 < 700)
        while (adxlAngles.y < 50 && count_1 < 425) {
          getData();
          wdt_reset();
        }
        //debug();
      }
    }
  }

  else if (randNum == 1) {  //  Rotate CCW
    randNum = random(2);    //  Reset Random

    if (randNum == 0) {  //  Start Front
      resetEncoders();
      moveNeck(forward, neckSpeed);
      while (adxlAngles.y > yMin && count_1 < 450) {
        getData();
        wdt_reset();
      }
      debug();

      for (int i = 0; i <= rotateNum; i++) {
        resetEncoders();
        moveNeck(left, neckSpeed);
        //while (adxlAngles.x < xMax && count_1 < 500) {
        while (adxlAngles.x < 80) {  //TODO: Check this count
          getData();
          wdt_reset();
        }
        debug();

        resetEncoders();
        moveNeck(backward, neckSpeed);
        //while (adxlAngles.y < xMax && count_1 < 700)
        while (adxlAngles.y < 50 && count_1 < 450) {  //TODO: Check this count
          getData();
          wdt_reset();
        }
        debug();

        resetEncoders();
        moveNeck(right, neckSpeed);
        while (adxlAngles.x > -60 && count_1 < 450) {  //TODO: Check this count
          getData();
          wdt_reset();
        }
        debug();

        resetEncoders();
        moveNeck(forward, neckSpeed);
        while (adxlAngles.y > -70 && count_1 < 450) {  //TODO: Check this count
          getData();
          wdt_reset();
        }
        debug();
      }
    }

    else if (randNum == 0) {  //  Start Back
      resetEncoders();
      moveNeck(backward, neckSpeed);
      while (adxlAngles.y < yMax && count_1 < 450) {
        getData();
        wdt_reset();
      }
      debug();


      for (int i = 0; i <= rotateNum; i++) {
        resetEncoders();
        moveNeck(right, neckSpeed);
        while (adxlAngles.x > -60 && count_1 < 450) {  //TODO: Check this count
          getData();
          wdt_reset();
        }
        debug();


        resetEncoders();
        moveNeck(forward, neckSpeed);
        while (adxlAngles.y > -70 && count_1 < 450) {  //TODO: Check this count
          getData();
          wdt_reset();
        }
        debug();

        resetEncoders();
        moveNeck(left, neckSpeed);
        //while (adxlAngles.x < xMax && count_1 < 500) {
        while (adxlAngles.x < 80) {  //TODO: Check this count
          getData();
          wdt_reset();
        }
        debug();

        resetEncoders();
        moveNeck(backward, neckSpeed);
        //while (adxlAngles.y < xMax && count_1 < 700)
        while (adxlAngles.y < 50 && count_1 < 425) {  //TODO: Check this count
          getData();
          wdt_reset();
        }
        debug();
      }
    }
  }
  adxlResetNeck();
}

/* The resetNeck function looks like a dance when it is called consecutively
 * Make the robot wiggle for a set duration of time
 *
 * @param milliseconds: amount of time that the robot should dance in milliseconds
 */
void timedWiggle(long milliseconds) {
  wdt_reset();
  long danceTime = milliseconds;
  unsigned long startTime = millis();  // Get the current time in milliseconds
  unsigned long elapsedTime = 0;       // declare elsaped time

  while (elapsedTime < danceTime) {
    getData();
    adxlResetNeck();
    elapsedTime = millis() - startTime;
  }
}

/* The resetNeck function looks like a dance when it is called consecutively
 * Make the robot wiggle and spin for a set duration of time
 *
 * @param milliseconds: amount of time that the robot should dance in milliseconds
 */

void timedWiggleSpin(long milliseconds) {
  wdt_reset();
  long danceTime = milliseconds;
  unsigned long startTime = millis();  // Get the current time in milliseconds
  unsigned long elapsedTime = 0;       // declare elsaped time

  wheelDrive(rotateCW, driveSpeed);
  while (elapsedTime < danceTime) {
    getData();
    adxlResetNeck();
    elapsedTime = millis() - startTime;
  }
  stopWheels();
}

void explore() {
  wdt_reset();  //  Reset Watchdog Timer
  getData();    //  Retrieve data'

  if (!obstacleF) {  // No obstacle in front

    resetEncoders();
    moveNeck(forward, neckSpeed);  // Look Forward
    wdt_reset();
    while (count_1 < 400) {
      getData();
    }
    stopNeck();  // Stop Neck
    delay(500);  // Short delay

    wheelDrive(forward, driveSpeed);  //begin driving
    while (!obstacleF && !obstacleUS) {
      wdt_reset();
      getData();

      if (obstacleF || obstacleUS) { break; }

      resetEncoders();
      moveNeck(left, neckSpeed);
      while (!obstacleF && !obstacleB && adxlAngles.x > -30 && count_1 < 500) { getData(); }
      stopNeck();
      wdt_reset();

      if (obstacleF || obstacleUS) { break; }

      resetEncoders();
      moveNeck(right, neckSpeed);
      while (!obstacleF && !obstacleUS && adxlAngles.x < 50 && count_1 < 500) { getData(); }
      stopNeck();
      wdt_reset();
    }
    stopWheels();

    if (obstacleUS || obstacleF) {  //  Celebrate by dancing for 5 seconds
      timedWiggleSpin(5000);
    }
  }

  else if (!obstacleB) {
    resetEncoders();
    moveNeck(backward, neckSpeed);  // Look backward - using 480 limit
    wdt_reset();                    // Reset Watchdog
    while (count_1 < 480) { getData(); }
    stopNeck();  // Stop neck
    delay(500);  // Short delay

    wheelDrive(backward, driveSpeed);  // Begin driving
    while (!obstacleB && !obstacleUS) {
      wdt_reset();  // Reset Watchdog
      getData();    // Retrieve data (obstacles)

      if (obstacleB || obstacleUS) { break; }  // Break for obstacle

      resetEncoders();                                                                        // Reset Encoders
      moveNeck(left, neckSpeed);                                                              // Look 'Left'
      while (!obstacleB && !obstacleB && adxlAngles.x > -30 && count_1 < 500) { getData(); }  // Retrieve Data while looking
      stopNeck();                                                                             // Stop neck
      wdt_reset();                                                                            // Reset Watchdog

      if (obstacleB || obstacleUS) { break; }  // Break for obstacle

      resetEncoders();                                                                        // Reset Encoders
      moveNeck(right, neckSpeed);                                                             // Look 'Right'
      while (!obstacleB && !obstacleUS && adxlAngles.x < 50 && count_1 < 500) { getData(); }  // Retrieve data while looking
      stopNeck();                                                                             // Stop neck
      wdt_reset();                                                                            // Reset Watchdog
    }
    stopWheels();  // Stop driving

    if (obstacleUS || obstacleB) {
      timedWiggleSpin(5000);
    }  // dance if something is found
  }

  else {
    // if trapped, then do a little dance
    //  TODO: This trapped behavior could be a 90 degree spin so that the robot can try to get unstuck by turing left or right
    timedWiggle(5000);
  }
}

// Testing program that moves neck in all directions
// Moves neck to a limit based on both ADXL and Encoder parameters
// Resets Neck after each movement
void testNeck() {
  wdt_reset();  //  Reset Watchdog Timer
  delay(1000);  //  5 Second Delay
  adxlResetNeck();
  getData();  //  Retrieve data from sensors
  delay(1000);

  resetEncoders();
  moveNeck(forward, neckSpeed);
  while (adxlAngles.y > -75.00 && count_1 < 350) {  //maybe 400
    getData();
    wdt_reset();
  }
  debug();
  stopNeck();
  delay(1000);
  adxlResetNeck();
  delay(1000);


  resetEncoders();
  moveNeck(forwardRight, neckSpeed);  //FR uses count_2 only
  while (adxlAngles.y > -75.00 && adxlAngles.x > -85.00 && count_2 < 400) {
    getData();
    wdt_reset();
  }
  debug();
  stopNeck();
  delay(1000);
  adxlResetNeck();
  delay(1000);

  resetEncoders();
  moveNeck(right, neckSpeed);
  while (adxlAngles.x > -85.00 && count_1 < 350) {
    getData();
    wdt_reset();
  }
  debug();
  stopNeck();
  delay(1000);
  adxlResetNeck();
  delay(1000);

  resetEncoders();
  moveNeck(backwardRight, neckSpeed);
  while (adxlAngles.x > -85.00 && adxlAngles.y < 90.00 && count_1 < 400) {
    getData();
    wdt_reset();
  }
  debug();
  stopNeck();
  delay(1000);
  adxlResetNeck();
  delay(1000);

  resetEncoders();
  moveNeck(backward, neckSpeed);
  while (adxlAngles.y < 90.00 && count_1 < 350) {
    getData();
    wdt_reset();
  }
  debug();
  stopNeck();
  delay(1000);
  adxlResetNeck();
  delay(1000);

  resetEncoders();
  moveNeck(backwardLeft, neckSpeed);
  while (adxlAngles.x < 78.00 && adxlAngles.y < 90.00 && count_2 < 400) {
    getData();
    wdt_reset();
  }
  debug();
  stopNeck();
  delay(1000);
  adxlResetNeck();
  delay(1000);

  resetEncoders();
  moveNeck(left, neckSpeed);
  while (adxlAngles.x < 78.00 && count_1 < 350) {
    getData();
    wdt_reset();
  }
  debug();
  stopNeck();
  delay(1000);
  adxlResetNeck();
  delay(1000);

  resetEncoders();
  moveNeck(forwardLeft, neckSpeed);
  while (adxlAngles.x < 78.00 && adxlAngles.y > -75.00 && count_1 < 400) {
    getData();
    wdt_reset();
  }
  debug();
  stopNeck();
}

/*  Uses the ADXL 345 to determine the location of the neck
 *  This function may be affected by changes to the robots physiology
 *  
 *  @return location: Integer value corresponding to 9 possible neck positions
 *  @precond: ADXL 345 must be calibrated correctly
 */
int findNeckPos() {
  wdt_reset();
  int location = -1;
  float x = adxlAngles.x;
  float y = adxlAngles.y;

  if (x > -10 && x < 10) {  //  Location: 0, 1, or 5
    if (y > 15) {
      location = 5;
    } else if (y < -15) {
      location = 1;
    } else {
      location = 0;
    }
  }

  else if (x < -10) {  //  Location: 2, 3, or 4
    if (y > 15) {
      location = 4;
    } else if (y < -15) {
      location = 2;
    } else {
      location = 3;
    }
  }

  else {  // Location: 6, 7, or 8
    if (y > 15) {
      location = 6;
    } else if (y < -15) {
      location = 8;
    } else {
      location = 7;
    }
  }

  return location;
}


/*  Recursive function responsible for returning neck to a nuetral position
 *  Moves neck based on neckPos global variable and accelerometer data
 *  Uses findNeckPos() helper method
 *  
 *  @precond: ADXL 345 must be calibrated to a nuetral position
 */
void adxlResetNeck() {

  neckPos = findNeckPos();

  if (neckPos == 0) { return; }  //  End if neck is in nuetral position

  else {
    switch (neckPos) {
      case 1:                                    //  Position: Forward
        moveNeck(backward, neckSpeed);           //  i.   Move in opposite direction
        while (adxlAngles.y < 0) { getData(); }  //  ii.  Continue moving and retrieve data until threshold achieved
        stopNeck();                              //  iii. Stop
        break;

      case 2:  //  Position: Forward-Right
        moveNeck(backwardLeft, neckSpeed);
        while (adxlAngles.y < 0 && adxlAngles.x < 0) { getData(); }
        stopNeck();
        break;

      case 3:  //  Position: Right
        moveNeck(left, neckSpeed);
        while (adxlAngles.x < 0) { getData(); }
        stopNeck();
        break;

      case 4:  //  Position: Backward-Right
        moveNeck(forwardLeft, neckSpeed);
        while (adxlAngles.y > 0 && adxlAngles.x < 0) { getData(); }
        stopNeck();
        break;

      case 5:  //  Position: Backward
        moveNeck(forward, neckSpeed);
        while (adxlAngles.y > 0) { getData(); }
        stopNeck();
        break;

      case 6:  //  Position: Backward-Left
        moveNeck(forwardRight, neckSpeed);
        while (adxlAngles.y > 0 && adxlAngles.x > 0) { getData(); }
        stopNeck();
        break;

      case 7:  //  Position: Left
        moveNeck(right, neckSpeed);
        while (adxlAngles.x > 0) { getData(); }
        stopNeck();
        break;

      case 8:  //  Position: Forward-Left
        moveNeck(backwardRight, neckSpeed);
        while (adxlAngles.y < 0 && adxlAngles.x > 0) { getData(); }
        stopNeck();
        break;
    }
    adxlResetNeck();  //  Recursive call
  }
}


/* Function for driving the wheels of the robot.
 * TODO: Write more directional control cases
 * 
 * @param direction: forward, forwardLeft, forwardRight, rotateCW, 
 *                   rotateCCW, backward, backwardLeft, backwardRight, stop
 *
 * @param speed: integer between 0 and 255
 */
void wheelDrive(moveDir direction, int speed) {
  switch (direction) {
    case forward:
      Motor3.SetSpeed(speed);
      Motor4.SetSpeed(speed);
      break;
    case backward:
      Motor3.SetSpeed(-speed);
      Motor4.SetSpeed(-speed);
      break;
    case rotateCW:
      Motor3.SetSpeed(speed);
      Motor4.SetSpeed(-speed);
      break;
    case rotateCCW:
      Motor3.SetSpeed(-speed);
      Motor4.SetSpeed(speed);
      break;
  }
}

void stopWheels() {
  Motor3.SetSpeed(0);
  Motor4.SetSpeed(0);
}


/* Function for controlling the neck motors
 * 
 * @param direction: forward, forwardLeft, forwardRight, rotateCW, 
 *                   rotateCCW, backward, backwardLeft, backwardRight
 *
 * @param speed: integer between 0 and 255
 */
void moveNeck(moveDir direction, int speed) {
  switch (direction) {
    case forward:
      Motor1.SetSpeed(speed);
      Motor2.SetSpeed(speed);
      break;
    case backward:
      Motor1.SetSpeed(-speed);
      Motor2.SetSpeed(-speed);
      break;
    case forwardLeft:
      Motor1.SetSpeed(0);
      Motor2.SetSpeed(speed);
      break;
    case forwardRight:
      Motor1.SetSpeed(speed);
      Motor2.SetSpeed(0);
      break;
    case backwardLeft:
      Motor1.SetSpeed(-speed);
      Motor2.SetSpeed(0);
      break;
    case backwardRight:
      Motor1.SetSpeed(0);
      Motor2.SetSpeed(-speed);
      break;
    case left:
      Motor1.SetSpeed(-speed);
      Motor2.SetSpeed(speed);
      break;
    case right:
      Motor1.SetSpeed(speed);
      Motor2.SetSpeed(-speed);
      break;
  }
}

void stopNeck() {
  Motor1.SetSpeed(0);
  Motor2.SetSpeed(0);
}

/*
 * Simple test to check that all motors are functioning correctly
 * Drives motors at minimum speed and incresses gradually to max speed param
 *
 * precond: minSpeed < maxSpeed
 *
 * @param minSpeed: minimum speed integer
 * @param maxSpeed: maxmimum speed integer
 */
void speedTest(int minSpeed, int maxSpeed) {

  int tempSpeed = minSpeed;


  while (tempSpeed < maxSpeed) {

    Serial.println("Speed: " + (String)tempSpeed);

    Motor1.SetSpeed(tempSpeed);
    Motor2.SetSpeed(tempSpeed);
    Motor3.SetSpeed(tempSpeed);
    Motor4.SetSpeed(tempSpeed);

    delay(100);

    tempSpeed++;
  }

  //cycle through motor speeds and blink built in LED
  while (tempSpeed > minSpeed) {

    Serial.println("Speed: " + (String)tempSpeed);

    Motor1.SetSpeed(tempSpeed);
    Motor2.SetSpeed(tempSpeed);
    Motor3.SetSpeed(tempSpeed);
    Motor4.SetSpeed(tempSpeed);


    delay(100);

    tempSpeed--;
  }
}

/* Core function of robot, pulls data from sensors
 * This function should be called frequently to ensure correct behavior
 * Can be called at the beginning of 'while' loops to check environmental conditions
 *
 * DEBUG MODE: if debugMode is on then all data from sensors will be printed when this function is called
 *
 */
void getData() {
  IR_updateCode();  //Check for IR signal recieved

  adxlAngles = adxlSensor.getCorrAngles();  // Get ADXL Data

  obstacleF = detectFront();
  obstacleB = detectBack();

  // Get Ultrasonic Reading, detect obstacle
  sonicData = sonicAvg();
  obstacleUS = detectSonic();

  // Alternatively, poll Ultrasonic data directly
  //sonicData = ults.Detect();
  //obstacleUS = detectSonic();

  //DEBUG MODE:
  if (debugMode) debug();
}

// Multiple sampling, take the average of ultrsonic data over 5 iterations
int sonicAvg() {
  float avgDist = 0;
  int i = 0;
  for (i = 0; i < 5; i++) avgDist += ults.Detect();
  return avgDist / i;
}

bool detectFront() {
  if (analogRead(sharpIR1) >= 250) {
    return true;
  }
  return false;
}

bool detectBack() {
  if (analogRead(sharpIR2) >= 250) {
    return true;
  }
  return false;
}

bool detectSonic() {
  if (sonicData <= sonicObstThresh) {
    return true;
  }
  return false;
}

// Displays all sensor data
void debug() {
  /*Serial.print("IR Code: ");
  Serial.println(IR_code);

  Serial.print("Front IR: " + (String)analogRead(sharpIR1));
  Serial.print('\t');
  Serial.println("Back IR: " + (String)analogRead(sharpIR2) + "\n");

  Serial.print("Obstacle Front: ");
  if (obstacleF) {
    Serial.print("TRUE\t");
  } else {
    Serial.print("FALSE\t");
  }

  Serial.print("Obstacle Back: ");
  if (obstacleB) {
    Serial.println("TRUE\n");
  } else {
    Serial.println("FALSE\n");
  }

  // Ultrasonic Data
  Serial.print("Ultrasonic: " + (String)sonicData);
  Serial.print('\t');
  Serial.print("Obstacle Ultrasonic: ");
  if (obstacleUS) {
    Serial.println("TRUE\n");
  } else {
    Serial.println("FALSE\n");
  }*/

  // ADXL data
  Serial.print("Angle X = ");
  Serial.print(adxlAngles.x);
  Serial.print("  |  Angle Y   = ");
  Serial.print(adxlAngles.y);
  Serial.print("  |  Angle Z   = ");
  Serial.println(adxlAngles.z);


  // Motor encoder data
  Serial.print("Motor 1: ");
  Serial.print(count_1);
  Serial.print("\t");
  Serial.print("Motor 2: ");
  Serial.println(count_2);

  // Neck Position data
  /*Serial.print("\nNeck Position: ");
  Serial.println(neckPos);
  Serial.println();*/
}


/* Very basic timed drive behavior
 * Robot will move in specified direction for specified amount of time and then stop
 *
 * @param direction: forward, forwardLeft, forwardRight, rotateCW, 
 *                   rotateCCW, backward, backwardLeft, backwardRight, stop
 * @param speed: motor speed integer between 0 and 255
 * @param seconds: amount of time in seconds (1 = 1 second, 60 = 1 minute)
 */
void timedDrive(moveDir direction, int speed, long milliseconds) {
  long driveTime = milliseconds;
  unsigned long startTime = millis();  // Get the current time in milliseconds
  unsigned long elapsedTime = 0;       // declare elsaped time

  //wheelDrive(direction, speed);
  moveNeck(direction, speed);

  while (elapsedTime < driveTime) {
    //getData();
    elapsedTime = millis() - startTime;
  }

  //wheelDrive(stop, 0);
  stopNeck();
}


void customDelay(long actionMillis) {
  unsigned long startTime = millis();  // Get the current time in milliseconds
  unsigned long elapsedTime = 0;       // declare elsaped time

  while (elapsedTime < actionMillis) {
    elapsedTime = millis() - startTime;
  }
}

/* Function for updating behavior code - updates via IR signal
 * TODO: Optimize - make it so the signal is lost left often
 *       Might not be able to optimize on this end
 *       IR signal should probably be sent multiple times from the source to ensure that the signal is properly recieved
 */
void IR_updateCode() {
  //Serial.println("Looking for signal...");
  if (irrecv.decode(&results)) {

    Serial.print("Signal recieved: ");
    Serial.println(results.value, HEX);

    // Adjust these according to your remote's buttons
    if (results.value == 0xFFA25D) {  //Code responds to power button on elegoo remote
      Serial.println("Power Button was pressed");
      IR_code = 0;
    }

    irrecv.resume();  // Ready to get the next code
  }
}



/* This function will be used to allow precision control of the neck motors.
 *
 * Interrupt Service Routine (ISR) for handling pin change interrupts on pins 18 and 19
 * The ISR is triggered automatically when a RISE is detected on any of the hall effect sensor pins
 * Constantly runs in the background
 * counter variables should be reset IMMEDIATELY BEFORE using the encoders to limit movement
 *
 * Updates the motor position variables: count_1 and count_2
 * Encoders initialize at 0, positive motion causes position variable to increase, reset encoders often
 */
/************************************************************/
//                   Motor 1 Encoder ISR                    //
/************************************************************/
void pulse_1() {
  count_1++;
}

void pulse_2() {
  count_2++;
}

void resetEncoders() {
  count_1 = 0;
  count_2 = 0;
}