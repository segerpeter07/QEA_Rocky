// This example shows how to make a Balboa balance on its two
// wheels and drive around while balancing.
//
// To run this demo, you will need to install the LSM6 library:
//
// https://github.com/pololu/lsm6-arduino
//
// To use this demo, place the robot on the ground with the
// circuit board facing up, and then turn it on.  Be careful to
// not move the robot for a few seconds after powering it on,
// because that is when the gyro is calibrated.  During the gyro
// calibration, the red LED is lit.  After the red LED turns off,
// turn the robot so that it is standing up.  It will detect that
// you have turned it and start balancing.
//
// Alternatively, you can press the A button while the robot is
// lying down and it will try to use its motors to kick up into
// the balancing position.
//
// This demo is tuned for the 50:1 high-power gearmotor with
// carbon brushes, 45:21 plastic gears, and 80mm wheels; you will
// need to adjust the parameters in Balance.h for your robot.
//
// After you have gotten the robot balance well, you can
// uncomment some lines in loop() to make it drive around and
// play a song.

#include <Balboa32U4.h>
#include <Wire.h>
#include <LSM6.h>
#include "Balance.h"


LSM6 imu;
Balboa32U4Motors motors;
Balboa32U4Encoders encoders;
Balboa32U4Buzzer buzzer;
Balboa32U4ButtonA buttonA;

uint32_t cur_time = 0;
// uint32_t prev_time = 0;    //TODO might need to change
float prev_time = -1.0;
extern int32_t distanceLeft;
extern int32_t distanceRight;
int lastCountsLeft = 0;
int lastCountsRight = 0;
float errorL = 0;
float errorR = 0;
float errorSumL = 0.0;
float errorSumR = 0.0;
float P_L = 0;
float P_R = 0;
float errorT = 0;
float errorSumT = 0.0;


extern int32_t angle_accum;

void lyingDown();
extern bool isBalancingStatus;
extern bool balanceUpdateDelayedStatus;

#define STATE_WAITING 0
#define STATE_TESTING 1
#define STATE_TERMINATING 2
#define ANGLE_CORRECTION (-0.029)
#define METERS_PER_CLICK 3.141592*80.0*(1/1000.0)/12.0/(162.5)

int state = STATE_WAITING;
float Kp_V = 500;
float Ki_V =  5000;
float Kd_V = 0;
float Kp_T = -6;
float Ki_T = -36;
float Kd_T = 0;
float Ki_Q = -912;
float Kp_Q = 9.3;
float desired_speed;
float desired_angle = 0.0;

float previous_vL = 0;
float previous_vR = 0;
float previous_angle = 0;
float v_sum_L = 0;
float v_sum_R = 0;

void setup()
{
  // Uncomment these lines if your motors are reversed.
  // motors.flipLeftMotor(true);
  // motors.flipRightMotor(true);

  Serial.begin(9600);
  prev_time = 0;
  ledYellow(0);
  ledRed(1);
  balanceSetup();
  ledRed(0);
  angle_accum = 0;
  ledGreen(0);
  ledYellow(0);
}



void UpdateSensors()
{
  static uint16_t lastMillis;
  uint16_t ms = millis();

  // Perform the balance updates at 100 Hz.
  //if ((uint16_t)(ms - lastMillis) < UPDATE_TIME_MS) { return; }
  balanceUpdateDelayedStatus = ms - lastMillis > UPDATE_TIME_MS + 1;
  lastMillis = ms;

  // call functions to integrate encoders and gyros
  balanceUpdateSensors();

  if (imu.a.x < 0)
  {
    lyingDown();
    isBalancingStatus = false;
  }
  else
  {
    isBalancingStatus = true;
  }
}


float getAngleRad()
{
  // calculate the angle in radians.
  return ((float)angle) / 1000 / 180 * 3.14159 - ANGLE_CORRECTION;
}


// You need to wait a few seconds until the red LED on the robot goes off before starting, for the gyro calibration

void loop()
{
  /* ----- Wait for button to be pressed ----- */
  if (buttonA.isPressed()) {
    buttonA.waitForRelease();
    delay(1000);
    if (state == 0) {
      state = 1;
    } else {
      state++;
      motors.setRightSpeed(0);
      motors.setLeftSpeed(0);
      return;
    }
  }

  if (state == 1) {
    cur_time = millis();

    float delta_t = (cur_time - prev_time) / 1000.0;

    // handle the case where this is the first time through the loop
    if (prev_time == 0) {
      delta_t = 0.01;
    }

    if (cur_time - prev_time > UPDATE_TIME_MS) {
      /* ----- DEFINE Vars ----- */
      static float angle_rad = 0;              // this is the angle in radians
      int currentCountsLeft, currentCountsRight;     // track the number of rotations of the wheels
      float vL, vR;

      /* ----- Get angle in rad ----- */
      UpdateSensors();
      angle_rad = getAngleRad();

      prev_time = cur_time;


      /* ----- Clear error if angle = 0 ----- */
      if (angle_rad == 0) {
        errorSumT = 0;
      }


      /* ----- Get desired velocity using angle ----- */
      errorT = desired_angle - angle_rad;
      errorSumT = errorSumT + (errorT * delta_t);
      desired_speed = (Kp_T * errorT) + (Ki_T * errorSumT) + Kd_T*(angle_rad - previous_angle)/delta_t;

      /* ----- Get velocity in m/s ----- */

      currentCountsLeft = encoders.getCountsLeft();
      currentCountsRight = encoders.getCountsRight();

      vL = (currentCountsLeft - lastCountsLeft) / delta_t*METERS_PER_CLICK;
      vR = (currentCountsRight - lastCountsRight) / delta_t*METERS_PER_CLICK;

      distanceLeft += (currentCountsLeft - lastCountsLeft) * METERS_PER_CLICK;
      distanceRight += (currentCountsRight - lastCountsRight) * METERS_PER_CLICK;

      lastCountsLeft = currentCountsLeft;
      lastCountsRight = currentCountsRight;

      /* ----- Calculate feedback loop error ----- */
      float out_L = desired_speed - (Kp_Q*vL + Ki_Q*v_sum_L);
      float out_R = desired_speed - (Kp_Q*vR + Ki_Q*v_sum_R);


      /* ----- Calculate drive commands ----- */
      errorL = out_L - vL;
      errorR = out_R - vR;
      errorSumL += errorL * delta_t;
      errorSumR += errorR * delta_t;


      P_L = (Kp_V * errorL) + (Ki_V * errorSumL) + Kd_V*(vL - previous_vL)/delta_t;
      P_R = (Kp_V * errorR) + (Ki_V * errorSumR) + Kd_V*(vR - previous_vR)/delta_t;

      if (angle_rad == 0) {
        P_L = 0;
        P_R = 0;
      }

      previous_vL = vL;
      previous_vR = vR;
      previous_angle = angle_rad;

      v_sum_L = v_sum_L + vL;
      v_sum_R = v_sum_R + vR;

      /* ----- Give drive commands to motors ----- */
      motors.setRightSpeed(P_R);
      motors.setLeftSpeed(P_L);

      /* ----- Print Output (for testing) ----- */
      Serial.print(cur_time / 1000.0);
      Serial.print("  ds: ");
      Serial.print(desired_speed);
      //Serial.print(currentCountsLeft);
      //Serial.print(" ");
      //Serial.print(currentCountsRight);
      //Serial.print(" ");
      //        Serial.print(vL);
      //        Serial.print("  vR: ");
      //        Serial.print(vR);
      //        Serial.print(" P_L: ");
      //        Serial.print(P_L);
      //        Serial.print(" P_L: ");
      //        Serial.print(P_R);
      //        Serial.print(" errorL: ");
      //        Serial.print(errorL);
      //        Serial.print(" errorR:");
      //        Serial.print(errorR);
      Serial.print(" angle: ");
      Serial.print(angle_rad);
      Serial.print(" Error_T: ");
      Serial.print(errorT);
      Serial.print(" ErrorSumT: ");
      Serial.print(errorSumT);
      Serial.print(" dt: ");
      Serial.println(delta_t);
      //Serial.print(" ");
      //Serial.println(errorSumL);
    } else {
      return;
    }
  }
}
