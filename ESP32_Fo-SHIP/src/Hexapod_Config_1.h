/**
 * S T E W A R T    P L A T F O R M    O N    E S P 3 2
 *
 * Copyright (C) 2019  Nicolas Jeanmonod, ouilogique.com
 * Modified by Jakob Frabosilio, 2024
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#pragma once

/*
 * ======== SERVO SETTINGS ========
 */

#define NB_SERVOS 6

/*
 * Servo calibration values.
 * These values should not be used to restrict servo movements.
 * Use MIN/MAX COORDINATES below for that.
 */
const double SERVO_FULL_ANGULAR_RANGE = radians(180);
const double SERVO_HALF_ANGULAR_RANGE = SERVO_FULL_ANGULAR_RANGE / 2;
const int SERVO_MIN_PWM = 100;
const int SERVO_MAX_PWM = 500;

/*
 * Offset values in µs to compensate for arm angle errors.
 * For odd servos, positive is down; for even servos, negative is down;
 */
const int PW_OFFSET[] = {
    2,          // servo1, platform1 (BASE)
    -1,
    0,
    -19,
    -4,
    1,
    6,          // servo1, platform2
    -12,
    1,
    14,
    -4,
    11,
    16,          // servo1, platform3
    -11,
    -12,
    9,
    -5,
    -16,
    -8,          // servo1, platform4 (TOP)
    7,
    8,
    -2,
    -2,
    -17};

/*
 * The gain in µs/rad
 * Typical gain =~ 318 µs/rad =~ 5.6 µs/°
 */
const double gain = (SERVO_MAX_PWM - SERVO_MIN_PWM) /
                    (SERVO_FULL_ANGULAR_RANGE);

/*
 * calibration_t
 */
typedef struct
{
    double gain;
    int offset;
} calibration_t;

/*
 * Calibration factors. These values take into account the fact
 * that the odd and even arms are a reflection of each other.
 * The calibration is linear:
 * pulse width (µs) = gain (µs/rad) + offset (µs)
 */
const calibration_t SERVO_CALIBRATION[] = {
    {-gain, SERVO_MAX_PWM + PW_OFFSET[0]},
    {gain, SERVO_MIN_PWM + PW_OFFSET[1]},
    {-gain, SERVO_MAX_PWM + PW_OFFSET[2]},
    {gain, SERVO_MIN_PWM + PW_OFFSET[3]},
    {-gain, SERVO_MAX_PWM + PW_OFFSET[4]},
    {gain, SERVO_MIN_PWM + PW_OFFSET[5]},
    {-gain, SERVO_MAX_PWM + PW_OFFSET[6]},
    {gain, SERVO_MIN_PWM + PW_OFFSET[7]},
    {-gain, SERVO_MAX_PWM + PW_OFFSET[8]},
    {gain, SERVO_MIN_PWM + PW_OFFSET[9]},
    {-gain, SERVO_MAX_PWM + PW_OFFSET[10]},
    {gain, SERVO_MIN_PWM + PW_OFFSET[11]},
    {-gain, SERVO_MAX_PWM + PW_OFFSET[12]},
    {gain, SERVO_MIN_PWM + PW_OFFSET[13]},
    {-gain, SERVO_MAX_PWM + PW_OFFSET[14]},
    {gain, SERVO_MIN_PWM + PW_OFFSET[15]},
    {-gain, SERVO_MAX_PWM + PW_OFFSET[16]},
    {gain, SERVO_MIN_PWM + PW_OFFSET[17]},
    {-gain, SERVO_MAX_PWM + PW_OFFSET[18]},
    {gain, SERVO_MIN_PWM + PW_OFFSET[19]},
    {-gain, SERVO_MAX_PWM + PW_OFFSET[20]},
    {gain, SERVO_MIN_PWM + PW_OFFSET[21]},
    {-gain, SERVO_MAX_PWM + PW_OFFSET[22]},
    {gain, SERVO_MIN_PWM + PW_OFFSET[23]}};

/*
 * ======== GEOMETRY SETTINGS ========
 */

/*
 * Orientations of the servos arms relative to the X axis.
 */
const double THETA_S[] = {
    radians(-60),
    radians(120),
    radians(180),
    radians(0),
    radians(60),
    radians(-120),
    radians(-60),
    radians(120),
    radians(180),
    radians(0),
    radians(60),
    radians(-120),
    radians(-60), 
    radians(120),
    radians(180),
    radians(0),
    radians(60),
    radians(-120),
    radians(-60),
    radians(120),
    radians(180),
    radians(0),
    radians(60),
    radians(-120)};

/*
 * MIN/MAX COORDINATES
 * NOTE: The actual min and max for each DOF are interdependent. eg:
 * If the platform is pitched by some amount, the roll min/max will be physically
 * different than what’s defined here. These are just the absolute maximums under
 * ideal conditions (eg: max for roll when pitch is zero).
 */
const double HX_X_MIN = -5;
const double HX_X_MAX = 5;
const double HX_X_MID = (HX_X_MAX + HX_X_MIN) / 2;
const double HX_X_BAND = HX_X_MAX - HX_X_MIN;

const double HX_Y_MIN = -5;
const double HX_Y_MAX = 5;
const double HX_Y_MID = (HX_Y_MAX + HX_Y_MIN) / 2;
const double HX_Y_BAND = HX_Y_MAX - HX_Y_MIN;

const double HX_Z_MIN = -16.0;
const double HX_Z_MAX = 16.0;
const double HX_Z_MID = (HX_Z_MAX + HX_Z_MIN) / 2;
const double HX_Z_BAND = HX_Z_MAX - HX_Z_MIN;

const double HX_A_MIN = radians(-15.0);
const double HX_A_MAX = radians(15.0);
const double HX_A_MID = (HX_A_MAX + HX_A_MIN) / 2;
const double HX_A_BAND = HX_A_MAX - HX_A_MIN;

const double HX_B_MIN = radians(-15.0);
const double HX_B_MAX = radians(15.0);
const double HX_B_MID = (HX_B_MAX + HX_B_MIN) / 2;
const double HX_B_BAND = HX_B_MAX - HX_B_MIN;

const double HX_C_MIN = radians(-5.0);
const double HX_C_MAX = radians(5.0);
const double HX_C_MID = (HX_C_MAX + HX_C_MIN) / 2;
const double HX_C_BAND = HX_C_MAX - HX_C_MIN;

const double THETA_P = radians(50.02);  // Platform joint angle (radians) offset from AXIS[1|2|3]. A value of zero puts these joints directly on the axes.
const double THETA_B = radians(20.53);  // Base Servo pinion angle (radians) offset from AXIS[1|2|3]. A value of zero puts the servo pinion directly on the axes.
const double P_RAD = 115.45 / 2;        // Platform radius (mm). The distance from the center of the platform to the center of one platform / pushrod "joint". This should be the same for all six pushrods.
const double B_RAD = 165.40 / 2;        // Base radius (mm). Distance from the center of the base plate to the center of one servo pinion gear. Again, this should be the same for all six servos.
const double ARM_LENGTH = 24.0;         // Servo arm length (mm). Distance from the center of the servo pivot to the center of the pushrod pivot on the servo arm.
const double ROD_LENGTH = 106.0;        // Push rod length (mm). Distance between pushrod ball joints (servo to platform).
const double Z_HOME = -100.0;           // Default Z height of the platform (above the base), with servo arms horizontal. Formally, the distance from the plane described by the collection of servo pinion gear centers, to the plane described by the collection of platform / pushrod joints. Must Be fine tuned manualy or computed with a numerical solver.

/*
 * ======== ALGORITHM FOR SERVO ANGLE CALCULATIONS ========
 */
#define ALGO 3
