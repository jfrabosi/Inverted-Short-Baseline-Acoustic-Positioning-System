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

// #include "Hexapod_Kinematics.h"

/**
 * Calculation of the servo angles in radians, degrees and in microseconds (PWM)
 * given the desired target platform coordinates.
 *
 * @param coord : the desired target platform coordinates.
 * A struct containing X (surge), Y (sway), Z (heave) in mm
 * and A (roll), B (pitch) and C (yaw) in radians.
 *
 * @param servo_angles : pointer to an array of struct containing
 * the calculated servos angles in radians, degrees and in microseconds (PWM).
 *
 * @return
 * Returns = 0 if OK
 * Returns < 0 if Error
 *
 */
int8_t Hexapod_Kinematics::calcServoAngles(platform_t coord, angle_t servo_angles[], int platform_num)
{
    // Number of time the function was called.
    static uint64_t nb_call = 0;
    ++nb_call;
    
    // NOTE FROM JAKOB: the coordinate frame must be transformed from the original implementation
    // to my custom implementation (X forward, Y left, Z down)
    double temp = coord.hx_x;
    coord.hx_x = -coord.hx_y;
    coord.hx_y = -temp;
    temp = coord.hx_a;
    coord.hx_a = -coord.hx_b;
    coord.hx_b = -temp;
    coord.hx_c = -coord.hx_c;
    coord.hx_z = -coord.hx_z;

    angle_t new_servo_angles[NB_SERVOS];

    // Intermediate values, to avoid recalculating sin and cos.
    // (3 µs).
    double
        cosA = cos(coord.hx_a),
        cosB = cos(coord.hx_b),
        cosC = cos(coord.hx_c),
        sinA = sin(coord.hx_a),
        sinB = sin(coord.hx_b),
        sinC = sin(coord.hx_c);

    // Assume everything will be OK.
    int8_t movOK = 0;

    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        // Compute the new platform joint coordinates relative to servo pivot.
        // NOTE FROM JAKOB: this is the XYZ vector pointing from the servo pivot
        // B to the top ball joint connection P.
        // (~7 µs)
        double BP_x = P_COORDS[sid][0] * cosB * cosC +
                      P_COORDS[sid][1] * (sinA * sinB * cosC - cosA * sinC) +
                      coord.hx_x -
                      B_COORDS[sid][0];
        double BP_y = P_COORDS[sid][0] * cosB * sinC +
                      P_COORDS[sid][1] * (sinA * sinB * sinC + cosA * cosC) +
                      coord.hx_y -
                      B_COORDS[sid][1];
        double BP_z = -P_COORDS[sid][0] * sinB +
                      P_COORDS[sid][1] * sinA * cosB +
                      coord.hx_z -
                      Z_HOME;

        // NOTE FROM JAKOB: this step rotates the BP vector into the coordinate frame
        // of each servo. essentially, each servo is rotated either 60, 120, or 180
        // degrees relative to the coordinate system of the base. 
        double
            a = COS_THETA_S[sid] * BP_x + SIN_THETA_S[sid] * BP_y,
            b = -SIN_THETA_S[sid] * BP_x + COS_THETA_S[sid] * BP_y,
            c = BP_z,
            a2 = POW(a, 2),
            a4 = POW(a, 4),
            b2 = POW(b, 2),
            b4 = POW(b, 4),
            c2 = POW(c, 2),
            c4 = POW(c, 4);

        // Distance^2 between servo pivot (B) and platform joint (P).
        // Note that BP_x^2 + BP_y^2 + BP_z^2 = a^2 + b^2 + c^2, so
        // we can compare BP2 to BP2_MAX in the next test.
        double BP2 = a2 + b2 + c2;

        // Test if the new distance between servo pivot and platform joint
        // is longer than physically possible.
        // Abort computation of remaining angles if the current angle is not OK.
        // (~1 µs)
        if (BP2 > BP2_MAX)
        {
            movOK = -1;
            break;
        }

        // NOTE FROM JAKOB: Time to explain the inverse kinematics solver! Essentially, we
        // have three equations: one describing the length of the rod, one describing the
        // length of the servo arm, and one describing the angle of the servo arm.
        // We combine these three equations, along with some tricky trig identities, to
        // get an equation inthe form of a quadratic. The term i1 represents the right-hand
        // side of the numerator: t = -B - sqrt(B^2 - 4*A*C) / 2*A. i1 is everything inside
        // of the square root. If it's negative, then there's no solution to the quadratic.
        double i1 = -ARM_LENGTH4 - ROD_LENGTH4 - a4 - b4 - c4 +
                    2 * (ARM_LENGTH2 * (ROD_LENGTH2 + a2 - b2 + c2) +
                         ROD_LENGTH2 * a2 + ROD_LENGTH2 * (b2 + c2) -
                         a2 * (b2 + c2) - b2 * c2);
        if (i1 < 0)
        {
            movOK = -5;
            break;
        }

        // NOTE FROM JAKOB: Once we know that i1 is valid, we just substitute into the rest
        // of the quadratic. The way we derive our equations, t = tan(theta/2), where theta
        // is the angle of the servo horn. I recommend reading this issue on the original
        // github for an explanation (or read my thesis!)
        // https://github.com/NicHub/stewart-platform-esp32/issues/5
        double rt_i1 = sqrt(i1);
        double i2 = (2 * ARM_LENGTH * c - rt_i1) /
                    (ARM_LENGTH2 + 2 * ARM_LENGTH * a -
                    ROD_LENGTH2 + BP2);
        double i3 = 2 * atan(i2); // ~10 µs

        // NOTE FROM JAKOB: now, i1 represents the angle that the servo should set to
        // to hit the desired platform_t setpoint.
        new_servo_angles[sid].rad = i3;

        // Rotate the angle.
        // (~1 µs)
        new_servo_angles[sid].rad += SERVO_HALF_ANGULAR_RANGE;

        // Convert radians to degrees.
        // (~2 µs)
        new_servo_angles[sid].deg = degrees(new_servo_angles[sid].rad);

        // Convert radians to PWM.
        // The calibration values take into account the fact
        // that the odd and even arms are a reflection of each other.
        // (~5 µs)
        new_servo_angles[sid].pwm_us =
            round(SERVO_CALIBRATION[sid + platform_num * 6].gain * new_servo_angles[sid].rad) +
            SERVO_CALIBRATION[sid + platform_num * 6].offset;

        // Check if the angle is in min/max.
        // Abort computation of remaining angles if the current angle is not OK.
        // (~1 µs)
        if (new_servo_angles[sid].pwm_us > SERVO_MAX_PWM)
        {
            movOK = -3;
            break;
        }
        else if (new_servo_angles[sid].pwm_us < SERVO_MIN_PWM)
        {
            movOK = -4;
            break;
        }
    }

    // Update platform coordinates if there are no errors.
    // (~1 µs)
    if (movOK == 0)
    {
        for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
        {
            servo_angles[sid] = new_servo_angles[sid];
        }
        _coord.hx_x = coord.hx_x;
        _coord.hx_y = coord.hx_y;
        _coord.hx_z = coord.hx_z;
        _coord.hx_a = coord.hx_a;
        _coord.hx_b = coord.hx_b;
        _coord.hx_c = coord.hx_c;
    }

    return movOK;
}
