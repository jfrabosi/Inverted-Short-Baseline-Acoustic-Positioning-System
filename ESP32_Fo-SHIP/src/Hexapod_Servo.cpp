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

#include <Hexapod_Servo.h>

#define COUNT_OF(x) ((sizeof(x) / sizeof(0 [x])) / ((size_t)(!(sizeof(x) % sizeof(0 [x])))))

extern angle_t servo_angles0[6];
extern angle_t servo_angles1[6];
extern angle_t servo_angles2[6];
extern angle_t servo_angles3[6];

/**
 *
 */
Hexapod_Servo::Hexapod_Servo()
{
}

/**
 * @brief Initializes the servo drivers and I2C connection
 */
void Hexapod_Servo::setupServo()
{
    Wire.begin();          // Wire must be started first
    Wire.setClock(100000); // Supported baud rates are 100kHz, 400kHz, and 1000kHz

    pwmControllerA.resetDevices();      // Software resets all PCA9685 devices on Wire line
    pwmControllerA.init(B000000);       // Address pins A5-A0 set to B000000 (for bottom PCA board)
    pwmControllerA.setPWMFrequency(50); // Default is 200Hz, supports 24Hz to 1526Hz

    pwmControllerB.init(B000001);       // Address pins A5-A0 set to B000001 (for top PCA board)
    pwmControllerB.setPWMFrequency(50); // Default is 200Hz, supports 24Hz to 1526Hz

    int8_t movOK = home(servo_angles0, servo_angles1, servo_angles2, servo_angles3);
    this->updateServos(movOK);
    delay(500);
}

/**
 * @brief Set servo values to the angles in servo_angles[].
 * 
 * @param movOK int that tracks if the move is valid
 * @param safetyWait_ms amount of milliseconds to wait before updating
 */
void Hexapod_Servo::updateServos(int8_t movOK, unsigned long safetyWait_ms)
{
    // Statistics of errors.
    static double nbMov = 0;
    static double nbMovNOK = 0;
    static double NOKrate = 0;
    nbMov++;

    if (movOK == 0)
    {
        // Do not update too quickly. This helps a little to prevent
        // the servos from going crazy. safetyWait_ms must be set to 0
        // when going on a smooth trajectory with a lot of points.
        static unsigned long T1;
        while ((millis() - T1) < safetyWait_ms)
        {
        }
        T1 = millis();

        // write the values in servo_angles0-3 to the PCA9685 PWM controllers
        static uint16_t pwms0[6];
        static uint16_t pwms1[6];
        static uint16_t pwms2[6];
        static uint16_t pwms3[6];
        for (uint8_t sid = 0; sid < 6; sid++)
        {
            pwms0[sid] = servo_angles0[sid].pwm_us;
            pwms1[sid] = servo_angles1[sid].pwm_us;
            pwms2[sid] = servo_angles2[sid].pwm_us;
            pwms3[sid] = servo_angles3[sid].pwm_us;
        }
        pwmControllerA.setChannelsPWM(0, NB_SERVOS + 0, pwms0);
        pwmControllerA.setChannelsPWM(6, NB_SERVOS + 6, pwms1);
        pwmControllerB.setChannelsPWM(0, NB_SERVOS + 0, pwms2);
        pwmControllerB.setChannelsPWM(6, NB_SERVOS + 6, pwms3);

    }
    else
    {
        // Error handling.
        nbMovNOK++;
        NOKrate = (nbMovNOK / nbMov) * (double)100.0;
        Serial.printf("%10lu", millis());
        Serial.printf(" | BAD MOVE | movOK = %d", movOK);
        Serial.printf(" | NB MOV = %10.0f", nbMov);
        Serial.printf(" | NOK rate = %4.1f %%", NOKrate);
        Serial.print("\n");
    }
}

/**
 * @brief Prints the angle of each servo in degrees for debugging.
 */
void Hexapod_Servo::printServoAngles()
{
    Serial.print("\nSERVO COORD        = ");
    Serial.print(getHX_X());
    Serial.print(" ");
    Serial.print(getHX_Y());
    Serial.print(" ");
    Serial.print(getHX_Z());
    Serial.print(" ");
    Serial.print(getHX_A());
    Serial.print(" ");
    Serial.print(getHX_B());
    Serial.print(" ");
    Serial.print(getHX_C());

    Serial.print("\nSERVO_ANGLES (rad) = ");
    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        Serial.print(servo_angles0[sid].deg);
        Serial.print(" ");
    }
    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        Serial.print(servo_angles1[sid].deg);
        Serial.print(" ");
    }
    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        Serial.print(servo_angles2[sid].deg);
        Serial.print(" ");
    }
    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        Serial.print(servo_angles3[sid].deg);
        Serial.print(" ");
    }

    Serial.print("\n");
}

/**
 * @brief Prints the coordinates of each servo's base and top joint connection (P and B)
 */
void Hexapod_Servo::printJointAndServoAxisCoord()
{
    Serial.println("P_COORDS");
    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        Serial.print(P_COORDS[sid][0]);
        Serial.print("  ");
        Serial.println(P_COORDS[sid][1]);
    }

    Serial.println("\nB_COORDS");
    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        Serial.print(B_COORDS[sid][0]);
        Serial.print("  ");
        Serial.println(B_COORDS[sid][1]);
    }
}
