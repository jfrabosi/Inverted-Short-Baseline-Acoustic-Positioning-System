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

#include <Hexapod_Demo.h>
#include <Hexapod_Servo.h>
#include <Hexapod_Kinematics.h>

#define COUNT_OF(x) ((sizeof(x) / sizeof(0 [x])) / ((size_t)(!(sizeof(x) % sizeof(0 [x])))))

extern angle_t servo_angles0[6];
extern angle_t servo_angles1[6];
extern angle_t servo_angles2[6];
extern angle_t servo_angles3[6];
extern Hexapod_Servo hx_servo;
extern String readSerialString();
extern void moveSlowly(platform_t *startCoords, platform_t *endCoords, uint16_t num_steps, uint8_t which_platforms=15);
extern int binaryToDecimal(int n);
extern HardwareSerial SerialPort;
extern void sendRobustFloat(float value);

/**
 *
 */
Hexapod_Demo::Hexapod_Demo()
{
}

/**
 * @brief Moves the Fo-SHIP along each of its axes from min to max
 * 
 * This function is a demonstration for showing the full range-of-motion of the Fo-SHIP.
 * It cycles through all its axes in order (XYZABC), moving each platform the same way.
 * Then, it makes the second and third platform move opposite to the first and fourth;
 * this makes the end effector stay stationary (or level).
 * 
 */
void Hexapod_Demo::demoMov_MinMaxAllAxis()
{
    Serial.println("\n########## demoMov_MinMaxAllAxis START ##########");

    const platform_t coords[] = {
        // X
        {HX_X_MAX, 0, 0, 0, 0, 0},
        {HX_X_MIN, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},

        // Y
        {0, HX_Y_MAX, 0, 0, 0, 0},
        {0, HX_Y_MIN, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},

        // Z
        {0, 0, HX_Z_MAX, 0, 0, 0},
        {0, 0, HX_Z_MIN, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},

        // A
        {0, 0, 0, HX_A_MAX, 0, 0},
        {0, 0, 0, HX_A_MIN, 0, 0},
        {0, 0, 0, 0, 0, 0},

        // B
        {0, 0, 0, 0, HX_B_MAX, 0},
        {0, 0, 0, 0, HX_B_MIN, 0},
        {0, 0, 0, 0, 0, 0},

        // C
        {0, 0, 0, 0, 0, HX_C_MAX},
        {0, 0, 0, 0, 0, HX_C_MIN},
        {0, 0, 0, 0, 0, 0}};

    const platform_t coords_inv[] = {
        // X
        {HX_X_MIN, 0, 0, 0, 0, 0},
        {HX_X_MAX, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},

        // Y
        {0, HX_Y_MIN, 0, 0, 0, 0},
        {0, HX_Y_MAX, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},

        // Z
        {0, 0, HX_Z_MIN, 0, 0, 0},
        {0, 0, HX_Z_MAX, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},

        // A
        {0, 0, 0, HX_A_MIN, 0, 0},
        {0, 0, 0, HX_A_MAX, 0, 0},
        {0, 0, 0, 0, 0, 0},

        // B
        {0, 0, 0, 0, HX_B_MIN, 0},
        {0, 0, 0, 0, HX_B_MAX, 0},
        {0, 0, 0, 0, 0, 0},

        // C
        {0, 0, 0, 0, 0, HX_C_MIN},
        {0, 0, 0, 0, 0, HX_C_MAX},
        {0, 0, 0, 0, 0, 0}};

    double coords_scaled[] = {0, 0, 0, 0, 0, 0};
    double coords_inv_scaled[] = {0, 0, 0, 0, 0, 0};

    platform_t startPos = {0, 0, 0, 0, 0, 0};
    platform_t endPos = {0, 0, 0, 0, 0, 0};
    platform_t startPosInv = {0, 0, 0, 0, 0, 0};
    platform_t endPosInv = {0, 0, 0, 0, 0, 0};
    
    int8_t movOK = -1;
    for (uint8_t cnt = 0; cnt < (COUNT_OF(coords) * 2); cnt++)
    {
        // if it's cycled through all coords once, then do the inverted movement (2 and 3 are inverse)
        endPos = coords[cnt % COUNT_OF(coords)];
        if (cnt < COUNT_OF(coords)){
            endPosInv = coords[cnt % COUNT_OF(coords)];
        }
        else{
            endPosInv = coords_inv[cnt % COUNT_OF(coords)];
        }

        // modify the max step size for different movements - need more steps when switching between axes
        uint16_t max_step = 10;
        if ((cnt % COUNT_OF(coords) == 6) or (cnt % COUNT_OF(coords) == 8) or (cnt % COUNT_OF(coords) == 9) or (cnt % COUNT_OF(coords) == 11) or (cnt % COUNT_OF(coords) == 12) or (cnt % COUNT_OF(coords) == 14)) {
            max_step = 30;
        }
        
        if ((cnt % COUNT_OF(coords) == 7) or (cnt % COUNT_OF(coords) == 10) or (cnt % COUNT_OF(coords) == 13)) {
            max_step = 60;
        }

        Serial.print("cnt = ");
        Serial.println(cnt);

        for (uint16_t step = 0; step < max_step; step++) {
            double t = ((double)step) / (double)max_step;

            // linearly interpolate between startPos and endPos
            platform_t interpPos;
            interpPos.hx_x = startPos.hx_x + t * (endPos.hx_x - startPos.hx_x);
            interpPos.hx_y = startPos.hx_y + t * (endPos.hx_y - startPos.hx_y);
            interpPos.hx_z = startPos.hx_z + t * (endPos.hx_z - startPos.hx_z);
            interpPos.hx_a = startPos.hx_a + t * (endPos.hx_a - startPos.hx_a);
            interpPos.hx_b = startPos.hx_b + t * (endPos.hx_b - startPos.hx_b);
            interpPos.hx_c = startPos.hx_c + t * (endPos.hx_c - startPos.hx_c);

            // linearly interpolate between startPosInv and endPosInv
            platform_t interpPosInv;
            interpPosInv.hx_x = startPosInv.hx_x + t * (endPosInv.hx_x - startPosInv.hx_x);
            interpPosInv.hx_y = startPosInv.hx_y + t * (endPosInv.hx_y - startPosInv.hx_y);
            interpPosInv.hx_z = startPosInv.hx_z + t * (endPosInv.hx_z - startPosInv.hx_z);
            interpPosInv.hx_a = startPosInv.hx_a + t * (endPosInv.hx_a - startPosInv.hx_a);
            interpPosInv.hx_b = startPosInv.hx_b + t * (endPosInv.hx_b - startPosInv.hx_b);
            interpPosInv.hx_c = startPosInv.hx_c + t * (endPosInv.hx_c - startPosInv.hx_c);

            // calculate servo angles using the interpolated positions
            movOK = hx_servo.calcServoAngles(interpPos, servo_angles0, 0);
            movOK += hx_servo.calcServoAngles(interpPosInv, servo_angles1, 1);
            movOK += hx_servo.calcServoAngles(interpPosInv, servo_angles2, 2);
            movOK += hx_servo.calcServoAngles(interpPos, servo_angles3, 3);

            hx_servo.updateServos(movOK);
            Serial.println(t);
            delay(50);
        }

        movOK = hx_servo.calcServoAngles(endPos, servo_angles0, 0);
        movOK += hx_servo.calcServoAngles(endPosInv, servo_angles1, 1);
        movOK += hx_servo.calcServoAngles(endPosInv, servo_angles2, 2);
        movOK += hx_servo.calcServoAngles(endPos, servo_angles3, 3);

        hx_servo.updateServos(movOK);

        startPos = coords[cnt % COUNT_OF(coords)];
        if (cnt < COUNT_OF(coords)){
            startPosInv = coords[cnt % COUNT_OF(coords)];
        }
        else{
            startPosInv = coords_inv[cnt % COUNT_OF(coords)];
        }
        delay(1000);
    }
    Serial.println("demoMov_MinMaxAllAxis DONE");
}

/**
 * @brief Moves the Fo-SHIP in a circle about the pitch and roll axes a set number of times
 * 
 * This function is a demonstration showing how the Fo-SHIP moves in a circle about its largest
 * axes of motion. It chases setpoints determined by (0, 0, 0, r*cos(theta), r*sin(theta), 0)
 * where r is the input radius_deg in degrees.
 * 
 * Care should be taken to start the Fo-SHIP in the coordinates (0, 0, 0, 11.25, 0, 0). This
 * must be done before this function is called.
 * 
 * @param nb_turn number of circles to do
 * @param radius_deg the radius of motion, must be smaller than HX_A_MAX
 */
void Hexapod_Demo::demoMov_circles(uint8_t nb_turn = 1, float radius_deg = 11.25)
{
    if (nb_turn == 0)
        return;

    Serial.println("\n########## demoMov_circles START ##########");

    // generate the coordinates to move through
    const uint8_t nb_points = 90;
    const double radius = (radians(radius_deg));
    const double angleInc = TWO_PI / nb_points;
    double angle = 0;
    platform_t coords[nb_points];
    for (uint8_t angleID = 0; angleID < nb_points; angleID++)
    {
        coords[angleID] = {0, 0, 0, (radius * cos(angle)), (radius * sin(angle)), 0};
        angle += angleInc;
    }

    // move through the coordinates
    int8_t movOK = -1;
    static unsigned long T1;
    for (uint8_t turn = 0; turn < nb_turn; turn++)
    {
        for (uint8_t cnt = 0; cnt < nb_points; cnt++)
        {
            while ((millis() - T1) < 50UL)
            {
            }
            T1 = millis();
            movOK = hx_servo.calcServoAngles(coords[cnt], servo_angles0, 0);
            movOK = hx_servo.calcServoAngles(coords[cnt], servo_angles1, 1);
            movOK = hx_servo.calcServoAngles(coords[cnt], servo_angles2, 2);
            movOK = hx_servo.calcServoAngles(coords[cnt], servo_angles3, 3);
            hx_servo.updateServos(movOK, 0UL);
        }
    }
    movOK = hx_servo.calcServoAngles(coords[0], servo_angles0, 0);
    movOK = hx_servo.calcServoAngles(coords[0], servo_angles1, 1);
    movOK = hx_servo.calcServoAngles(coords[0], servo_angles2, 2);
    movOK = hx_servo.calcServoAngles(coords[0], servo_angles3, 3);
    hx_servo.updateServos(movOK, 0UL);
    Serial.println("demoMov_circles DONE");
}


/**
 * @brief Points the iSBL array at the transmitter, active tracking
 * 
 * This function is a demonstration for actively tracking the transmitter.
 * It should be called in the setup() function in main.cpp before the main
 * loop() takes over. It's a blocking function, so it should only be enabled
 * when demonstration is desired.
 * 
 */
void Hexapod_Demo::lookAtTX()
{
    platform_t current_pos, new_pos;
    Serial.println("txx!");
    while (true){

        // read the last datapoint from the iSBL
        while (!SerialPort.available());
        Serial.println("new!");
        String receivedString = readSerialString();
        float x_kf, y_kf, z_kf, x_r_isbl, y_r_isbl, z_r_isbl, x_isbl, y_isbl, z_isbl, stm_roll, stm_pitch, stm_yaw;
        sscanf(receivedString.c_str(), "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", 
                &x_kf, &y_kf, &z_kf, &x_r_isbl, &y_r_isbl, &z_r_isbl, 
                &x_isbl, &y_isbl, &z_isbl, &stm_roll, &stm_pitch, &stm_yaw);

        Serial.println(stm_pitch);
        // calculate the angle to move the Fo-SHIP to point the iSBL array at the transmitter
        float pitch, yaw;
        pitch = atan2f(z_isbl, x_isbl);
        yaw = atan2f(y_isbl, x_isbl);
        Serial.println(degrees(pitch));
        Serial.println(degrees(yaw));
        new_pos.hx_b += pitch/4;
        new_pos.hx_c += yaw/4;
        moveSlowly(&current_pos, &new_pos, 14, binaryToDecimal(1111));
        current_pos = new_pos;
        
        // send a float (STM32 expects one)
        int good_send = 0;
        while (!good_send){
            sendRobustFloat(0.110f);
            while (!SerialPort.available());
            String receivedString = readSerialString();
            sscanf(receivedString.c_str(), "%i", &good_send);
            if (good_send == -1){
                Serial.println("DISCARD");
                break;
            }
        }
    }
}