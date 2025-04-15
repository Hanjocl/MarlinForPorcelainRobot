/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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

/**
 * robot_arm.h - robot_arm-specific functions
 */

#include "../core/types.h"

// Template to populate with array defined in configuration.h
struct JOINT {
    float theta;
    float r;
    float d;
    float alpha;
};

// 
/** Creates easy to use values picker for DH Parameter calculations/ 
 *  e.g. -> joint[0].theta or joint[2].r
 *  
 * (I think this should be in types.h but i don't know how to integrate shit)
 */
template <int SIZE>
struct DHParameters {  
    JOINT joint[SIZE];

    DHParameters(const float arr[SIZE][4]) {
        for (int i = 0; i < 3; ++i) {
            joint[i].theta = arr[i][0];
            joint[i].r = arr[i][1];
            joint[i].d = arr[i][2];
            joint[i].alpha = arr[i][3];
        }
    }
};


extern float segments_per_second;

void forward_kinematics(const_float_t a, const_float_t b, const_float_t c);
void inverse_kinematics(const xyz_pos_t &raw);

void home_robot_arm(bool doX, bool doY, bool doZ);

void robot_arm_report_positions();

float angle_to_position(const_float_t joint_angle, const_float_t radius);
float position_to_angle(const_float_t position, const_float_t radius);