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

extern float segments_per_second;

void forward_kinematics(const_float_t a, const_float_t b, const_float_t c);
void inverse_kinematics(const xyz_pos_t &raw);

void home_robot_arm();

void robot_arm_report_positions();


/*
*  Input an angle for a joint and converts it based on parameters to linear actuator position.
*  These are highly specific functions. That is only needed for my usecase probably
*  (Kinda of a post-processor for the inverse kinematics)
*/ 
float angle_to_position(const_float_t joint_angle, const_float_t radius) {    
    // Add offsets and get angle that is important
    float angle = ABS(joint_angle) + JOINT_OFFSET;
    angle = 180 - angle;

    // convert theta to distance 
    float position = 2 * radius * sin(RADIANS(angle/2));

    // Depending on if angle is positive or negative return right result
    if (joint_angle > 0) {
        return position - DISTANCE_OFFSET;
    } else {
        return -position + DISTANCE_OFFSET;
    }
  }

/*
*  Input position of linear actuator and converts it based on parameters to angle of joint.
*  These are highly specific functions. That is only needed for my usecase probably
*  (Kinda of a post-processor for the inverse kinematics function)
*/ 
float position_to_angle(const_float_t position, const_float_t radius) {    
    // Adjust position based on offset at zero
    float adj_distance = DISTANCE_OFFSET - ABS(position);

    // Solve for angle (distance = 2 * radius * sin(angle / 2))
    float angle_radians = asin(adj_distance / (2 * radius));
    float angle = DEGREES(angle_radians) *2;
    
    // Adjust the angle by the JOINT_OFFSET and keep what is left over as current angle
    angle = 180.0f - JOINT_OFFSET - angle ;
    
    return angle;
}


// Template to populate with array defined in configuration.h
struct joint {
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
template <int N>
struct DHParameters {  
    joint joint[N];

    DHParameters(const float arr[N][4]) {
        for (int i = 0; i < 3; ++i) {
            joints[i].alpha = arr[i][0];
            joints[i].r = arr[i][1];
            joints[i].d = arr[i][2];
            joints[i].theta = arr[i][3];
        }
    }
};

// Populate array based on JOINTS define in configuration.h
constexpr float joint_arr[][4] = JOINTS;
const int N_joint = COUNT(joint_arr);

// Define joints in easy useable form.
DHParameters<N_joint> dh_para = joint_arr;