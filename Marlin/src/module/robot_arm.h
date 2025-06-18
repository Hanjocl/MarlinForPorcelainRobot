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

#include "../../lib/BasicLinearAlgebra-master/BasicLinearAlgebra.h"
using namespace BLA;

// Template to populate with array defined in configuration.h
struct JOINT {
    float theta;
    float d;
    float a;
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
    JOINT joints[SIZE];

    DHParameters(const float arr[SIZE][4]) {
        for (int i = 0; i < SIZE; ++i) {
            joints[i].theta = arr[i][0];
            joints[i].d = arr[i][1];
            joints[i].a = arr[i][2];
            joints[i].alpha = arr[i][3];
        }
    }
};


extern float segments_per_second;
extern xyz_float_t joint_axis_travel_offset;
extern xyz_pos_t end_affector_start_position;           // Should replace the manual_home_pos! Used to get the joint_travel_offset
extern float max_distance;                               // TO be implemented... now set manual in configuration.h
extern float min_distance;                               // TO be implemented... now set manual in configuration.h
extern float distance_c;                                // TO be implemented... now set manual in configuration.h
extern xyz_float_t plane_ref_point;                      // Calculate at startup!
extern xyz_float_t plane_normal;                         // Calculate at startup!
extern xyz_float_t origin_on_plane;                      // Calculate at startup! 

// Variables for posititon to angle convertion
extern float axis_z_angle_offset_low;
extern float axis_z_angle_offset_high;
extern float axis_z_d1;
extern float axis_z_default_length ;

extern float axis_y_angle_offset_low;
extern float axis_y_angle_offset_high;
extern float axis_y_d1;
extern float axis_y_d2;
extern float axis_y_default_length;

extern float axis_x_angle_offset_low;
extern float axis_x_angle_offset_high;
extern float axis_x_d1;
extern float axis_x_d2;
extern float axis_x_default_length;

constexpr float joint_arr[][4] = JOINTS;
const int N_joint = COUNT(joint_arr);
// Define joints in easy useable form.
const DHParameters<N_joint> dh_para_ref = joint_arr;
BLA::Matrix<4,4> dh_transform(JOINT& j);

void forward_kinematics(const_float_t pos_x, const_float_t pos_y, const_float_t pos_z);
void inverse_kinematics(const xyz_pos_t &raw);

void home_robot_arm(bool doX, bool doY, bool doZ);
void robot_arm_report_positions();

float axis_x_angle_to_position(const_float_t angle);
float axis_y_angle_to_position(const_float_t angle);
float axis_z_angle_to_position(const_float_t angle);

float axis_x_position_to_angle(const_float_t pos);
float axis_y_position_to_angle(const_float_t pos);
float axis_z_position_to_angle(const_float_t pos);

BLA::Matrix<4,4> dh_transform_up_to(const DHParameters<N_joint>& dh_params, int joint_idx);
xyz_float_t project_point_to_plane(const xyz_float_t &point, const xyz_float_t &plane_point, const xyz_float_t &plane_normal);