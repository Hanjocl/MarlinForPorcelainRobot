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
* delta.h - Delta-specific functions
*/

#include "../core/types.h"
#include "../core/macros.h"

extern xy_float_t robot_arm[ABC];


/**
* robotic_arm Inverse Kinematics
*
* Calculate the tower positions for a given machine
* position, storing the result in the robotic_arm[] array.
*/
void inverse_kinematics(const xyz_pos_t &raw);

/**
* robotic_arm Forward Kinematics
* 
* Has to convert raw angles for each joint to motor positions using the DH-parameters
* 
* The result is stored in the cartes[] array.
*/
void forward_kinematics(const_float_t z1, const_float_t z2, const_float_t z3);

FORCE_INLINE void forward_kinematics(const abc_float_t &point) {
  forward_kinematics(point.a, point.b, point.c);
}

void home_robotic_arm();
