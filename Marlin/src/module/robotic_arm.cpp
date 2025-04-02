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

/**
 * delta.cpp
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(ARTICULATED_ROBOT_ARM)

#include "robotic_arm.h"
#include "motion.h"

// For homing:
#include "planner.h"
#include "endstops.h"
#include "../lcd/marlinui.h"
#include "../MarlinCore.h"

#if HAS_BED_PROBE // UNTESTED
#include "probe.h"
#endif

#if ENABLED(SENSORLESS_HOMING) // UNTESTED
#include "../feature/tmc_util.h"
#include "stepper/indirection.h"
#endif

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../core/debug_out.h"

// Initialized by settings.load
xy_float_t robot_arm[ABC];
xy_float_t joint_1[ABCE];
xy_float_t joint_2[ABCE];
xy_float_t joint_3[ABCE];
xy_float_t joint_4[ABCE];


/**
 * ROBOT ARM Inverse Kinematics
 */

#define ROBOTIC_ARM_DEBUG(VAR) do { \
    SERIAL_ECHOLNPGM_P(PSTR("Cartesian X"), VAR.x, SP_Y_STR, VAR.y, SP_Z_STR, VAR.z); \
    SERIAL_ECHOLNPGM_P(PSTR("Robotic Arm"), delta.a, SP_B_STR, delta.b, SP_C_STR, delta.c); \
}while(0)


// input two matrices, outputs multiplication of those two matrices 
void generate_matrices() {
    joint_1[C_AXIS] = 0;
}

void inverse_kinematics(const xyz_pos_t &raw) {

}

void forward_kinematics() {

}

/**
 * A Robot Arm can only safely home one axis at the time
 */
void home_robotic_arm() {

}

#endif // robotic_arm
