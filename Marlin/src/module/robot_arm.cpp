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
 * robot_arm.cpp
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(ROBOT_ARM)

#include "robot_arm.h"

//#include "../inc/MarlinConfig.h"
//#include "../MarlinCore.h"
#include "motion.h"
#include "planner.h"
#include "endstops.h"

float segments_per_second = DEFAULT_SEGMENTS_PER_SECOND;

// Convert joint inputs in degrees to XYZ outputs in mm
void forward_kinematics(const_float_t j1, const_float_t j2, const_float_t j3) {

}

// Home each axis individually and move it back to centre
void home_robot_arm() {
  // Init the current position of all carriages to 0,0,0
  current_position.reset();
  destination.reset();
  sync_plan_position();

  // Disable stealthChop if used. Enable diag1 pin on driver.
  #if ENABLED(SENSORLESS_HOMING)
    TERN_(X_SENSORLESS, sensorless_t stealth_states_x = start_sensorless_homing_per_axis(X_AXIS));
    TERN_(Y_SENSORLESS, sensorless_t stealth_states_y = start_sensorless_homing_per_axis(Y_AXIS));
    TERN_(Z_SENSORLESS, sensorless_t stealth_states_z = start_sensorless_homing_per_axis(Z_AXIS));
  #endif

  //const int x_axis_home_dir = TOOL_X_HOME_DIR(active_extruder);

  //const xy_pos_t pos { max_length(X_AXIS) , max_length(Y_AXIS) };
  //const float mlz = max_length(X_AXIS),

  // Move all carriages together linearly until an endstop is hit.
  //do_blocking_move_to_xy_z(pos, mlz, homing_feedrate(Z_AXIS));

  // Set the homing current for all motors
  TERN_(HAS_HOMING_CURRENT, set_homing_current(Z_AXIS));

  // Move each axis individually
  do_blocking_move_to_x(max_length(X_AXIS), homing_feedrate(X_AXIS));
  endstops.validate_homing_move();
  set_axis_is_at_home(X_AXIS);
  do_blocking_move_to_x(0, homing_feedrate(X_AXIS));

  do_blocking_move_to_y(max_length(Y_AXIS), homing_feedrate(Y_AXIS));
  endstops.validate_homing_move();
  set_axis_is_at_home(Y_AXIS);

  do_blocking_move_to_z(max_length(Z_AXIS), homing_feedrate(Z_AXIS));
  endstops.validate_homing_move();
  set_axis_is_at_home(Z_AXIS);


  // Re-enable stealthChop if used. Disable diag1 pin on driver.
  #if ENABLED(SENSORLESS_HOMING)
    TERN_(X_SENSORLESS, end_sensorless_homing_per_axis(X_AXIS, stealth_states_x));
    TERN_(Y_SENSORLESS, end_sensorless_homing_per_axis(Y_AXIS, stealth_states_y));
    TERN_(Z_SENSORLESS, end_sensorless_homing_per_axis(Z_AXIS, stealth_states_z));
  #endif

  sync_plan_position();
}


/* GOAL: convert raw cartesion XYZ coordinates into 'delta' aka rotation angles for each joint.
*
*/
void inverse_kinematics(const xyz_pos_t &raw) {

    
  delta.set(raw.x, raw.y, raw.z);
  //SERIAL_ECHOLNPGM(" SCARA (x,y,z) ", spos.x , ",", spos.y, ",", spos.z, " Rho=", RHO, " Rho2=", RHO2, " Theta=", THETA, " Phi=", PHI, " Psi=", PSI, " Gamma=", GAMMA);
}



void robot_arm_report_positions() {
  SERIAL_ECHOLNPGM(
    "Joint 1:", planner.get_axis_position_mm(A_AXIS)
  , "  Joint 2:", planner.get_axis_position_mm(B_AXIS)
  , "  Joint 3:", planner.get_axis_position_mm(C_AXIS)
  );
}

#endif // ROBOT_ARM
