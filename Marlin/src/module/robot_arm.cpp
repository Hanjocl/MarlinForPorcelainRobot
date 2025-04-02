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

#include "robot_arm.h"

#if ENABLED(ARTICULATED_ROBOT_ARM)

//#include "../inc/MarlinConfig.h"
//#include "../MarlinCore.h"
//#include "motion.h"
#include "planner.h"

#include "endstops.h"

float segments_per_second = DEFAULT_SEGMENTS_PER_SECOND;


/// Redo motion system: From length to joint angles
#if ENABLED(ARTICULATED_ROBOT_ARM)

  void robot_arm_set_axis_is_at_home(const AxisEnum axis) {
    if (axis == Z_AXIS)
      current_position.z = Z_HOME_POS;
    else {
      xyz_pos_t homeposition = { X_HOME_POS, Y_HOME_POS, Z_HOME_POS };
      //DEBUG_ECHOLNPGM_P(PSTR("homeposition X"), homeposition.x, SP_Y_LBL, homeposition.y, SP_Z_LBL, homeposition.z);

      inverse_kinematics(homeposition);
      forward_kinematics(delta.a, delta.b, delta.c);
      current_position[axis] = cartes[axis];

      //DEBUG_ECHOLNPGM_P(PSTR("Cartesian X"), current_position.x, SP_Y_LBL, current_position.y);
      update_software_endstops(axis);
    }
  }

  // Convert ABC inputs in degrees to XYZ outputs in mm
  void forward_kinematics(const_float_t a, const_float_t b, const_float_t c) {

  }

  // Home YZ together, then X (or all at once). Based on quick_home_xy & home_delta
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

    // Move all axis individually
    do_blocking_move_to_x(max_length(X_AXIS), homing_feedrate(X_AXIS));
    endstops.validate_homing_move();
    set_axis_is_at_home(X_AXIS);

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

  void inverse_kinematics(const xyz_pos_t &raw) {

    //SERIAL_ECHOLNPGM(" SCARA (x,y,z) ", spos.x , ",", spos.y, ",", spos.z, " Rho=", RHO, " Rho2=", RHO2, " Theta=", THETA, " Phi=", PHI, " Psi=", PSI, " Gamma=", GAMMA);
  }

#endif

void robot_arm_report_positions() {
  SERIAL_ECHOLNPGM(
    "Joint 1:", planner.get_axis_position_degrees(A_AXIS)
  , "  Joint 2:", planner.get_axis_position_degrees(B_AXIS)
  , "  Joint 3:", planner.get_axis_position_degrees(C_AXIS)
  );
}

#endif // ARTICULATED_ROBOT_ARM
