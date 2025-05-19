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

#include "../MarlinCore.h"
#include "motion.h"
#include "planner.h"
#include "endstops.h"

// Custom Imported libraries
//#include "../../lib/Eigen_Main/Eigen.h"
#include "../../lib/BasicLinearAlgebra-master/BasicLinearAlgebra.h"
using namespace BLA;

// Init by settings.load
float segments_per_second = DEFAULT_SEGMENTS_PER_SECOND;
xyz_float_t joint_axis_travel_offset;
xyz_pos_t end_affector_start_position;

// Custom Homing routine for Robot Arm motors (Homes each axis (only) one after another)
void home_robot_arm(bool doX, bool doY, bool doZ) {
  // Init the current position of all carriages to 0,0,0
  if (doX) {
    current_position.x = 0;
  }
   if (doY) {
    current_position.y = 0;
  }
   if (doZ) {
    current_position.z = 0;
  }
  
  destination.reset();
  if (!doX) {
    destination.x = current_position.x;
  }
  if (!doY) {
    destination.x = current_position.y;
  }
  if (!doZ) {
    destination.x = current_position.z;
  }
  
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
  if (doX) {
    homeaxis(X_AXIS);
    set_axis_is_at_home(X_AXIS);
  }
  if (doY) {
    homeaxis(Y_AXIS);
    set_axis_is_at_home(Y_AXIS);
  }
  if (doZ) {
    homeaxis(Z_AXIS);
    set_axis_is_at_home(Z_AXIS);
  }


  // Re-enable stealthChop if used. Disable diag1 pin on driver.
  #if ENABLED(SENSORLESS_HOMING)
    TERN_(X_SENSORLESS, end_sensorless_homing_per_axis(X_AXIS, stealth_states_x));
    TERN_(Y_SENSORLESS, end_sensorless_homing_per_axis(Y_AXIS, stealth_states_y));
    TERN_(Z_SENSORLESS, end_sensorless_homing_per_axis(Z_AXIS, stealth_states_z));
  #endif

  sync_plan_position();
}

// Convert joint inputs in degrees to XYZ outputs in mm
void forward_kinematics(const_float_t pos_m1, const_float_t pos_m2, const_float_t pos_m3) {
  // Convert postiion of motors to angles
  float angle_m1 = position_to_angle(pos_m1);
  float angle_m2 = position_to_angle(pos_m2);
  float angle_m3 = position_to_angle(pos_m3);
  // Get original parameters and create a temporary matrix
  DHParameters<N_joint> dh_para_cal = dh_para_ref;

  // Add angle values to joints
  dh_para_cal.joints[1].theta += angle_m1;
  dh_para_cal.joints[2].theta += angle_m2;
  dh_para_cal.joints[3].theta += angle_m3;

  BLA::Matrix<4,4> temp_matrix = {
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
  };

  for (JOINT& joint : dh_para_cal.joints) {
    temp_matrix *= dh_transform(joint);
  }

  const float pos_x = temp_matrix(0, 3);
  const float pos_y = temp_matrix(1, 3);
  const float pos_z = temp_matrix(2, 3);

  cartes.set(pos_x, pos_y, pos_z);
}

BLA::Matrix<4,4> dh_transform(JOINT& j) {
  BLA::Matrix<4,4> T = {
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
  };;

  float cos_theta = cos(j.theta);
  float sin_theta = sin(j.theta);
  float cos_aplha = cos(j.alpha);
  float sin_aplha = sin(j.alpha);

  T = { cos_theta,    -sin_theta * cos_aplha,   sin_theta * sin_aplha,    j.a * cos_theta,
        sin_theta,     cos_theta * cos_aplha,   -cos_theta * sin_aplha,   j.a * sin_theta,
        0.0,           sin_aplha,               cos_aplha,                j.d,
        0.0,           0.0,                     0.0,                      1.0
      };
  
  return T;
}

// Convery cartesian coordinates into movements for the arm.
void inverse_kinematics(const xyz_pos_t &target) {
  //SERIAL_ECHOLNPGM("  (IV_K) Target destination => x:", target.x,"Y:", target.y, " Z:", target.z);
  
  // Store calculated angle in here
  float joint_1 = 0;
  float joint_2 = 0;
  float joint_3 = 0;
  
  // STEP 1:  Find distance between org and given point in 3d space: √((x2-x1)^2+(y2-y1)^2+(z2-z1)^2)
  const float distance = SQRT(sq(target.x) + sq(target.y) + sq(target.z));
  //SERIAL_ECHOLNPGM("  (IV_K) Target Distance => D: ", distance);

  // STEP 2:  Get angle for Joint 3
  //          Calculates the angle for given distances
  float cos_angle = (sq(dh_para_ref.joints[2].a) + sq(dh_para_ref.joints[3].a) - sq(distance)) / (2 * dh_para_ref.joints[2].a * dh_para_ref.joints[3].a);
  LIMIT(cos_angle, -1, 1); // Make sure cos is not going out of bound
  joint_3 = RADIANS(180) - ACOS(cos_angle);

  //SERIAL_ECHOLNPGM("(IV_K) | Joint 2 a:", dh_para_ref.joints[2].a, "Joint 3 a:",dh_para_ref.joints[3].a);

  // STEP 3:  Do forward kinematics to get the end position of the arm with only joint 3 turned
  DHParameters<N_joint> dh_para_cal = dh_para_ref;
  dh_para_cal.joints[3].theta += joint_3;

  BLA::Matrix<4,4> temp_matrix = {
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
  };

  for (JOINT& joint : dh_para_cal.joints) {
    temp_matrix *= dh_transform(joint);
  }

  // STEP 4:  Get working end position of the arm == temporay vector of arm
  const xyz_pos_t temp_pos = { temp_matrix(0, 3), temp_matrix(1, 3), temp_matrix(2, 3) };
  //SERIAL_ECHOLNPGM("(IV_K) | Temp Target is X:", temp_pos[0], " Y:", temp_pos[1], " Z:", temp_pos[2]);

  // STEP 5:  Get vector to rotate X and Y axis based on temp vector and target position
  const float magnitude_temp = SQRT(sq(temp_pos.x) + sq(temp_pos.y) +sq(temp_pos.z));
  const float magnitude_target = SQRT(sq(target.x) + sq(target.y) +sq(target.z));
  
    // joint angle = Target angle - temporary pos anlge 
  joint_1 = acos(target.x / magnitude_target) - acos(temp_pos.x / magnitude_temp);
  joint_2 = acos(target.y / magnitude_target) - acos(temp_pos.y / magnitude_temp);
  
  //SERIAL_ECHOLNPGM("  (IV_K) Joint angles       => j1: ", DEGREES(joint_1)," | j2: ",  DEGREES(joint_2), " | j3: ",  DEGREES(joint_3));

  // STEP 6: Output angles to delta    
  delta.set(angle_to_position(joint_1, joint_axis_travel_offset.x) , angle_to_position(joint_2, joint_axis_travel_offset.y), angle_to_position(joint_3, joint_axis_travel_offset.z));
  SERIAL_ECHOLNPGM("  (IV_K) Delta Position     => x: ", delta.a," | y: ", delta.b, " | z: ", delta.c);
}

// Copied and adjusted from another kinematic system
void robot_arm_report_positions() {
  SERIAL_ECHOLNPGM(
    "Joint 1:", planner.get_axis_position_mm(X_AXIS)
  , "  Joint 2:", planner.get_axis_position_mm(Y_AXIS)
  , "  Joint 3:", planner.get_axis_position_mm(Z_AXIS)
  );
}


/*
*  Input an angle for a joint and converts it based on parameters to linear actuator position.
*  These are highly specific functions. That is only needed for my usecase probably
*  (Kinda of a post-processor for the inverse kinematics)
*/ 
float angle_to_position(const_float_t joint_angle, const_float_t zero_offset) { 
  // Add offsets and get angle that is important
  float angle = ABS(joint_angle) + RADIANS(JOINT_ANGLE_OFFSET);
  angle = RADIANS(180) - angle;

  // convert theta to distance 
  float distance = 2 * JOINT_RADIUS * sin(angle/2);

  float position = 0;
  // Depending on if angle is positive or negative return right result
  if (joint_angle >= 0) {
    position = distance - DISTANCE_OFFSET;
  } else {
    position = -distance + DISTANCE_OFFSET;
  }

  // Add zero_offset to compensate for difference in porcelain 
  position -= zero_offset;

  if (position >= MAX_AXIS_TRAVEL) {
    position = MAX_AXIS_TRAVEL;
    SERIAL_ECHOLNPGM("WARNING: Maximum axis travel reached!");
  } else if (position <= MIN_AXIS_TRAVEL) {
    position = MIN_AXIS_TRAVEL;
    SERIAL_ECHOLNPGM("WARNING: Minimum axis travel reached!");
  }
  
  // Limits travel range to axis max
  return position;
}

/*
*  Input position of linear actuator and converts it based on parameters to angle of joint.
*  These are highly specific functions. That is only needed for my usecase probably
*  (Kinda of a post-processor for the inverse kinematics function)
*/ 
float position_to_angle(const float position) {
  // Linear displacement from the neutral position
  float delta = position;

  // Chord length for this displacement
  float chord = DISTANCE_OFFSET + delta;

  // Ensure chord is within domain of asin
  if (chord > 2 * JOINT_RADIUS) chord = 2 * JOINT_RADIUS;
  if (chord < 0) chord = 0;

  // Compute central angle from chord length
  float angle_radians = 2 * asin(chord / (2 * JOINT_RADIUS));

  // Remove the zero-angle offset
  float zero_offset = 2 * asin(DISTANCE_OFFSET / (2 * JOINT_RADIUS));
  float angle = angle_radians - zero_offset;

  return angle;
}

#endif // ROBOT_ARM
