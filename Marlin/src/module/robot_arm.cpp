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
#include "../../lib/BasicLinearAlgebra-master/BasicLinearAlgebra.h"
using namespace BLA;

// Init by settings.load
float segments_per_second = DEFAULT_SEGMENTS_PER_SECOND;
xyz_float_t joint_axis_travel_offset;
xyz_pos_t end_affector_start_position;

float distance_c;
float max_distance;
float min_distance;
xyz_float_t plane_ref_point;
xyz_float_t plane_normal;
xyz_float_t origin_on_plane;   

float axis_z_angle_offset_low;
float axis_z_angle_offset_high;
float axis_z_d1;
float axis_z_default_length ;

float axis_y_angle_offset_low;
float axis_y_angle_offset_high;
float axis_y_d1;
float axis_y_d2;
float axis_y_default_length;

float axis_x_angle_offset_low;
float axis_x_angle_offset_high;
float axis_x_d1;
float axis_x_d2;
float axis_x_default_length;

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
  float angle_m1 = axis_x_position_to_angle(pos_m1);
  float angle_m2 = axis_y_position_to_angle(pos_m2);
  float angle_m3 = axis_z_position_to_angle(pos_m3);

  SERIAL_ECHOLNPGM("(FW_K) Joint 1 => Angle: ", DEGREES(angle_m1)," | pos: ", pos_m1);
  SERIAL_ECHOLNPGM("(FW_K) Joint 1 => Angle: ", DEGREES(angle_m2)," | pos: ", pos_m2);
  SERIAL_ECHOLNPGM("(FW_K) Joint 1 => Angle: ", DEGREES(angle_m3)," | pos: ", pos_m3);
  // Get original parameters and create a temporary matrix
  DHParameters<N_joint> dh_para_cal = dh_para_ref;

  // Add angle values to joints
  dh_para_cal.joints[1].theta -= angle_m1;
  dh_para_cal.joints[2].theta -= angle_m2;
  dh_para_cal.joints[3].theta -= angle_m3;

  SERIAL_ECHOLNPGM("(FW_K) joint Angle 1:", DEGREES(dh_para_cal.joints[1].theta), " | joint Angle 2:", DEGREES(dh_para_cal.joints[2].theta), " | joint Angle 3:", DEGREES(dh_para_cal.joints[3].theta));

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

// Convery cartesian coordinates into linear actuator position for each joint.
void inverse_kinematics(const xyz_pos_t &target) {
  //SERIAL_ECHOLNPGM("  (IV_K) Target destination => x:", target.x,"Y:", target.y, " Z:", target.z);
  DHParameters<N_joint> dh_para_cal = dh_para_ref;
  // Store calculated angle in here
  float joint_1 = 0;
  float joint_2 = 0;
  float joint_3 = 0;
  
  // Find distances
  const float distance_to_target = SQRT(sq(target.x) + sq(target.y) + sq(target.z));
  //const xyz_float_t plane_ref_point = {-48.71,   126.48089413, -804.93247537};           // Calculate at startup!
  //const xyz_float_t plane_normal = {0.39113274, -0.90917865, -0.14286134};           // Calculate at startup!
  //const xyz_float_t origin_on_plane = {-7.45189049, 17.32174043,  2.72180498};           // Calculate at startup!

  
  const float distance_projected_orgin_to_target =  SQRT(sq(target.x - origin_on_plane.x) + sq(target.y - origin_on_plane.y) + sq(target.z - origin_on_plane.z));
  
  
  const float side_b = SQRT(sq(plane_ref_point.x - origin_on_plane.x) + sq(plane_ref_point.y - origin_on_plane.y) + sq(plane_ref_point.z - origin_on_plane.z));
  const float side_c = SQRT(sq(distance_projected_orgin_to_target) - sq(dh_para_ref.joints[3].d - distance_c));
  
  const float beta = acos(dh_para_ref.joints[2].a / side_b);
  const float alpha = acos((sq(side_b) + sq(dh_para_ref.joints[3].a) - sq(side_c)) / (2 * side_b * dh_para_ref.joints[3].a));
  const float angle_z = PI + beta - alpha;

  //SERIAL_ECHOLNPGM("  (IV_K) Distance             => ", distance_to_target);
  //SERIAL_ECHOLNPGM("  (IV_K) Distance (virtual)   => ", distance_projected_orgin_to_target);
  //SERIAL_ECHOLNPGM("  (IV_K) Origin point   => ", origin_on_plane.x, ", ", origin_on_plane.y, ", ", origin_on_plane.z, );
  
  // STEP 3:  Do forward kinematics to get the end position of the arm with only joint 3 turned
  joint_3 = -1 * (angle_z + dh_para_cal.joints[3].theta);
  
  dh_para_cal.joints[3].theta = -angle_z;
  //SERIAL_ECHOLNPGM("  (IV_K) JOINT 3              => ", DEGREES(dh_para_cal.joints[3].theta));
  //SERIAL_ECHOLNPGM("  (IV_K) Anlge              => ", DEGREES(angle_z));

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
  const xyz_pos_t end_affector_temp_pos = { temp_matrix(0, 3), temp_matrix(1, 3), temp_matrix(2, 3) };

  // STEP 5:  Get vector to rotate X and Y axis based on temp vector and target position
  const float magnitude_end_affector = SQRT(sq(end_affector_temp_pos.x) + sq(end_affector_temp_pos.y) +sq(end_affector_temp_pos.z));
  const float magnitude_target = SQRT(sq(target.x) + sq(target.y) +sq(target.z));

  const float alpha_target = acos(target.x / magnitude_target);
  const float alpha_end_affector = acos(end_affector_temp_pos.x / magnitude_end_affector);
  
  const float beta_target = acos(target.y / magnitude_target);
  const float beta_end_affector = acos(end_affector_temp_pos.y / magnitude_end_affector);

  // joint angle = Target angle - temporary pos anlge 
  joint_1 -= alpha_target - alpha_end_affector;
  joint_2 -= beta_target - beta_end_affector;
 
  float delta_a = axis_x_angle_to_position(joint_1);
  float delta_b = axis_y_angle_to_position(joint_2);
  float delta_c = axis_z_angle_to_position(joint_3);

  // Limit range of motion
  LIMIT(delta_a, MIN_AXIS_TRAVEL, MAX_AXIS_TRAVEL);
  LIMIT(delta_b, MIN_AXIS_TRAVEL, MAX_AXIS_TRAVEL);
  LIMIT(delta_c, MIN_AXIS_TRAVEL, MAX_AXIS_TRAVEL);
  
  // STEP 6: Output angles to delta    
  delta.set(delta_a, delta_b, delta_c);

  //SERIAL_ECHOLNPGM("  (IV_K) Angles   => alpha_target: ", DEGREES(alpha_target), " | alpha_end_affector: ", DEGREES(alpha_end_affector), " | beta_target: ", DEGREES(beta_target), " | beta_end_affector: ", DEGREES(beta_end_affector));
  
  //SERIAL_ECHOLNPGM("  (IV_K) Angles Offset   => x: ", DEGREES(joint_1), " | y: ", DEGREES(joint_2), " | z: ", DEGREES(joint_3));
  //SERIAL_ECHOLNPGM("  (IV_K) Position => x: ", delta.a, " | y: ", delta.b, " | z: ", delta.c);
}

// Copied and adjusted from another kinematic system
void robot_arm_report_positions() {
  SERIAL_ECHOLNPGM(
    "Joint 1:", planner.get_axis_position_mm(X_AXIS)
  , "  Joint 2:", planner.get_axis_position_mm(Y_AXIS)
  , "  Joint 3:", planner.get_axis_position_mm(Z_AXIS)
  );
}


/*  Custom formula to convert angle to position for the X axis
*   INPUT: Angle in rads
*   OUTPUT: Position in mm
*/
float axis_x_angle_to_position(const_float_t angle) { 
  const float gamma = PI - angle;

  const float beta = asin(((axis_x_d1 - axis_x_d2) * sin(gamma)) / axis_x_d1);

  const float alpha_1 = PI - beta - gamma;
  
  const float alpha_3 = PI - axis_x_angle_offset_high - axis_x_angle_offset_low - alpha_1;
  
  const float length = SQRT(2 * sq(axis_x_d1) * (1 - cos(alpha_3)));
  
  float pos = length - axis_x_default_length;

  return pos;
}

/*  Custom formula to convert position to angle for the X axis
*   INPUT: position in mm
*   OUTPUT: Angle in rads
*/
float axis_x_position_to_angle(const_float_t pos) {
  const float length = pos + axis_x_default_length;
  

  const float alpha_3 = acos((2 * sq(axis_x_d1) - sq(length)) / (2 * sq(axis_x_d1)));

  const float alpha_1 = PI - axis_x_angle_offset_high - axis_x_angle_offset_low - alpha_3;
  
  const float chord = SQRT(sq(axis_x_d1 - axis_x_d2) + sq(axis_x_d1) - 2 * (axis_x_d1 - axis_x_d2) * axis_x_d1 * cos(alpha_1));

  const float angle = asin( (sin(alpha_1) * axis_x_d1) / chord);

  return angle;
}

/*  Custom formula to convert angle to position for the Y axis
*   INPUT: Angle in rads
*   OUTPUT: Position in mm
*/
float axis_y_angle_to_position(const_float_t angle) { 
  const float alpha_1 = angle + axis_y_angle_offset_low;

  const float beta = asin(((axis_y_d2 - axis_y_d1) * sin(alpha_1)) / axis_y_d1);

  const float gamma = PI - alpha_1 - beta;

  const float alpha_2 = PI - gamma;

  const float angle_3 = PI - axis_y_angle_offset_high - alpha_2;

  const float length = SQRT(2 * sq(axis_y_d1) * (1 - cos(angle_3)));

  float pos = length - axis_y_default_length;

  return pos;
}

/*  Custom formula to convert position to angle for the Y axis
*   INPUT: position in mm
*   OUTPUT: Angle in rads
*/
float axis_y_position_to_angle(const_float_t pos) {
  const float length = pos + axis_y_default_length;

  const float alpha_3 = acos((2 * sq(axis_y_d1) - sq(length)) / (2 * sq(axis_y_d1)));

  const float alpha_2 = PI - alpha_3 - axis_y_angle_offset_high;

  const float gamma = PI - alpha_2;

  const float chord = SQRT(sq(axis_y_d2 - axis_y_d1) + sq(axis_y_d1) - 2 * (axis_y_d2 - axis_y_d1) * axis_y_d1 * cos(gamma));

  const float alpha_1 = asin((sin(gamma) * axis_y_d1) / chord);

  const float angle = alpha_1 - axis_y_angle_offset_low;

  return angle;
}

/*  Custom formula to convert angle to position for the Z axis
*   INPUT: Angle in rads
*   OUTPUT: Position in mm
*/
float axis_z_angle_to_position(const_float_t angle) { 
  const float alpha_3 = PI - axis_z_angle_offset_low - axis_z_angle_offset_high - angle;

  const float length = SQRT(2 * sq(axis_z_d1) * (1 - cos(alpha_3)));

  float pos = length - axis_z_default_length;
  
  return pos;
}

/*  Custom formula to convert position to angle for the Z axis
*   INPUT: position in mm
*   OUTPUT: Angle in rads
*/
float axis_z_position_to_angle(const_float_t pos) {
  const float length = pos + axis_z_default_length;

  const float alpha_3 = acos((2* sq(axis_z_d1) - sq(length)) / (2 * sq(axis_z_d1)));

  const float angle = PI - axis_z_angle_offset_low - axis_z_angle_offset_high - alpha_3;

  return angle;
}

// Project a point onto a plane defined by a point and a normal (all as xyz_float_t)
xyz_float_t project_point_to_plane(const xyz_float_t &point, const xyz_float_t &plane_point, const xyz_float_t &plane_normal) {
  // Convert to BLA::Matrix<3> for math
  BLA::Matrix<3> p = {point.x, point.y, point.z};
  BLA::Matrix<3> pp = {plane_point.x, plane_point.y, plane_point.z};
  BLA::Matrix<3> n = {plane_normal.x, plane_normal.y, plane_normal.z};
  n = n / Norm(n);
  BLA::Matrix<3> v = p - pp;
  float distance = v(0)*n(0) + v(1)*n(1) + v(2)*n(2);
  BLA::Matrix<3> proj = p - distance * n;
  return xyz_float_t{proj(0), proj(1), proj(2)};
}

// Helper: Compute the transformation matrix up to (and including) a given joint index
BLA::Matrix<4,4> dh_transform_up_to(const DHParameters<N_joint>& dh_params, int joint_idx) {
  BLA::Matrix<4,4> T = {
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1
  };
  for (int i = 0; i <= joint_idx && i < N_joint; ++i) {
    T *= dh_transform(const_cast<JOINT&>(dh_params.joints[i]));
  }
  return T;
}


#endif // ROBOT_ARM
