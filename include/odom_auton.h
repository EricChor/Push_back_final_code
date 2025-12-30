#ifndef ODOM_AUTON
#define ODOM_AUTON
#pragma once

extern void turn_to_point(double target_x_pos, double target_y_pos, float acceptable_error, float max_time, float kP, float kI, float kD, float max_speed, int settle_delay, float settle_speed);

extern void turn_to_point_reverse(double target_x_pos, double target_y_pos, float acceptable_error, float max_time, float kP, float kI, float kD, float max_speed, int settle_delay, float settle_speed);

extern void drive_to_point (double target_x_pos, double target_y_pos, float max_time, float linear_kP, float linear_kI, float linear_kD, float angular_kP, float angular_kI, float angular_kD, float max_linear_speed, float min_linear_speed, float max_angular_speed, float dead_zone_distance, float max_accel);

extern void drive_to_point_reverse (double target_x_pos, double target_y_pos, float max_time, float linear_kP, float linear_kI, float linear_kD, float angular_kP, float angular_kI, float angular_kD, float max_linear_speed, float min_linear_speed, float max_angular_speed, float dead_zone_distance, float max_accel);

extern void move_to_pose(double target_x_pos, double target_y_pos, float target_angle, float lead_distance, float setback_distance, float max_time, float linear_kP, float linear_kI, float linear_kD, float angular_kP, float angular_kI, float angular_kD, float max_linear_speed, float min_linear_speed, float max_angular_speed, float dead_zone_distance, float carrot_distance, float max_accel);

extern void move_to_pose_reverse(double target_x_pos, double target_y_pos, float target_angle, float lead_distance, float setback_distance, float max_time, float linear_kP, float linear_kI, float linear_kD, float angular_kP, float angular_kI, float angular_kD, float max_linear_speed, float min_linear_speed, float max_angular_speed, float dead_zone_distance, float carrot_distance, float max_accel);

extern void drive_to_point_with_heading (double target_x_pos, double target_y_pos, float target_heading, float max_time, float linear_kP, float linear_kI, float linear_kD, float angular_kP, float angular_kI, float angular_kD, float max_linear_speed, float min_linear_speed, float max_angular_speed, float dead_zone_distance, float max_accel);

extern float global_min_speed;

#endif