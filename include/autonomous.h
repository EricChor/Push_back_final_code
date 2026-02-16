#pragma once

extern void inertial_turn(int max_speed, float target_heading, float max_time, float kP, 
    float kI, float kD, float settle_error, int settle_time, float settle_speed);

extern void drive_straight(int max_linear_speed, int max_angular_speed, float target_heading, float target_distance,
    float acceptable_error, float max_time, float linear_kP, float linear_kI, float linear_kD, float angular_kP,
    float angular_kI, float angular_kD, float max_accel);

extern void ultimate_drive_straight(int max_linear_speed, int max_angular_speed, float target_heading, float target_distance,
    float acceptable_error, float max_time, float linear_kP, float linear_kI, float linear_kD, float angular_kP,
    float angular_kI, float angular_kD, float max_accel);

extern void wall_alignment(int max_linear_speed, int max_angular_speed, float target_heading, float target_distance, float acceptable_error, float max_time, float linear_kP, float linear_kI, float linear_kD, float angular_kP,float angular_kI, float angular_kD, float max_accel);

extern void color_sorting_intake();