#pragma once

extern double x_pos;

extern double y_pos;

extern double theta; //angle in degrees

extern double vertical_encoder_distance_from_arc_center; 

extern double lateral_encoder_distance_from_arc_center;

extern double vertical_encoder_pos;

extern double prev_vertical_encoder_pos;

extern double lateral_encoder_pos;

extern double prev_lateral_encoder_pos;

extern double theta_pos;

extern double prev_theta_pos;

extern double drivebase_gear_ratio;

extern void set_pose(double initial_x_pos,double initial_y_pos,double starting_theta);

extern void update_pose();

//for distance from center : vertical : right is positive, left is negative
                         //: lateral  : forward is negative, backwards is positive (Both from facing up)
extern void odom_setup(double vertical_encoder_distance_from_center,double lateral_encoder_distance_from_center, double vertical_wheel_diam, double lateral_wheel_diam, double gear_ratio);

extern void odom_telemetry();

extern void odom_thread();

extern void print_odom_pos();

