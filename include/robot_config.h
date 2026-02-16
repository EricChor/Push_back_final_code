#pragma once
#include "vex.h"
using namespace vex;

//brain
extern brain Brain;

//controller
extern controller Controller;

//left_drive_train
extern motor left_front_motor;
extern motor left_middle_motor;
extern motor left_rear_motor;

//right_drive_train
extern motor right_front_motor;
extern motor right_middle_motor;
extern motor right_rear_motor;

//drivetrain_motor_groups
extern motor_group left_drive;
extern motor_group right_drive;

//inertial sensor
extern inertial inertial_sensor;

//intake
extern motor main_intake;
extern motor final_intake_stage;
extern motor intermediate_intake_stage;

//timers
extern timer master_timer;

//loader pneumatic

extern digital_out loader;
extern digital_out tracking_wheel_cylinder;
extern digital_out descorer;
extern digital_out high_goal_aligner;

extern optical color_sorting_sensor;

enum alliance_color{
    RED, //0
    BLUE, //1
    NEUTRAL //2
};

extern alliance_color current_alliance_color;

extern rotation lateral_tracking;

extern rotation vertical_tracking;

extern distance massive_distance_sensor;