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
extern motor intake;

//timers
extern timer master_timer;

//loader pneumatic
extern digital_out loader;