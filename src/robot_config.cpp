#include "vex.h"
#include "robot_config.h"

//brain
brain Brain;

//controller
controller Controller(primary);

//red = 36:1
//green = 18:1
//blue = 6:1

//left drive train
motor left_front_motor(PORT17, ratio6_1,true);
motor left_rear_bottom_motor(PORT19, ratio6_1, true);
motor left_rear_top_motor(PORT18, ratio6_1,false);

//right drive train
motor right_front_motor(PORT8, ratio6_1,false);
motor right_rear_bottom_motor (PORT9, ratio6_1,false);
motor right_rear_top_motor(PORT10, ratio6_1, true);

//drivebase_motor_groups
motor_group left_drive(left_front_motor,left_rear_bottom_motor,left_rear_top_motor);
motor_group right_drive(right_front_motor,right_rear_bottom_motor,right_rear_top_motor);

//inertial_sensor
inertial inertial_sensor(PORT17);

//intake
motor intake(PORT10, ratio6_1, true);

//timer
timer master_timer;

//arm rotation sensor
rotation rotation_sensor(PORT14,true);

//intake arm
motor intake_arm(PORT16, ratio36_1, false);

//loader pneumatic
digital_out loader(Brain.ThreeWirePort.A);