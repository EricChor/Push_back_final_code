#include "vex.h"
#include "robot_config.h"

//brain
brain Brain;

//controller
controller Controller(primary);

//red = 36:1
//green = 18:1
//blue = 6:1

// radio = 15

//left drive train
// motor left_front_motor(PORT8, ratio6_1,true);
// motor left_rear_bottom_motor(PORT19, ratio6_1, true);
// motor left_rear_top_motor(PORT20, ratio6_1,false);
motor left_front_motor(PORT8, ratio6_1,true);
motor left_rear_bottom_motor(PORT19, ratio6_1, true);
motor left_rear_top_motor(PORT20, ratio6_1,false);  

//right drive train
// motor right_front_motor(PORT17, ratio6_1,false);
// motor right_rear_bottom_motor (PORT13, ratio6_1,false);
// motor right_rear_top_motor(PORT14, ratio6_1, true);
motor right_front_motor(PORT1, ratio6_1,false);
motor right_rear_bottom_motor (PORT9, ratio6_1,false);
motor right_rear_top_motor(PORT10, ratio6_1, true);

//drivebase_motor_groups
motor_group left_drive(left_front_motor,left_rear_bottom_motor,left_rear_top_motor);
motor_group right_drive(right_front_motor,right_rear_bottom_motor,right_rear_top_motor);

//inertial_sensor
inertial inertial_sensor(PORT5);

//intake
// motor main_intake(PORT15, ratio6_1, false);
// motor final_intake_stage(PORT10, ratio18_1,false);
// motor intermediate_intake_stage(PORT16, ratio18_1, false);
motor main_intake(PORT7, ratio6_1, false);
motor final_intake_stage(PORT3, ratio18_1,false);
motor intermediate_intake_stage(PORT6, ratio18_1, false);

//timer
timer master_timer;

// optical color_sorting_sensor(PORT2);
optical color_sorting_sensor(PORT11);

digital_out loader(Brain.ThreeWirePort.A);
digital_out tracking_wheel_cylinder(Brain.ThreeWirePort.C);
digital_out descorer(Brain.ThreeWirePort.G);
digital_out high_goal_aligner(Brain.ThreeWirePort.H);

alliance_color current_alliance_color = BLUE;

rotation lateral_tracking(PORT18);

rotation vertical_tracking(PORT17,true);

distance massive_distance_sensor(PORT2);