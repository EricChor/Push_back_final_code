#include "vex.h"
#include "robot_config.h"
#include "autonomous.h"

void test_routine(){
    while(inertial_sensor.isCalibrating())
        vex::task::sleep(10);
    printf("inertial sensor calibrated\n");
    master_timer.clear();
    inertial_sensor.resetHeading();
    left_middle_motor.resetPosition();

    inertial_turn(100,90,20,1.0,
        0,0,3,100,20);

    float final_time = master_timer.time(seconds);
    printf("final time:%f\n",final_time);
}