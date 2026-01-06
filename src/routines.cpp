#include "vex.h"
#include "robot_config.h"
#include "autonomous.h"
#include "odometry.h"
#include "odom_auton.h"

void left_side_half_AWP(){
    while(inertial_sensor.isCalibrating())
        vex::task::sleep(10);
    printf("inertial sensor calibrated\n");
    master_timer.clear();
    inertial_sensor.resetHeading();

    odom_setup(1,0,3.25,0,4.0/2.0);
    thread odometry(odom_thread);

    set_pose(0,0,0);
    

    inertial_turn(100,90,20,1.0,
        0,0,3,100,20);

    float final_time = master_timer.time(seconds);
    printf("final time:%f\n",final_time);
}

void test_routine(){
    




    while(inertial_sensor.isCalibrating())
        vex::task::sleep(10);
    printf("inertial sensor calibrated\n");
    master_timer.clear();
    inertial_sensor.resetHeading();

    inertial_turn(100,90,20,1.0,
        0,0,3,100,20);

    float final_time = master_timer.time(seconds);
    printf("final time:%f\n",final_time);
}