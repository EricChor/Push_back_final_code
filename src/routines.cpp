#include "vex.h"
#include "robot_config.h"
#include "autonomous.h"
#include "odometry.h"
#include "odom_auton.h"
#include "driver_control.h"


void right_low_auton(){
while(inertial_sensor.isCalibrating())

        vex::task::sleep(10);
    printf("inertial sensor calibrated\n");
    master_timer.clear();
    inertial_sensor.resetHeading();

    lower_intake.spin(fwd, 100, pct);
    high_intake.spin(fwd, 100, pct);
    drive_straight(100,100,0  ,12,2,3,8,0.8,0,0.45,0,0.1,200);
    inertial_turn(100,90,3,0.45,0,0.1,4,150,200);
    drive_straight(50,50,90  ,13,2,3,8,0.8,0,0.45,0,0.1,200);
    vex::task::sleep(300);
    lower_intake.stop();
    high_intake.stop();
    inertial_turn(100,315,3,0.45,0,0.1,4,150,200);
    drive_straight(100,100,315,11,2,3,8,0.8,0,0.45,0,0.1,200);
    lower_intake.spin(reverse, 100, pct);
    high_intake.spin(reverse, 100, pct);
    vex::task::sleep(2000);
    lower_intake.spin(fwd, 100, pct);
    high_intake.spin(fwd, 100, pct);
    drive_straight(100,100,315,-20,2,3,8,0.8,0,0.45,0,0.1,200);
    drive_straight(100,100,315,10,2,3,8,0.8,0,0.45,0,0.1,200);
    inertial_turn(100,0,3,0.45,0,0.1,4,150,200);

    float final_time = master_timer.time(seconds);
    printf("final time:%f\n",final_time);
}

void defense_auton(){
while(inertial_sensor.isCalibrating())

        vex::task::sleep(10);
    printf("inertial sensor calibrated\n");
    master_timer.clear();
    inertial_sensor.resetHeading();

    drive_straight(100,100,0,30,2,3,8,0.8,0,0.45,0,0.1,200);
    inertial_turn(100,90,3,0.45,0,0.1,4,150,200);
    drive_straight(100,100,90,13,2,3,8,0.8,0,0.45,0,0.1,200);
    lower_intake.stop();
    high_intake.stop();
    inertial_turn(100,0,3,0.45,0,0.1,4,150,200);
    drive_straight(100,100,0,-1000000000,2,105,8,0.8,0,0.45,0,0.1,200);

    float final_time = master_timer.time(seconds);
    printf("final time:%f\n",final_time);
}