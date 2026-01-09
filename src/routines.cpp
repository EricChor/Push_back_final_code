#include "vex.h"
#include "robot_config.h"
#include "autonomous.h"
#include "odometry.h"
#include "odom_auton.h"
#include "driver_control.h"

void left_side(){
    while(inertial_sensor.isCalibrating())
        vex::task::sleep(10);
    printf("inertial sensor calibrated\n");
    master_timer.clear();
    inertial_sensor.resetHeading();

    odom_setup(5.875,0,3.25,0,2.0/1.0);
    thread odometry(odom_thread);
    // thread telemetry(odom_telemetry);

    set_pose(0,0,0);
    
    main_intake.spin(fwd,100, pct);
    final_intake_stage.spin(fwd,100,pct);
    drive_to_point(-9, 54, 150, 8, 0.8, 0, 0.45, 0, 0.1, 45, 0, 50, 4, 200);
    print_odom_pos();
    
    inertial_turn(50,225,10,0.6,0.1,0,4,100,30);
    drive_to_point_reverse_with_heading(11, 63,40, 150, 8, 0.8, 0, 0.45, 0, 0.1, 30, 0, 40, 6, 200);
    print_odom_pos();
    
    intermediate_intake_stage.spin(reverse,100,pct);
    final_intake_stage.spin(reverse,70,pct);
    vex::task::sleep(1250);

    intermediate_intake_stage.stop(coast);
    drive_to_point(-40, 26, 150, 8, 0.8, 0, 0.45, 0, 0.1, 55, 0, 70, 3, 200);
    inertial_turn(70,180,10,0.5,0,0,4,100,30);
    final_intake_stage.spin(fwd,100,pct);

    loader.set(LOAD);
    drive_straight(40,30,180,30,2,2.5,8,0,0,0.8,0,0,150);
    // vex::task::sleep(400);
    // drive_to_point(-40, -50, 4, 8, 0.8, 0, 0.45, 0, 0.1, 70, 0, 70, 3, 200);
    high_goal_aligner.set(ALIGN);
    drive_to_point_reverse_with_heading(-40, 40, 0,3, 8, 0.8, 0, 0.45, 0, 0.1, 50, 0, 50, 3, 200);
    intermediate_intake_stage.spin(reverse,100,pct);
    final_intake_stage.spin(fwd,100,pct);

    vex::task::sleep(3000);

    intermediate_intake_stage.stop(coast);
    final_intake_stage.stop(coast);

    left_drive.spin(fwd,100,pct);
    right_drive.spin(fwd,100,pct);

    vex::task::sleep(250);

    left_drive.spin(reverse,100,pct);
    right_drive.spin(reverse,100,pct);

    vex::task::sleep(500);

    left_drive.stop(coast);
    right_drive.stop(coast);

    float final_time = master_timer.time(seconds);
    printf("final time:%f\n",final_time);
}

void test_routine(){
    




    while(inertial_sensor.isCalibrating())
        vex::task::sleep(10);
    printf("inertial sensor calibrated\n");
    master_timer.clear();
    inertial_sensor.resetHeading();


    float final_time = master_timer.time(seconds);
    printf("final time:%f\n",final_time);
}