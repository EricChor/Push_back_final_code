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

    set_pose(1,0,0);
    
    loader.set(UNLOAD);
    main_intake.spin(fwd,100, pct);
    final_intake_stage.spin(fwd,100,pct);
    drive_to_point(-9, 54, 150, 8, 0.8, 0, 0.45, 0, 0.1, 50, 0, 50, 4, 200);
    loader.set(LOAD);
    print_odom_pos();
    inertial_turn(40,225,10,0.6,0.1,0,4,100,30);
    drive_to_point_reverse_with_heading(10, 63,40, 150, 8, 0.8, 0, 0.45, 0, 0.1, 50, 0, 50, 6, 200);
    print_odom_pos();
    intermediate_intake_stage.spin(reverse,100,pct);
    final_intake_stage.spin(reverse,70,pct);
    vex::task::sleep(1500);
    intermediate_intake_stage.stop(coast);
    drive_to_point(-40, 26, 150, 8, 0.8, 0, 0.45, 0, 0.1, 45, 0, 45, 3, 200);
    inertial_turn(50,180,10,0.5,0,0,4,100,30);
    final_intake_stage.spin(fwd,100,pct);
    drive_straight(50,50,180,30,2,1,8,0,0,0.8,0,0,150);
    // vex::task::sleep(400);
    // drive_to_point(-40, -50, 4, 8, 0.8, 0, 0.45, 0, 0.1, 70, 0, 70, 3, 200);
    loader.set(UNLOAD);
    high_goal_aligner.set(ALIGN);
    drive_to_point_reverse_with_heading(-35, 40, 0,2.2, 8, 0.8, 0, 0.45, 0, 0.1, 45, 0, 45, 3, 200);
    intermediate_intake_stage.spin(reverse,100,pct);
    final_intake_stage.spin(fwd,100,pct);
    vex::task::sleep(2500);
    intermediate_intake_stage.stop(coast);
    high_goal_aligner.set(UNALIGN);
    drive_straight(60,60,180,10,2,4.0,100,0,0,0.8,0,0,150);
    vex::task::sleep(200);
    drive_straight(100,100,180,-20,2,4.0,8,0,0,0.8,0,0,150);;

    float final_time = master_timer.time(seconds);
    printf("final time:%f\n",final_time);
}

void right_side(){
    while(inertial_sensor.isCalibrating())
        vex::task::sleep(10);
    printf("inertial sensor calibrated\n");
    master_timer.clear();
    inertial_sensor.resetHeading();

    odom_setup(5.875,0,3.25,0,2.0/1.0);
    thread odometry(odom_thread);
    // thread telemetry(odom_telemetry);

    set_pose(-1,0,5);
    
    loader.set(UNLOAD);
    main_intake.spin(fwd,100, pct);
    final_intake_stage.spin(fwd,100,pct);
    drive_to_point(9, 54, 150, 8, 0.8, 0, 0.45, 0, 0.1, 55, 0, 55, 4, 200);
    loader.set(LOAD);
    print_odom_pos();
    inertial_turn(40,315,10,0.6,0.1,0,4,100,30);
    drive_to_point_reverse_with_heading(40, 18,135, 5, 8, 0.8, 0, 0.45, 0, 0.1, 50, 0, 50, 6, 200);
    print_odom_pos();
    inertial_turn(70,180,10,0.5,0,0,4,100,30);
    print_odom_pos();
    final_intake_stage.spin(fwd,100,pct);
    vex::task::sleep(500);
    drive_straight(65,65,180,30,3,0.7,8,0,0,0.8,0,0,150);
    print_odom_pos();
    // vex::task::sleep(400);
    // drive_to_point(-40, -50, 4, 8, 0.8, 0, 0.45, 0, 0.1, 70, 0, 70, 3, 200);
    loader.set(UNLOAD);
    high_goal_aligner.set(ALIGN);
    drive_to_point_reverse_with_heading(60, 40,15, 2.3, 8, 0.8, 0, 0.75, 0, 0.1, 40, 0, 30, 4, 200);
    print_odom_pos();
    intermediate_intake_stage.spin(reverse,100,pct);
    final_intake_stage.spin(fwd,100,pct);
    vex::task::sleep(2500);
    final_intake_stage.spin(reverse, 100,pct);
    intermediate_intake_stage.stop(coast);
    drive_straight(50,50,180,5,2,0.8,8,0,0,0.8,0,0,150);
    high_goal_aligner.set(UNALIGN);
    vex::task::sleep(250);
    loader.set(LOAD);
    drive_straight(100,100,180,-10,2,1.0,100,0,0,0.8,0,0,150);
    vex::task::sleep(100);
    drive_straight(40,40,180,90,2,4.0,8,0,0,0.8,0,0,150);
    drive_to_point(-40, 26, 150, 8, 0.8, 0, 0.45, 0, 0.1, 100, 0, 100, 3, 200);
    intermediate_intake_stage.spin(reverse,100,pct);
    final_intake_stage.spin(fwd,100,pct);
    vex::task::sleep(2500);
    intermediate_intake_stage.stop(coast);
    high_goal_aligner.set(UNALIGN);
    vex::task::sleep(200);
    drive_straight(100,100,180,-20,2,4.0,8,0,0,0.8,0,0,150);;

    float final_time = master_timer.time(seconds);
    printf("final time:%f\n",final_time);
}

void sawp(){
    while(inertial_sensor.isCalibrating())
    vex::task::sleep(10);
    printf("inertial sensor calibrated\n");
    master_timer.clear();
    inertial_sensor.resetHeading();

    odom_setup(5.875,0,3.25,0,2.0/1.0);
    thread odometry(odom_thread);
    // thread telemetry(odom_telemetry);

    set_pose(0,0,0);
    
    drive_to_point(80, 30, 50, 6, 0.8, 0, 0.45, 0, 0.1, 90, 0, 90, 10, 150);
    drive_straight(80,80,180,10,2,4.0,100,0,0,0.8,0,0,150);

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