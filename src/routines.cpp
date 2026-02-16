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
    drive_to_point(-6, 40, 150, 8, 0.8, 0, 0.45, 0, 0.1, 50, 0, 50, 4, 200);
    loader.set(LOAD);
    print_odom_pos();
    inertial_turn(40,225,10,0.6,0.1,0,4,100,30);
    drive_to_point_reverse_with_heading(5, 50,40, 1, 8, 0.8, 0, 0.45, 0, 0.1, 50, 0, 50, 6, 200);
    print_odom_pos();
    intermediate_intake_stage.spin(reverse,100,pct);
    final_intake_stage.spin(reverse,70,pct);
    vex::task::sleep(1500);
    intermediate_intake_stage.stop(coast);
    drive_to_point(-31, 18, 150, 8, 0.8, 0, 0.45, 0, 0.1, 45, 0, 45, 3, 200);
    inertial_turn(50,180,10,0.5,0,0,4,100,30);
    final_intake_stage.spin(fwd,100,pct);
    drive_straight(40,40,183,60,2,1.4,8,0,0,0.8,0,0,150);
    // vex::task::sleep(400);
    // drive_to_point(-40, -50, 4, 8, 0.8, 0, 0.45, 0, 0.1, 70, 0, 70, 3, 200);
    loader.set(UNLOAD);
    high_goal_aligner.set(ALIGN);
    drive_to_point_reverse_with_heading(-29, 45, 0,2.2, 8, 0.8, 0, 0.45, 0, 0.1, 45, 0, 45, 3, 200);
    intermediate_intake_stage.spin(reverse,100,pct);
    final_intake_stage.spin(fwd,100,pct);
    vex::task::sleep(2500);
    intermediate_intake_stage.stop(coast);
    high_goal_aligner.set(UNALIGN);
    inertial_turn(70,90,10,0.5,0,0,4,100,30);
    drive_straight(70,70,90,9,2,0.8,8,0,0,0.8,0,0,150);
    inertial_turn(70,180,10,0.5,0,0,4,100,30);
    drive_straight(50,50,182,-25,2,0.8,8,0,0,0.8,0,0,150);

    float final_time = master_timer.time(seconds);
    printf("final time:%f\n",final_time);
}

void left_descore_only(){
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
    drive_to_point(-6, 40, 150, 8, 0.8, 0, 0.45, 0, 0.1, 50, 0, 50, 4, 200);
    loader.set(LOAD);
    print_odom_pos();
    inertial_turn(40,225,10,0.6,0.1,0,4,100,30);
    print_odom_pos();
    drive_to_point(-15, 18, 150, 8, 0.8, 0, 0.45, 0, 0.1, 45, 0, 45, 3, 200);
    inertial_turn(50,180,10,0.5,0,0,4,100,30);
    final_intake_stage.spin(fwd,100,pct);
    drive_straight(40,40,183,60,2,1.4,8,0,0,0.8,0,0,150);
    // vex::task::sleep(400);
    // drive_to_point(-40, -50, 4, 8, 0.8, 0, 0.45, 0, 0.1, 70, 0, 70, 3, 200);
    loader.set(UNLOAD);
    high_goal_aligner.set(ALIGN);
    drive_to_point_reverse_with_heading(-29, 45, 0,2.2, 8, 0.8, 0, 0.45, 0, 0.1, 45, 0, 45, 3, 200);
    intermediate_intake_stage.spin(reverse,100,pct);
    final_intake_stage.spin(fwd,100,pct);
    vex::task::sleep(2500);
    intermediate_intake_stage.stop(coast);
    high_goal_aligner.set(UNALIGN);
    inertial_turn(70,90,10,0.5,0,0,4,100,30);
    drive_straight(70,70,90,9,2,0.8,8,0,0,0.8,0,0,150);
    inertial_turn(70,180,10,0.5,0,0,4,100,30);
    drive_straight(50,50,182,-25,2,0.8,8,0,0,0.8,0,0,150);

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
    drive_to_point(6.5, 34, 150, 8, 0.8, 0, 0.45, 0, 0.1, 55, 0, 55, 4, 200);
    loader.set(LOAD);
    print_odom_pos();
    inertial_turn(40,315,10,0.6,0.1,0,4,5,30);
    drive_to_point_reverse_with_heading(32, 18,135, 5, 8, 0.8, 0, 0.45, 0, 0.1, 55, 0, 55, 6, 200);
    print_odom_pos();
    inertial_turn(60,180,10,0.5,0,0,4,100,30);
    print_odom_pos();
    final_intake_stage.spin(fwd,100,pct);
    vex::task::sleep(500);
    drive_straight(60,60,180,30,3,0.8,8,0,0,0.8,0,0,150);
    print_odom_pos();
    // vex::task::sleep(400);
    // drive_to_point(-40, -50, 4, 8, 0.8, 0, 0.45, 0, 0.1, 70, 0, 70, 3, 200);
    drive_straight(40,40,180,-5,3,1.2,8,0,0,0.8,0,0,150);
    loader.set(UNLOAD);
    high_goal_aligner.set(ALIGN);
    drive_to_point_reverse_with_heading(43, 30,15, 2.3, 8, 0.8, 0, 0.75, 0, 0.1, 70, 0, 70, 4, 200);
    print_odom_pos();
    intermediate_intake_stage.spin(reverse,100,pct);
    final_intake_stage.spin(fwd,100,pct);
    vex::task::sleep(2500);
    final_intake_stage.spin(reverse, 100,pct);
    intermediate_intake_stage.stop(coast);
    drive_straight(70,70,180,5,2,0.8,8,0,0,0.8,0,0,150);
    high_goal_aligner.set(UNALIGN);
    inertial_turn(70,90,10,0.5,0,0,4,100,30);
    drive_straight(70,70,90,12,2,0.8,8,0,0,0.8,0,0,150);
    inertial_turn(70,180,10,0.5,0,0,4,100,30);
    drive_straight(50,50,182,-25,2,0.8,8,0,0,0.8,0,0,150);

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

    set_pose(0,0,5);
    
    loader.set(LOAD);
    main_intake.spin(fwd,100, pct);
    final_intake_stage.spin(fwd,100,pct);
    drive_to_point(0, 26, 10, 8, 0.8, 0, 1, 0, 0.1, 70, 0, 70, 4, 200);
    vex::task::sleep(800); 
    inertial_turn(70,90,10,0.5,0,0,4,100,30);
    drive_straight(80,80,90,80,0.6,1,8,0,0,0.8,0,0,150);    
    vex::task::sleep(400); 
    loader.set(UNLOAD);
    high_goal_aligner.set(ALIGN);
    drive_straight(40,40,100,-30,2,1.0,8,0,0,0.8,0,0,150);    
    intermediate_intake_stage.spin(reverse,100,pct);    
    vex::task::sleep(1000);
    high_goal_aligner.set(UNALIGN);
    intermediate_intake_stage.stop(coast);
    drive_to_point(0, 30, 10, 8, 0.8, 0, 0.45, 0, 0.1, 70, 0, 70, 4, 200);
    inertial_turn(70,225,10,0.5,0,0,4,100,30);
    drive_to_point(-26, 2, 10, 8, 0.8, 0, 0.45, 0, 0.1, 70, 0, 70, 4, 200);
    loader.set(LOAD);
    inertial_turn(70,180,10,0.5,0,0,4,100,30);
    loader.set(UNLOAD);
    drive_to_point(-26, -46, 10, 8, 0.8, 0, 0.45, 0, 0.1, 70, 0, 70, 4, 200);
    loader.set(LOAD);
    inertial_turn(70,180,10,0.5,0,0,4,100,30);
    drive_to_point_reverse_with_heading(-42, -32,15, 2.3, 8, 0.8, 0, 0.75, 0, 0.1, 70, 0, 70, 4, 200);
    intermediate_intake_stage.spin(reverse,100,pct);    
    final_intake_stage.spin(reverse,100,pct);
    vex::task::sleep(1000);
    intermediate_intake_stage.stop(coast);
    final_intake_stage.spin(fwd,100,pct);
    drive_to_point(0, -66, 10, 8, 0.8, 0, 0.45, 0, 0.1, 70, 0, 70, 4, 200);

    float final_time = master_timer.time(seconds);
    printf("final time:%f\n", final_time);
}

void skills_auton(){
    while(inertial_sensor.isCalibrating())
        vex::task::sleep(10);
    printf("inertial sensor calibrated\n");
    master_timer.clear();
    inertial_sensor.resetHeading();

    odom_setup(5.875,0,3.25,0,2.0/1.0);
    thread odometry(odom_thread);
    // thread telemetry(odom_telemetry);

    set_pose(0,0,5);
    
    main_intake.spin(fwd,100, pct);
    drive_straight(70,70,0,10,2,10,8,0,0,0.8,0,0,150);
    drive_straight(70,70,0,-10,2,10,8,0,0,0.8,0,0,150);
    drive_straight(70,70,0,10,2,10,8,0,0,0.8,0,0,150);
    drive_straight(70,70,0,-10,2,10,8,0,0,0.8,0,0,150);
    drive_straight(70,70,0,10,2,10,8,0,0,0.8,0,0,150);
    drive_straight(70,70,0,-10,2,10,8,0,0,0.8,0,0,150);
    drive_straight(70,70,0,10,2,10,8,0,0,0.8,0,0,150);

    
    float final_time = master_timer.time(seconds);
    printf("final time:%f\n",final_time);
}

void asawp(){
    while(inertial_sensor.isCalibrating())
        vex::task::sleep(10);
    printf("inertial sensor calibrated\n");
    master_timer.clear();
    inertial_sensor.resetHeading();

    odom_setup(5.875,0,3.25,0,2.0/1.0);
    thread odometry(odom_thread);
    // thread telemetry(odom_telemetry);

    set_pose(0,0,5);
    
    drive_straight(70,70,0,3,2,10,8,0,0,0.8,0,0,150);

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

void square(){
    while(inertial_sensor.isCalibrating())

        vex::task::sleep(10);
    printf("inertial sensor calibrated\n");
    master_timer.clear();
    inertial_sensor.resetHeading();

    odom_setup(-3.2,7.9,2.75,2.0,1.0/1.0);

    thread odometry(odom_thread);
    // thread telemeytry(odom_telemetry);
    vex::task::sleep(100);

    set_pose(0,0,0);

    drive_straight(60,60,0,48,2,10,8,0,0,0.8,0,0,150);
    inertial_turn(70,90,10,0.5,0,0,5,100,30);
    drive_straight(60,60,90,48,2,10,8,0,0,0.8,0,0,150);
    inertial_turn(70,180,10,0.5,0,0,5,100,30);
    drive_straight(60,60,180,48,2,10,8,0,0,0.8,0,0,150);
    inertial_turn(70,270,10,0.5,0,0,5,100,30);
    drive_straight(60,60,270,48,2,10,8,0,0,0.8,0,0,150);
    inertial_turn(70,0,10,0.5,0,0,5,100,30);

    float final_time = master_timer.time(seconds);
     printf("final time:%f\n",final_time);
    



}
