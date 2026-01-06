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

    odom_setup(5.875,0,3.25,0,2.0/1.0);
    thread odometry(odom_thread);

    set_pose(0,0,0);
    
    // thread telemetry(odom_telemetry);

    main_intake.spin(fwd,100,pct);

    // turn_to_point(-24,30,4,6,0.8,0.1,0.25,30,150,150);

    drive_to_point(-24,30,6,8,0,1,0.5,0,0.1,35,0,35,3,100);
    print_odom_pos();

    turn_to_point_reverse(12,54,4,6,0.8,0.1,0.25,30,150,150);

    left_drive.spin(fwd,100,pct);
    right_drive.spin(fwd,100,pct);
    vex::task::sleep(125);
    left_drive.spin(reverse,100,pct);
    right_drive.spin(reverse,100,pct);
    vex::task::sleep(125);
    left_drive.stop(coast);
    right_drive.stop(coast);

    drive_to_point_reverse_with_heading(2,48,45,4,8,0,1,0.75,0,0.1,35,0,35,3,100);
    print_odom_pos();

    main_intake.spin(fwd,100,pct);
    intermediate_intake_stage.spin(reverse,100,pct);
    final_intake_stage.spin(fwd,100,pct);


    vex::task::sleep(1000);




    main_intake.stop();
    intermediate_intake_stage.stop();
    final_intake_stage.stop();
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