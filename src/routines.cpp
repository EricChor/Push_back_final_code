#include "vex.h"
#include "robot_config.h"
#include "autonomous.h"
#include "odometry.h"
#include "odom_auton.h"
#include "driver_control.h"


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

void routine_right(){
    while(inertial_sensor.isCalibrating())

        vex::task::sleep(10);
    printf("inertial sensor calibrated\n");
    master_timer.clear();
    inertial_sensor.resetHeading();

    odom_setup(5.875,0,3.25,0,2.0/1.0);

    thread odometry(odom_thread);

    set_pose(0,0,0);

    drive_to_point(6,24,3,8,0.8,0,0.45,0,0.1,45,15,50,4,200);
     print_odom_pos();

    main_intake.spin(fwd,100,pct);
    final_intake_stage.spin(fwd,100,pct);

    vex::task::sleep(100);

    drive_to_point(11,56,3,8,0.8,0,0.45,0,0.1,45,0,50,4,200);
     print_odom_pos();

    vex::task::sleep(100);

    turn_to_point(50.5,15.5,2,3,0.45,0,0.1,36,150,200);
    print_odom_pos();

    loader.set(LOAD);

    drive_to_point(52,25,3,8,0.8,0,0.45,0,0.1,45,0,50,4,200);
     print_odom_pos();

     inertial_turn(45,180,3,0.45,0,0.1,4,150,200);

    

    drive_straight(40,30,180,12,2,3,8,0.8,0,0.45,0,0.1,200);

    vex::task::sleep(100);

    drive_to_point_reverse_with_heading(54,30,0,3,8,0.8,0,0.45,0,0.1,45,15,50,4,200);
     print_odom_pos();

    loader.set(UNLOAD);

    high_goal_aligner.set(ALIGN);


    drive_to_point_reverse_with_heading(48.5,58,0,1,8,0.8,0,0.45,0,0.1,45,0,50,4,200);
     print_odom_pos();

    

    main_intake.spin(reverse,60,pct);

    left_drive.spin(fwd,100,pct);
    right_drive.spin(fwd,100,pct);
    vex::task::sleep(200);
    left_drive.spin(reverse,100,pct);
    right_drive.spin(reverse,100,pct);
    vex::task::sleep(300);

    // left_drive.spin(fwd,40,pct);
    // right_drive.spin(fwd,40,pct);

    // vex::task::sleep(100);

    left_drive.stop(hold);
    right_drive.stop(hold);

    main_intake.spin(fwd,100,pct);
     intermediate_intake_stage.spin(reverse,100,pct);




     float final_time = master_timer.time(seconds);
     printf("final time:%f\n",final_time);




}

void routine_left(){
    while(inertial_sensor.isCalibrating())

        vex::task::sleep(10);
    printf("inertial sensor calibrated\n");
    master_timer.clear();
    inertial_sensor.resetHeading();

    odom_setup(5.875,0,3.25,0,2.0/1.0);

    thread odometry(odom_thread);

    set_pose(0,0,0);


    drive_to_point(-6,24,3,8,0.8,0,0.45,0,0.1,45,15,50,4,200);
     print_odom_pos();

    main_intake.spin(fwd,100,pct);
    final_intake_stage.spin(fwd,100,pct);

    drive_to_point(-11,56,3,8,0.8,0,0.45,0,0.1,45,0,50,4,200);
     print_odom_pos();

     turn_to_point_reverse(5,65,2,3,0.45,0,0.1,36,150,200);
    
    drive_to_point_reverse_with_heading(5,67,45,1,8,0.8,0,0.45,0,0.1,45,0,50,4,200);
     print_odom_pos();

     

    final_intake_stage.spin(reverse,100,pct);
    intermediate_intake_stage.spin(reverse,70,pct);

    vex::task::sleep(2000);
    left_drive.spin(fwd,30,pct);
    right_drive.spin(fwd,30,pct);
    vex::task::sleep(200);
    left_drive.spin(reverse,30,pct);
    right_drive.spin(reverse,30,pct);
    vex::task::sleep(2000);
    left_drive.stop(coast);
    right_drive.stop(coast);
    
    

    float final_time = master_timer.time(seconds);
     printf("final time:%f\n",final_time);

} 

