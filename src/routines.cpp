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


    tracking_wheel_cylinder.set(true);
    main_intake.spin(fwd,100,pct);
    final_intake_stage.spin(fwd,100,pct);

    left_drive.spin(fwd,60,pct);
    right_drive.spin(fwd,60,pct);

    

    vex::task::sleep(1200);
    left_drive.stop(coast);
    right_drive.stop(coast);

    float final_time = master_timer.time(seconds);
    printf("final time:%f\n",final_time);
}

void routine_right(){
    while(inertial_sensor.isCalibrating())

        vex::task::sleep(10);
    printf("inertial sensor calibrated\n");
    master_timer.clear();
    inertial_sensor.resetHeading();

    odom_setup(-2.8,7.95,2.75,2.0,1.0/1.0);

    thread odometry(odom_thread);
    vex::task::sleep(100);

    set_pose(0,0,0);

    // drive_to_point(3,24,3,8,0.8,0,0.45,0,0.1,45,25,50,3,200);
    //  print_odom_pos();

    main_intake.spin(fwd,100,pct);
    final_intake_stage.spin(fwd,100,pct);


    drive_to_point(7,40,5,8,0.1,0,0.45,0,0.1,50,0,50,3,200);
     print_odom_pos();

    vex::task::sleep(200);

    turn_to_point(24,19,5,3,0.53,0.1,0.175,45,0,200);
    print_odom_pos();

    loader.set(LOAD);

    drive_to_point(26,19, 12,6,0,0,0.45,0,0.1,40,0,50,3,200);
    print_odom_pos();

    inertial_turn(45,180,3,0.45,0,0.1,4,150,200);

    drive_straight(30,30,180,16,2,5,8,0.8,0,0.45,0,0.1,200);
    print_odom_pos();

    vex::task::sleep(100);

    drive_to_point_reverse_with_heading(27,20,0,3,8,0.8,0,0.45,0,0.1,20,0,15,2,200);
    print_odom_pos();

    loader.set(UNLOAD);

    high_goal_aligner.set(ALIGN);
    main_intake.spin(reverse,20,pct);


    drive_to_point_reverse_with_heading(27,40,0,1.5,8,0.8,0,0.45,0,0.1,25,0,15,2,200);
     print_odom_pos();


    main_intake.spin(fwd,100,pct);
    intermediate_intake_stage.spin(reverse,100,pct);

    vex::task::sleep(3000);

    intermediate_intake_stage.stop(coast);




    //  descorer.set(DESCORE);
    // high_goal_aligner.set(UNALIGN);
    //  drive_straight(25,30,180,12,2,4,8,0.8,0,0.45,0,0.1,200);
    // print_odom_pos();

    // drive_to_point_reverse_with_heading(44,43,0,1,8,0.8,0,0.45,0,0.1,35,0,50,2,200);
    //  print_odom_pos();

    // drive_to_point_reverse_with_heading(45,48,0,1,8,0.8,0,0.45,0,0.1,35,0,50,4,200);
    //  print_odom_pos();

    // descorer.set(UNDESCORE);
    
    // drive_straight(40,30,180,-12,2,4,8,0.8,0,0.45,0,0.1,200);
    // print_odom_pos();

      float final_time = master_timer.time(seconds);
     printf("final time:%f\n",final_time);
    



}

void routine_left(){
    while(inertial_sensor.isCalibrating())

        vex::task::sleep(10);
    printf("inertial sensor calibrated\n");
    master_timer.clear();
    inertial_sensor.resetHeading();

    odom_setup(-2.8,7.95,2.75,2.0,1.0/1.0);

    thread odometry(odom_thread);
    vex::task::sleep(100);

    set_pose(0,0,0);

    // drive_to_point(3,24,3,8,0.8,0,0.45,0,0.1,45,25,50,3,200);
    //  print_odom_pos();

    main_intake.spin(fwd,100,pct);
    final_intake_stage.spin(fwd,100,pct);


    drive_to_point(-7,40,5,8,0.1,0,0.45,0,0.1,50,0,50,3,200);
     print_odom_pos();

    vex::task::sleep(200);

    turn_to_point_reverse(1,55,5,3,0.53,0.1,0.175,45,0,200);
    print_odom_pos();

    drive_to_point_reverse_with_heading(1,55,45,5,8,0.1,0,0.45,0,0.1,50,0,50,3,200);
    print_odom_pos();

    intermediate_intake_stage.spin(reverse,100,pct);
    final_intake_stage.spin(reverse,70,pct);

    vex::task::sleep(1500);
    intermediate_intake_stage.stop(coast);
    final_intake_stage.stop(coast);
    main_intake.stop(coast);
    loader.set(LOAD);

    drive_to_point(-35,20,8,8,0.1,0,0.45,0,0.1,40,0,50,3,200);
    print_odom_pos();

    main_intake.spin(fwd,100,pct);
    final_intake_stage.spin(fwd,100,pct);

    inertial_turn(45,180,3,0.45,0,0.1,4,150,200);

    drive_straight(45,30,180,16,2,2,8,0.8,0,0.45,0,0.1,200);
    print_odom_pos();

    vex::task::sleep(700);

    drive_to_point_reverse_with_heading(-35,25,0,3,8,0.8,0,0.45,0,0.1,20,0,15,2,200);
    print_odom_pos();

    loader.set(UNLOAD);

    high_goal_aligner.set(ALIGN);
    main_intake.spin(reverse,25,pct);


    drive_to_point_reverse_with_heading(-35,40,0,1.5,8,0.8,0,0.45,0,0.1,25,0,15,2,200);
    print_odom_pos();


    main_intake.spin(fwd,100,pct);
    intermediate_intake_stage.spin(reverse,100,pct);

    vex::task::sleep(3000);

    intermediate_intake_stage.stop(coast);

    float final_time = master_timer.time(seconds);
    printf("final time:%f\n",final_time);

} 

void auton_skills(){
    while(inertial_sensor.isCalibrating())
        vex::task::sleep(10);
    printf("inertial sensor calibrated\n");

    master_timer.clear();
    inertial_sensor.resetHeading();

    odom_setup(-2.8,7.95,2.75,2.0,1.0/1.0);

    thread odometry(odom_thread);
    vex::task::sleep(100);

    set_pose(0,0,90);

    loader.set(LOAD);
    drive_to_point(47,0, 6,6,0,0,0.45,0,0.1,40,0,50,3,200);
    print_odom_pos();

    main_intake.spin(fwd,100,pct);
    final_intake_stage.spin(fwd,100,pct);
    inertial_turn(45,180,3,0.45,0.1,0.1,4,150,200);

    drive_straight(45,30,180,16,2,3,8,0.8,0,0.45,0,0.1,200);
    print_odom_pos();

    vex::task::sleep(1500);


    drive_to_point_reverse_with_heading(52,0,0,10,8,0.8,0,0.45,0,0.1,20,0,15,2,200);
    print_odom_pos();

    drive_to_point_reverse_with_heading(63,12,0,10,8,0.8,0,0.45,0,0.1,25,0,15,2,200);
    print_odom_pos();

    loader.set(UNLOAD);
    descorer.set(DESCORE);
    main_intake.stop(coast);
    final_intake_stage.stop(coast);

    drive_to_point_reverse(63,80,10,8,0.8,0,0.45,0,0.1,35,0,15,2,200);
    print_odom_pos();

    drive_to_point_reverse_with_heading(50,95,0,10,8,0.8,0,0.45,0,0.1,20,0,15,2,200);
    print_odom_pos();

    inertial_turn(45,0,3,0.45,0.1,0.1,4,150,200);

    high_goal_aligner.set(ALIGN);
    drive_straight(30,30,0,-16,2,3,8,0.8,0,0.45,0,0.1,200);
    print_odom_pos();

    main_intake.spin(reverse,30,pct);
    vex::task::sleep(200);

    main_intake.spin(fwd,100,pct);
    final_intake_stage.spin(fwd,100,pct);
    intermediate_intake_stage.spin(reverse,100,pct);

    vex::task::sleep(4000);
    intermediate_intake_stage.stop(coast);

    high_goal_aligner.set(UNALIGN);
    loader.set(LOAD);
    drive_straight(50,30,0,35,2,3,8,0.8,0,0.45,0,0.1,200);
    print_odom_pos();

    vex::task::sleep(3000);

    
    drive_straight(30,30,0,-5,2,10,8,0.8,0,0.45,0,0.1,200);
    print_odom_pos();
    loader.set(UNLOAD);
    high_goal_aligner.set(ALIGN);

    drive_straight(30,30,0,-25,2,3,8,0.8,0,0.45,0,0.1,200);
    print_odom_pos();

    main_intake.spin(reverse,30,pct);
    vex::task::sleep(200);

    main_intake.spin(fwd,100,pct);
    intermediate_intake_stage.spin(reverse,100,pct);
    vex::task::sleep(4000);

    high_goal_aligner.set(UNALIGN);
    intermediate_intake_stage.stop(coast);

    drive_straight(30,30,0,10,2,10,8,0.8,0,0.45,0,0.1,200);
    print_odom_pos();

    turn_to_point_reverse(65,86,5,3,0.53,0.1,0.175,45,0,200);
    print_odom_pos();

    

    drive_to_point_reverse_with_heading(65,86,0,10,8,0.8,0,0.45,0,0.1,35,0,15,2,200);
    print_odom_pos();


    inertial_turn(45,0,3,0.45,0.1,0.1,4,150,200);

    drive_to_point_reverse_with_heading(65,6,0,10,8,0.8,0,0.45,0,0.1,35,0,15,2,200);
    print_odom_pos();


    turn_to_point(4,-9,5,3,0.53,0.1,0.175,45,0,200);
    print_odom_pos();

    drive_to_point_with_heading(4,-9,270,10,8,0.8,0,0.45,0,0.1,45,0,15,2,200);
    print_odom_pos();




    // tracking_wheel_cylinder.set(true);
    // main_intake.spin(fwd,100,pct);
    // final_intake_stage.spin(fwd,100,pct);

    // left_drive.spin(fwd,60,pct);
    // right_drive.spin(fwd,60,pct);

    

    // vex::task::sleep(1000);
    // left_drive.stop(coast);
    // right_drive.stop(coast);


    
    
    float final_time = master_timer.time(seconds);
    printf("final time:%f\n",final_time);
}