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
    inertial_turn(100,75,3,0.45,0,0.1,4,150,200);
    drive_straight(100,100,90  ,8,2,3,8,0.8,0,0.45,0,0.1,200);
    
    lower_intake.stop();
    high_intake.stop();
    inertial_turn(100,315,3,0.45,0,0.1,4,150,200);
    drive_straight(100,100,315,11,2,3,8,0.8,0,0.45,0,0.1,200);
    lower_intake.spin(reverse, 100, pct);
    high_intake.spin(reverse, 100, pct);

    float final_time = master_timer.time(seconds);
    printf("final time:%f\n",final_time);
}