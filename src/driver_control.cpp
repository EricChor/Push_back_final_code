#include "vex.h"
#include "driver_control.h"
#include "robot_config.h"

void robot_setup(){
    //drivebase
    left_drive.setMaxTorque(100,pct);
    right_drive.setMaxTorque(100,pct);
    left_drive.setStopping(coast);
    right_drive.setStopping(coast);
}

void arcade_drive(){
    int linear_speed = 0;
    int turn_speed = 0;
    while(true){
        linear_speed = Controller.Axis3.position();
        turn_speed = Controller.Axis1.position();
        if((abs(linear_speed)+abs(turn_speed))>controller_deadzone){
            left_drive.spin(fwd,linear_speed + turn_speed, pct);
            right_drive.spin(fwd, linear_speed - turn_speed, pct);
        } else {
            left_drive.stop();
            right_drive.stop();
        }
        vex::task::sleep(10);
    }
}

intake_state current_intake_state = STOP;

void intake_controller(){
    if(current_intake_state == INTAKE){
        intake.spin(fwd,100,pct);
    } else if (current_intake_state == OUTTAKE){
        intake.spin(reverse,100,pct);
    } else {
        intake.stop(coast);
    }
}

void intake_pressed(){
    if(current_intake_state == INTAKE){
        current_intake_state = STOP;
    } else {
        current_intake_state = INTAKE;
    }
    intake_controller();
}

void outtake_pressed(){
    if(current_intake_state == OUTTAKE){
        current_intake_state = STOP;
    } else {
        current_intake_state = OUTTAKE;
    }
    intake_controller();
}



void load_pressed(){
    loader.set(LOAD);
}

void unload_pressed(){
    loader.set(UNLOAD);
}