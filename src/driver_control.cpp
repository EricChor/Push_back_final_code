#include "vex.h"
#include "driver_control.h"
#include "robot_config.h"
#include "autonomous.h"

void robot_setup(){
    //drivebase
    left_drive.setMaxTorque(100,pct);
    right_drive.setMaxTorque(100,pct);
    left_drive.setStopping(coast);
    right_drive.setStopping(coast);
    loader.set(UNLOAD);
    descorer.set(UNDESCORE);
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

void intake_pressed(){
    if(current_intake_state != INTAKE){
        current_intake_state = INTAKE;
    } else {
        current_intake_state = STOP;
    }
    intake_controller();
}

void outtake_pressed(){
    if(current_intake_state != OUTTAKE){
        current_intake_state = OUTTAKE;
    } else {
        current_intake_state = STOP;
    }
    intake_controller();
}

void score_high_pressed(){
    if(current_intake_state != SCORE_HIGH){
        current_intake_state = SCORE_HIGH;
    }
    stop_color_sorting();
    main_intake.spin(fwd,100,pct);
    intermediate_intake_stage.spin(reverse,100,pct);
    final_intake_stage.spin(fwd,100,pct);
    while(Controller.ButtonL1.pressing()){
        vex::task::sleep(10);
    }
    main_intake.stop(coast);
    intermediate_intake_stage.stop(coast);
    final_intake_stage.stop(coast);
}

void score_low_pressed(){
    if(current_intake_state != SCORE_LOW){
        current_intake_state = SCORE_LOW;
    } else {
        current_intake_state = STOP;
    }
    intake_controller();
}


thread color_sorting_thread;
bool color_sorting_running = false;

void start_color_sorting() {
    if (!color_sorting_running) {
        color_sorting_running = true;
        color_sorting_thread = thread(color_sorting_intake);
    }
}

void stop_color_sorting() {
    if (color_sorting_running) {
        color_sorting_running = false;
        color_sorting_thread.join(); 
    }
}

void intake_controller(){
    switch (current_intake_state){
        case STOP:{
            stop_color_sorting();
            main_intake.stop(coast);
            intermediate_intake_stage.stop(coast);
            final_intake_stage.stop(coast);
            break;
        }

        case INTAKE:{
            main_intake.spin(fwd,100,pct);
            intermediate_intake_stage.stop(coast);
            final_intake_stage.spin(fwd,100,pct);
            // start_color_sorting();
            break;
        }

        case OUTTAKE:{
            stop_color_sorting();
            main_intake.spin(reverse,100,pct);
            intermediate_intake_stage.spin(fwd, 100, pct);
            final_intake_stage.spin(reverse,100,pct);
            break;
        }
        
        case SCORE_HIGH:{
            stop_color_sorting();
            main_intake.spin(fwd,100,pct);
            intermediate_intake_stage.spin(reverse,100,pct);
            final_intake_stage.spin(fwd,100,pct);
            while(Controller.ButtonL1.pressing()){
                vex::task::sleep(10);
            }
            main_intake.stop(coast);
            intermediate_intake_stage.stop(coast);
            final_intake_stage.stop(coast);
            break;
        }
        case SCORE_LOW:{
            stop_color_sorting();
            main_intake.spin(fwd,100,pct);
            intermediate_intake_stage.spin(reverse,100,pct);
            final_intake_stage.spin(reverse,100,pct);
            break;
            while(Controller.ButtonL2.pressing()){
                vex::task::sleep(10);
            }
            main_intake.stop(coast);
            intermediate_intake_stage.stop(coast);
            final_intake_stage.stop(coast);
        }
    }
}

bool loader_state = 0; //0 = off | 1 = on
void loader_pressed(){
    if(loader_state){
        loader_state = false;
        loader.set(UNLOAD);
    } else {
        loader_state = true;
        loader.set(LOAD);
        high_goal_aligner.set(UNALIGN);
    }
}

bool descore_state = 0; //0 = off | 1 = on
void descorer_pressed(){
    if(descore_state){
        descore_state = false;
        descorer.set(UNDESCORE);
    } else {
        descore_state = true;
        descorer.set(DESCORE);
    }
}


aligner_states current_aligner_state = UNALIGN;

void aligner_pressed(){
    if(current_aligner_state == ALIGN){
        current_aligner_state = UNALIGN;
        high_goal_aligner.set(UNALIGN);
    } else {
        current_aligner_state = ALIGN;
        high_goal_aligner.set(ALIGN);
        loader_state = UNLOAD;
        loader.set(UNLOAD);
    }
    
}

