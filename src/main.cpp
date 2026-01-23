/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       cloud                                                     */
/*    Created:      12/23/2025, 3:47:52 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "driver_control.h"
#include "robot_config.h"
#include "routines.h"
#include "autonomous.h"
#include "odometry.h"

#include <iostream>

using namespace vex;

competition Competition;

void autonomousControl(){
    //thread telemetry(odom_telemetry);
    //test_routine();
    // routine_left();
    // routine_right();
    auton_skills();
}

void userControl(){

    while(inertial_sensor.isCalibrating())

        vex::task::sleep(10);
    thread arcade(arcade_drive);

    odom_setup(-2.8,7.95,2.75,2.0,1.0/1.0);

    thread odometry(odom_thread);
    thread telemetry(odom_telemetry);

    set_pose(0,0,0);

    // thread loader(loader_controller);
    tracking_wheel_cylinder=true;

    color_sorting_sensor.setLightPower(100,pct);
    color_sorting_sensor.setLight(ledState::on);

    Controller.ButtonB.pressed(aligner_controller);
    Controller.ButtonX.pressed(descorer_controller);

    Controller.ButtonL1.pressed(score_high_pressed);
    Controller.ButtonL2.pressed(score_low_pressed);
    Controller.ButtonR1.pressed(intake_pressed);
    Controller.ButtonR2.pressed(outtake_pressed);

    Controller.ButtonA.pressed(loader_controller);
}



int main() {
    Competition.bStopTasksBetweenModes = true;
    Competition.autonomous(autonomousControl);
     Competition.drivercontrol(userControl);

}
