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
    left_side_half_AWP();
}

void userControl(){
    thread arcade(arcade_drive);
    thread loader(loader_controller);
    thread descoring(descorer_controller);

    color_sorting_sensor.setLightPower(100,pct);
    color_sorting_sensor.setLight(ledState::on);

    Controller.ButtonA.pressed(aligner_controller);

    Controller.ButtonL1.pressed(score_high_pressed);
    Controller.ButtonL2.pressed(score_low_pressed);
    Controller.ButtonR1.pressed(intake_pressed);
    Controller.ButtonR2.pressed(outtake_pressed);
}



int main() {
    Competition.bStopTasksBetweenModes = true;
    Competition.drivercontrol(userControl);
    Competition.autonomous(autonomousControl);

}
