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
    Competition.bStopTasksBetweenModes = true;
    // left_side();
    // right_side();
    left_descore();
    right_descore();
}

void userControl(){
    Competition.bStopTasksBetweenModes = true;
    thread arcade(arcade_drive);
    thread loader(loader_pressed);
    thread descoring(descorer_pressed);

    color_sorting_sensor.setLightPower(100,pct);
    color_sorting_sensor.setLight(ledState::on);

    Controller.ButtonA.pressed(aligner_pressed);
    Controller.ButtonLeft.pressed(descorer_pressed);
    Controller.ButtonL1.pressed(score_high_pressed);
    Controller.ButtonL2.pressed(score_low_pressed);
    Controller.ButtonR1.pressed(intake_pressed);
    Controller.ButtonR2.pressed(outtake_pressed);
    Controller.ButtonRight.pressed(loader_pressed);
}



int main() {
    Competition.bStopTasksBetweenModes = true;
    Competition.drivercontrol(userControl);
    Competition.autonomous(autonomousControl);
    loader.set(UNLOAD);
    descorer.set(UNDESCORE);
}
