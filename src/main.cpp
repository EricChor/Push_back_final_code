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

#include "odometry.h"

#include <iostream>

using namespace vex;

competition Competition;

void autonomousControl(){
    //test_routine();
    thread telemetry(odom_telemetry);
}

void userControl(){
    thread arcade(arcade_drive);

    Controller.ButtonR1.pressed(intake_pressed);
    Controller.ButtonR2.pressed(outtake_pressed);

    Controller.ButtonL1.pressed(load_pressed);
    Controller.ButtonL2.pressed(unload_pressed);

}



int main() {
    Competition.drivercontrol(userControl);
    Competition.autonomous(autonomousControl);

}
