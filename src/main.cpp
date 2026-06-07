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
right_low_auton();
    
}

void userControl(){

    
    thread arcade(arcade_drive);

    // thread loader(loader_controller);
    tracking_wheel_cylinder=true;

    color_sorting_sensor.setLightPower(100,pct);
    color_sorting_sensor.setLight(ledState::on);

    Controller.ButtonX.pressed(aligner_controller);
    Controller.ButtonB.pressed(descorer_controller);

    Controller.ButtonR1.pressed(intake_pressed);
    Controller.ButtonR2.pressed(outtake_pressed);

    Controller.ButtonA.pressed(loader_controller);
}



int main() {
    Competition.bStopTasksBetweenModes = true;
    Competition.autonomous(autonomousControl);
     Competition.drivercontrol(userControl);

}
