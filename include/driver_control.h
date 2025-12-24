#define controller_deadzone 5
#pragma once

extern void robot_setup();

extern void arcade_drive();

extern void intake_controller();

extern void intake_pressed();

extern void outtake_pressed();

extern void load_pressed();

extern void unload_pressed();

enum intake_state{
    STOP,
    INTAKE,
    OUTTAKE
};

enum loader_states{
    LOAD = true,
    UNLOAD = false
};