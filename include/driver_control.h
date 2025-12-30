#define controller_deadzone 5
#pragma once

extern void robot_setup();

extern void arcade_drive();

extern void intake_controller();

extern void intake_pressed();

extern void outtake_pressed();

extern void score_high_pressed();

extern void score_low_pressed();

extern void loader_controller();



extern void descorer_controller();

extern void aligner_controller();

enum intake_state{
    STOP,
    INTAKE,
    OUTTAKE,
    SCORE_HIGH,
    SCORE_LOW
};

enum loader_states{
    LOAD = true,
    UNLOAD = false
};

enum aligner_states{
    ALIGN = true,
    UNALIGN = false
};

enum descorer_states{
    DESCORE = true,
    UNDESCORE = false
};

extern intake_state current_intake_state;

extern vex::thread color_sorting_thread;
extern bool color_sorting_running ;
extern void start_color_sorting();
extern void stop_color_sorting();