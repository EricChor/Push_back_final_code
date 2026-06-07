#include "vex.h"
#include "robot_config.h"
#include "trig.h"
#include "driver_control.h"
#define PI 3.14159

void inertial_turn(int max_speed, float target_heading, float max_time, float kP, float kI, float kD, float settle_error, int settle_time, float settle_speed){
    float end_time = max_time + master_timer.time(seconds);
    float current_heading = 0;

    float error = 0;
    float derivative_error = 0;
    float previous_error = 0;
    float integral_error = 0;

    double left_drive_velocity = 0;
    double right_drive_velocity = 0;

    double converted_settle_speed = settle_speed / 100;
    while(true){
        current_heading = inertial_sensor.heading();
        error = target_heading - current_heading;
        // printf("error:%f\n",error);
        if(fabs(error) > 180 ){
            if(error > 0){
                error = error - 360;
            } else {
                error = error + 360;
            }
        }

        derivative_error = error - previous_error;

        previous_error = error;

        if(fabs(error) <=4 ){
            integral_error = integral_error + error;
        } else {
            integral_error = 0;
        }
        left_drive_velocity = kP * error + kI * integral_error + kD * derivative_error;
        right_drive_velocity = kP * error + kI * integral_error + kD * derivative_error;

        if(fabs(left_drive_velocity) > max_speed){
            if(left_drive_velocity > 0){
                left_drive_velocity = max_speed;
            } else {
                left_drive_velocity = -max_speed;
            }
        }

        if(fabs(right_drive_velocity) > max_speed){
            if(right_drive_velocity > 0){
                right_drive_velocity =  max_speed;
            } else {
                right_drive_velocity = -max_speed;
            }
        }

        left_drive.spin(fwd,left_drive_velocity,pct);
        right_drive.spin(reverse,right_drive_velocity,pct);

        if((fabs(error) < settle_error) && (fabs(derivative_error) < converted_settle_speed)){
            left_drive.stop(brake);
            right_drive.stop(brake);
            vex::task::sleep(settle_time);

            error = target_heading - inertial_sensor.heading(degrees);

            if(fabs(error) > 180){
                if(error > 0){
                    error -= 360;
                } else {
                    error += 360;
                }
            }

            if(fabs(error) < settle_error){
                left_drive.stop(brake);
                right_drive.stop(brake);
                printf("error:%f\n",error);
                break;
            } else {
                printf("target not in range:%f\n",error);
            }

            if(master_timer.time(seconds) >= end_time){
                left_drive.stop(brake);
                right_drive.stop(brake);
                printf("turn timeout\n");
                break;
            }
    }
    vex::task::sleep(10);
}
}

void drive_straight(int max_linear_speed, int max_angular_speed, float target_heading, float target_distance,float acceptable_error, float max_time, float linear_kP, float linear_kI, float linear_kD, float angular_kP, float angular_kI, float angular_kD, float max_accel){
    float end_time = master_timer.time(seconds) + max_time;

    float current_heading = 0;

    float linear_error = 0;
    float linear_derivative_error = 0;
    float linear_previous_error = 0;
    float linear_integral_error = 0;

    float angular_error = 0;
    float angular_derivative_error = 0;
    float angular_previous_error = 0;
    float angular_integral_error = 0;

    float initial_position = left_front_motor.position(degrees);
    float current_position = initial_position;

    float left_drive_angular_velocity = 0;
    float right_drive_angular_velocity = 0;

    float left_drive_linear_velocity = 0;
    float right_drive_linear_velocity = 0;

    float prev_left_drive_linear_velocity = 0;
    float prev_right_drive_linear_velocity = 0;

    max_accel = max_accel / 100;
    while(true){
        current_heading = inertial_sensor.heading(degrees);
        angular_error = target_heading - current_heading;

        if(fabs(angular_error) > 180){
            if(angular_error > 0){
                angular_error -= 360;
            } else {
                angular_error += 360;
            }
        }

        angular_derivative_error = angular_error - angular_previous_error;

        angular_previous_error = angular_error;

        current_position = (left_front_motor.position(degrees)-initial_position) * 3.25 * M_PI /360 * 1.0/2.0;

        linear_error = target_distance - current_position;

        linear_derivative_error = linear_error - linear_previous_error;

        linear_previous_error = linear_error;

        if(fabs(angular_error) <=4 ){
            angular_integral_error += angular_error;
        } else {
            angular_integral_error = 0;
        }

        if(fabs(linear_error) <=2){
            linear_integral_error += linear_error;
        } else {
            linear_integral_error = 0;
        }

        left_drive_angular_velocity = angular_kP * angular_error + angular_kI * angular_integral_error + angular_kD * angular_derivative_error;
        right_drive_angular_velocity = -1*(angular_kP * angular_error + angular_kI * angular_integral_error + angular_kD * angular_derivative_error);
        if(fabs(left_drive_angular_velocity) > max_angular_speed){
            if(left_drive_angular_velocity > 0){
                left_drive_angular_velocity = max_angular_speed;
            } else {
                left_drive_angular_velocity = -max_angular_speed;
            }
        }

        if(fabs(right_drive_angular_velocity) > max_angular_speed){
            if(right_drive_angular_velocity > 0){
                right_drive_angular_velocity = max_angular_speed;
            } else {
                right_drive_angular_velocity = -max_angular_speed;
            }
        }

        left_drive_linear_velocity = linear_kP * linear_error + linear_kI * linear_integral_error + linear_kD * linear_derivative_error;
        right_drive_linear_velocity = linear_kP * linear_error + linear_kI * linear_integral_error + linear_kD * linear_derivative_error;

        if(((left_drive_linear_velocity) > (prev_left_drive_linear_velocity + max_accel))&&(max_accel != 0)){
            left_drive_linear_velocity = prev_left_drive_linear_velocity + max_accel;
            right_drive_linear_velocity = prev_right_drive_linear_velocity + max_accel;
        }
        
        if(fabs(left_drive_linear_velocity) > max_linear_speed){
            if(left_drive_linear_velocity > 0){
                left_drive_linear_velocity = max_linear_speed;
            } else {
                left_drive_linear_velocity = -max_linear_speed;
            }
        }

        if(fabs(right_drive_linear_velocity) > max_linear_speed){
            if(right_drive_linear_velocity > 0){
                right_drive_linear_velocity = max_linear_speed;
            } else {
                right_drive_linear_velocity = -max_linear_speed;
            }
        }

        left_drive.spin(fwd,left_drive_linear_velocity+left_drive_angular_velocity,pct);
        right_drive.spin(fwd,right_drive_linear_velocity+right_drive_angular_velocity,pct);

        if(fabs(linear_error) < acceptable_error){
            left_drive.setStopping(brake);
            right_drive.setStopping(brake);
            left_drive.stop();
            right_drive.stop();
            printf("distance from target:%f\n",linear_error);
            break;  
        }

        if(master_timer.time(seconds) >= end_time){
            left_drive.stop(brake);
            right_drive.stop(brake);
            printf("drive timeout\n");
            break;
        }

        prev_left_drive_linear_velocity = left_drive_linear_velocity;
        prev_right_drive_linear_velocity = right_drive_linear_velocity;
        vex::task::sleep(10);

    }

}

void color_sorting_intake(){
    double red_value = 0;
    double blue_value = 224;
    double blue_threshold = 50;
    double red_threshold = 50;
    double optical_value = 0;
    double red_hue_difference = 0;
    double blue_hue_difference = 0;
    lower_intake.spin(fwd,100,pct);

    alliance_color block_color = current_alliance_color;

    while (color_sorting_running){
        optical_value = color_sorting_sensor.hue();
        red_hue_difference = red_value - optical_value;

        if(fabs(red_hue_difference) > 180){
            if(red_hue_difference > 0){
                red_hue_difference -= 360;
            } else {
                red_hue_difference += 360;
            }
        }

        blue_hue_difference = blue_value - optical_value;
        if(fabs(blue_hue_difference) > 180){
            if(blue_hue_difference > 0){
                blue_hue_difference -= 360;
            } else {
                blue_hue_difference += 360;
            }
        }


        if(color_sorting_sensor.isNearObject()){
            if(fabs(red_hue_difference) < fabs(blue_hue_difference)){
                block_color = RED;
            } else {
                block_color = BLUE;
            }
        }
   
        if((current_alliance_color != block_color) && ((fabs(blue_hue_difference) < blue_threshold) || (fabs(red_hue_difference)) < red_threshold) && ((blue_threshold!=0) || (red_threshold!=0))){
            lower_intake.spin(fwd,100,pct);
            high_intake.spin(fwd,100,pct);
        } else {
            vex::task::sleep(100);
            lower_intake.spin(reverse,100,pct);
            high_intake.spin(reverse,100,pct);
            vex::task::sleep(250);
            high_intake.spin(fwd,100,pct);
            block_color = current_alliance_color;
        }

        vex::task::sleep(10);
    }
}