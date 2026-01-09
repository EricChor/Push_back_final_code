#include "vex.h"
#include "odometry.h"
#include "odom_auton.h"
#include "robot_config.h"
#include "trig.h"

float global_min_speed = 0;

void turn_to_point(double target_x_pos, double target_y_pos, float acceptable_error, float max_time, float kP, float kI, float kD, float max_speed, int settle_delay, float settle_speed){
    double delta_x = 0;
    double delta_y = 0;
    double final_theta = 0;
    
    left_drive.setStopping(brake);
    right_drive.setStopping(brake);
    float current_heading = 0;
    float error = 0;
    float previous_error = 0;
    float integral_error = 0;
    float derivative_error = 0;
    float left_drive_speed = 0;
    float right_drive_speed = 0;
    float time_limit = max_time + master_timer.time(seconds);
    float current_time = master_timer.time(seconds);

    while(true){
        delta_x = target_x_pos - x_pos;       //center current position at origin
        delta_y = target_y_pos - y_pos;       //center current position at origin
        final_theta = atan2(delta_x,delta_y); //calculate angle, atan2 gives values between -π and π, use delta_x, delta_y to define positive y-axis as 0 and clockwise as positive (reflection about line y=x)
        if(final_theta < 0)                   //make all angles positive
            final_theta += (2 * M_PI);
        
        final_theta = radToDeg(final_theta);
        current_heading = inertial_sensor.heading(degrees); //update inertial sensor heading
        error = final_theta - current_heading;       //calculate angle error

        if(error > 180){                             //fix angle to smaller angle
            error -= 360;
        }else if (error <-180){
            error += 360;
        }

        if(fabs(error) <= 4){                        //calculate integral error, restriction is to reduce integral windup
            integral_error += error;
        }else{
            integral_error = 0;
        }

        if(fabs(integral_error) > 10){              //caps integral error to prevent integral windup
            if(integral_error > 0){
                integral_error = 10;
            } else {
                integral_error = -10;
            }
        }

        derivative_error = error - previous_error;   //calculate derivative error
        previous_error = error;                      //update previous error
        
        left_drive_speed = error * kP + integral_error * kI + derivative_error * kD;  //set left drive speed
        right_drive_speed = error * kP + integral_error * kI + derivative_error * kD; //set right drive speed

        //printf("error:%f\n",error);
        //printf("left:%f\n",left_drive_speed);
        if(fabs(left_drive_speed) > max_speed){      //cap turning velocity
            if(left_drive_speed > 0){
                left_drive_speed = max_speed;
                right_drive_speed = max_speed;
            }else{
                left_drive_speed = -1 * max_speed;
                right_drive_speed = -1 * max_speed;
            }
        }


        //printf("error:%f left:%f right:%f\n",error,left_drive_speed,right_drive_speed);
        left_drive.spin(fwd,left_drive_speed,pct);       //spin left drive
        right_drive.spin(reverse,right_drive_speed,pct); //spin right drive

        if(fabs(error) < acceptable_error && fabs(derivative_error) < settle_speed){ //settle controller if error and speed are low enough
            left_drive.stop();
            right_drive.stop();
            vex::task::sleep(settle_delay);
            error = final_theta - inertial_sensor.heading(degrees);

            if(fabs(error) < acceptable_error){
                break;
            }
        }

        current_time = master_timer.time(seconds);  //timeout if for if function never settles
        if(current_time > time_limit){
            left_drive.stop();
            right_drive.stop();
            break;
        }
    vex::task::sleep(10);
    }
    error = final_theta - current_heading;
    printf("turn to point finished angle from target: %f\n",error);

}

void turn_to_point_reverse(double target_x_pos, double target_y_pos, float acceptable_error, float max_time, float kP, float kI, float kD, float max_speed, int settle_delay, float settle_speed){
    double delta_x = 0;
    double delta_y = 0;
    double final_theta = 0;
    
    left_drive.setStopping(brake);
    right_drive.setStopping(brake);
    float current_heading = 0;
    float error = 0;
    float previous_error = 0;
    float integral_error = 0;
    float derivative_error = 0;
    float left_drive_speed = 0;
    float right_drive_speed = 0;
    float time_limit = max_time + master_timer.time(seconds);
    float current_time = master_timer.time(seconds);

    while(true){
        delta_x = target_x_pos - x_pos;       //center current position at origin
        delta_y = target_y_pos - y_pos;       //center current position at origin
        final_theta = atan2(delta_x,delta_y); //calculate angle, atan2 gives values between -π and π, use delta_x, delta_y to define positive y-axis as 0 and clockwise as positive (reflection about line y=x)
        if(final_theta < 0)                   //make all angles positive
            final_theta += (2 * M_PI);
        
        final_theta = radToDeg(final_theta);
        current_heading = fmod(inertial_sensor.heading(degrees) + 180,360); //update inertial sensor heading
        //printf("current heading:%f\n",current_heading);
        error = final_theta - current_heading;       //calculate angle error

        if(error > 180){                             //fix angle to smaller angle
            error -= 360;
        }else if (error <-180){
            error += 360;
        }

        if(fabs(error) <= 4){                        //calculate integral error, cap is to reduce integral windup
            integral_error += error;
        }else{
            integral_error = 0;
        }

        if(fabs(integral_error) > 10){
            if(integral_error > 0){
                integral_error = 10;
            } else {
                integral_error = -10;
            }
        }


        derivative_error = error - previous_error;   //calculate derivative error
        previous_error = error;                      //update previous error
        
        left_drive_speed = error * kP + integral_error * kI + derivative_error * kD;  //set left drive speed
        right_drive_speed = error * kP + integral_error * kI + derivative_error * kD; //set right drive speed

        //printf("heading error to point:%f\n",error);
        //printf("left:%f\n",left_drive_speed);
        if(fabs(left_drive_speed) > max_speed){      //cap turning velocity
            if(left_drive_speed > 0){
                left_drive_speed = max_speed;
                right_drive_speed = max_speed;
            }else{
                left_drive_speed = -1 * max_speed;
                right_drive_speed = -1 * max_speed;
            }
        }


        //printf("error:%f left:%f right:%f\n",error,left_drive_speed,right_drive_speed);
        left_drive.spin(fwd,left_drive_speed,pct);       //spin left drive
        right_drive.spin(reverse,right_drive_speed,pct); //spin right drive

        if(fabs(error) < acceptable_error && fabs(derivative_error) < settle_speed){ //settle controller if error and speed are low enough
            left_drive.stop();
            right_drive.stop();
            vex::task::sleep(settle_delay);
            error = final_theta - fmod(inertial_sensor.heading(degrees) + 180,360);

            if(fabs(error) < acceptable_error){
                break;
            }
        }

        current_time = master_timer.time(seconds);  //timeout if for if function never settles
        if(current_time > time_limit){
            left_drive.stop();
            right_drive.stop();
            break;
        }
    vex::task::sleep(10); 
    }

    error = final_theta - current_heading;
    printf("turn to point finished distance from target: %f\n",error);

}

void drive_to_point (double target_x_pos, double target_y_pos, float max_time, float linear_kP, float linear_kI, float linear_kD, float angular_kP, float angular_kI, float angular_kD, float max_linear_speed, float min_linear_speed, float max_angular_speed, float dead_zone_distance, float max_accel){
    left_drive.setStopping(brake);
    right_drive.setStopping(brake);
    float current_heading = inertial_sensor.heading(degrees);

    float distance_error = 0;
    float previous_distance_error = 0;
    float integral_distance_error = 0;
    float derivative_distance_error = 0;

    float angular_error;
    float previous_angular_error = 0;
    float integral_angular_error = 0;
    float derivative_angular_error = 0;

    float left_drive_angular_speed = 0;
    float right_drive_angular_speed = 0;

    float left_drive_linear_speed = 0;
    float right_drive_linear_speed = 0;

    float prev_left_drive_linear_speed = global_min_speed;
    float prev_right_drive_linear_speed = global_min_speed;

    global_min_speed = min_linear_speed;

    float left_drive_total_speed = 0;
    float right_drive_total_speed = 0;

    float time_limit = max_time + master_timer.time(seconds);
    float current_time = master_timer.time();


    float delta_x = target_x_pos - x_pos;       //center current position at origin
    float delta_y = target_y_pos - y_pos;       //center current position at origin
    double final_theta;

    float heading_scale_factor;

    if(delta_y == 0)
        delta_y = 0.00000001;
    
    float slope = -1 * (delta_x / delta_y); //slope of the perpendicular line which passes through the target point
    float y_intercept = target_y_pos -(slope * target_x_pos);

    bool initial_condition = false; //true = above line, false = below line
    if (y_pos > (slope*x_pos + y_intercept))
        initial_condition = true;
    else
        initial_condition = false;

    max_accel = max_accel / 100; // convert max_accel from %/1sec to %/10msec

    while(true){
        delta_x = target_x_pos - x_pos;       //center current position at origin
        delta_y = target_y_pos - y_pos;       //center current position at origin
        //printf("x:%f y:%f\n",delta_x,delta_y);
        final_theta = atan2(delta_x,delta_y); //calculate angle, atan2 gives values between -π and π, use delta_x, delta_y to define positive y-axis as 0 and clockwise as positive (reflection about line y=x)
        //printf("final_theta:%f\n",final_theta);
        if(final_theta < 0)                   //make all angles positive
            final_theta += (2 * M_PI);

        final_theta = radToDeg(final_theta);
        current_heading = inertial_sensor.heading(degrees);
        angular_error = final_theta - current_heading;       //calculate angle error
        //printf("F:%f C:%f\n",final_theta,current_heading);

        //printf("angular_error:%f\n",angular_error);
        if(angular_error > 180){                             //fix angle to smaller angle
            angular_error -= 360;
        }else if (angular_error <-180){
            angular_error += 360;
        }

        if(angular_error < -90){            //restrict angular error t0 -90 to 90 for cosine function
            angular_error = -90;
        }

        if(angular_error > 90){
            angular_error = 90;
        }

        if(fabs(angular_error) <= 4){                        //calculate integral error, cap is to reduce integral windup
            integral_angular_error += angular_error;
        }else{
            integral_angular_error = 0;
        }

        //printf("angular_error:%f\n",angular_error);

        derivative_angular_error = angular_error - previous_angular_error;   //calculate derivative error
        previous_angular_error = angular_error;                      //update previous error
        
        left_drive_angular_speed = angular_error * angular_kP + integral_angular_error * angular_kI + derivative_angular_error * angular_kD;  //set left drive speed
        right_drive_angular_speed = angular_error * angular_kP + integral_angular_error * angular_kI + derivative_angular_error * angular_kD; //set right drive speed

        if(fabs(left_drive_angular_speed) > max_angular_speed){
            if(left_drive_angular_speed < 0){
                left_drive_angular_speed  = -1 * max_angular_speed;
                right_drive_angular_speed = -1 * max_angular_speed; 
            } else {
                left_drive_angular_speed  = max_angular_speed;
                right_drive_angular_speed = max_angular_speed;
            }
        }

        /////////////////////////////////
        distance_error = sqrt(pow((target_x_pos-x_pos),2)+pow((target_y_pos-y_pos),2));
        derivative_distance_error = distance_error - previous_distance_error;
        previous_distance_error = distance_error;

        if(fabs(distance_error) <= 2){
            integral_distance_error += distance_error;
        }else{
            integral_distance_error = 0;
        }

        if(fabs(integral_distance_error) > 5){
            if(integral_distance_error > 0){
                integral_distance_error = 5;
            } else {
                integral_distance_error = -5;
            }
        }



        left_drive_linear_speed = distance_error * linear_kP + integral_distance_error * linear_kI + derivative_distance_error * linear_kD;
        right_drive_linear_speed = distance_error * linear_kP + integral_distance_error * linear_kI + derivative_distance_error * linear_kD;


        if(fabs(left_drive_linear_speed) > max_linear_speed){
            if(left_drive_linear_speed < 0 ){
                left_drive_linear_speed  = -1 * max_linear_speed;
                right_drive_linear_speed = -1 * max_linear_speed;
            } else {
                left_drive_linear_speed = max_linear_speed;
                right_drive_linear_speed = max_linear_speed;
            }
        }

        if(distance_error < dead_zone_distance){
            left_drive_angular_speed = 0;
            right_drive_angular_speed = 0;
            angular_error = 0;
            angular_error = 0;
        }

        heading_scale_factor = cos(degToRad(angular_error));
        left_drive_linear_speed = left_drive_linear_speed * heading_scale_factor;   //scaling linear speeds by cos of angular speeds (drive train slows at large angular errors)
        right_drive_linear_speed = right_drive_linear_speed * heading_scale_factor;

        if(fabs(left_drive_linear_speed) < min_linear_speed){
            if(left_drive_linear_speed < 0){
                left_drive_linear_speed = -1 * min_linear_speed;
                right_drive_linear_speed = -1 * min_linear_speed;
            } else {
                left_drive_linear_speed = min_linear_speed;
                right_drive_linear_speed = min_linear_speed;
            }
        }

        if((left_drive_linear_speed > (prev_left_drive_linear_speed+max_accel))&&(max_accel != 0)){
            left_drive_linear_speed  = prev_left_drive_linear_speed  + max_accel;
            right_drive_linear_speed = prev_right_drive_linear_speed + max_accel;
        }
        //printf("left:%f right:%f\n",left_drive_total_speed,right_drive_total_speed);
        left_drive_total_speed  = left_drive_linear_speed + left_drive_angular_speed;
        right_drive_total_speed = right_drive_linear_speed - right_drive_angular_speed;

        //printf("left:%f right:%f error:%f i:%f\n",left_drive_linear_speed,right_drive_linear_speed,distance_error,integral_distance_error);
        left_drive.spin(fwd,left_drive_total_speed,pct);
        right_drive.spin(fwd,right_drive_total_speed,pct);

        if(initial_condition == true){
            if(y_pos < (slope*x_pos + y_intercept)){
                if(min_linear_speed == 0){
                    left_drive.stop();
                    right_drive.stop();
                }
                //printf("crossed line\n");
                break;
            }
        }

        if(initial_condition == false){
            if(y_pos > (slope*x_pos + y_intercept)){
                if(min_linear_speed == 0){
                    left_drive.stop();
                    right_drive.stop();
                }
                //printf("crossed line\n");
                break;
            }
        }

        current_time = master_timer.time(seconds);
        if(current_time > time_limit){
            if(min_linear_speed == 0){
                left_drive.stop();
                right_drive.stop();
            }
            printf("timeout\n");
            break;
        }
        prev_left_drive_linear_speed = left_drive_linear_speed;
        prev_right_drive_linear_speed = right_drive_linear_speed;
        vex::task::sleep(10);
    }
    distance_error = sqrt(pow((target_x_pos-x_pos),2)+pow((target_y_pos-y_pos),2));
    printf("drive to point finished distance from target: %f\n",distance_error);
}

void drive_to_point_with_heading (double target_x_pos, double target_y_pos, float target_heading, float max_time, float linear_kP, float linear_kI, float linear_kD, float angular_kP, float angular_kI, float angular_kD, float max_linear_speed, float min_linear_speed, float max_angular_speed, float dead_zone_distance, float max_accel){
    left_drive.setStopping(brake);
    right_drive.setStopping(brake);
    float current_heading = inertial_sensor.heading(degrees);

    static float distance_error = sqrt(pow((target_x_pos-x_pos),2)+pow((target_y_pos-y_pos),2));
    float previous_distance_error = 0;
    float integral_distance_error = 0;
    float derivative_distance_error = 0;

    float angular_error;
    float previous_angular_error = 0;
    float integral_angular_error = 0;
    float derivative_angular_error = 0;

    float left_drive_angular_speed = 0;
    float right_drive_angular_speed = 0;

    float left_drive_linear_speed = 0;
    float right_drive_linear_speed = 0;

    float prev_left_drive_linear_speed = global_min_speed;
    float prev_right_drive_linear_speed = global_min_speed;

    global_min_speed = min_linear_speed;

    float left_drive_total_speed = 0;
    float right_drive_total_speed = 0;

    float time_limit = max_time + master_timer.time(seconds);
    float current_time = master_timer.time();


    float delta_x = target_x_pos - x_pos;       //center current position at origin
    float delta_y = target_y_pos - y_pos;       //center current position at origin
    double final_theta;

    float heading_scale_factor;

    if(delta_y == 0)
        delta_y = 0.00000001;
    
    float slope = -1 * (delta_x / delta_y); //slope of the perpendicular line which passes through the target point
    float y_intercept = target_y_pos -(slope * target_x_pos);

    bool initial_condition = false; //true = above line, false = below line
    if (y_pos > (slope*x_pos + y_intercept))
        initial_condition = true;
    else
        initial_condition = false;

    max_accel = max_accel / 100; // convert max_accel from %/1sec to %/10msec

    while(true){
        delta_x = target_x_pos - x_pos;       //center current position at origin
        delta_y = target_y_pos - y_pos;       //center current position at origin
        //printf("x:%f y:%f\n",delta_x,delta_y);
        final_theta = atan2(delta_x,delta_y); //calculate angle, atan2 gives values between -π and π, use delta_x, delta_y to define positive y-axis as 0 and clockwise as positive (reflection about line y=x)
        //printf("final_theta:%f\n",final_theta);
        if(final_theta < 0)                   //make all angles positive
            final_theta += (2 * M_PI);

        final_theta = radToDeg(final_theta);
        current_heading = inertial_sensor.heading(degrees);
        
        // printf("distance :%f deadzone:%f \n",distance_error,dead_zone_distance);
        if(distance_error > dead_zone_distance){
            angular_error = final_theta - current_heading;       //calculate angle error
            // printf("finding heading\n");
        } else {
            // printf("heading lock\n");
            angular_error = target_heading - current_heading;
        }
        //printf("F:%f C:%f\n",final_theta,current_heading);

        //printf("angular_error:%f\n",angular_error);
        if(angular_error > 180){                             //fix angle to smaller angle
            angular_error -= 360;
        }else if (angular_error <-180){
            angular_error += 360;
        }

        if(angular_error < -90){            //restrict angular error t0 -90 to 90 for cosine function
            angular_error = -90;
        }

        if(angular_error > 90){
            angular_error = 90;
        }

        if(fabs(angular_error) <= 4){                        //calculate integral error, cap is to reduce integral windup
            integral_angular_error += angular_error;
        }else{
            integral_angular_error = 0;
        }

        //printf("angular_error:%f\n",angular_error);

        derivative_angular_error = angular_error - previous_angular_error;   //calculate derivative error
        previous_angular_error = angular_error;                      //update previous error
        
        left_drive_angular_speed = angular_error * angular_kP + integral_angular_error * angular_kI + derivative_angular_error * angular_kD;  //set left drive speed
        right_drive_angular_speed = angular_error * angular_kP + integral_angular_error * angular_kI + derivative_angular_error * angular_kD; //set right drive speed

        if(fabs(left_drive_angular_speed) > max_angular_speed){
            if(left_drive_angular_speed < 0){
                left_drive_angular_speed  = -1 * max_angular_speed;
                right_drive_angular_speed = -1 * max_angular_speed; 
            } else {
                left_drive_angular_speed  = max_angular_speed;
                right_drive_angular_speed = max_angular_speed;
            }
        }

        /////////////////////////////////
        distance_error = sqrt(pow((target_x_pos-x_pos),2)+pow((target_y_pos-y_pos),2));
        derivative_distance_error = distance_error - previous_distance_error;
        previous_distance_error = distance_error;

        if(fabs(distance_error) <= 2){
            integral_distance_error += distance_error;
        }else{
            integral_distance_error = 0;
        }

        if(fabs(integral_distance_error) > 5){
            if(integral_distance_error > 0){
                integral_distance_error = 5;
            } else {
                integral_distance_error = -5;
            }
        }



        left_drive_linear_speed = distance_error * linear_kP + integral_distance_error * linear_kI + derivative_distance_error * linear_kD;
        right_drive_linear_speed = distance_error * linear_kP + integral_distance_error * linear_kI + derivative_distance_error * linear_kD;


        if(fabs(left_drive_linear_speed) > max_linear_speed){
            if(left_drive_linear_speed < 0 ){
                left_drive_linear_speed  = -1 * max_linear_speed;
                right_drive_linear_speed = -1 * max_linear_speed;
            } else {
                left_drive_linear_speed = max_linear_speed;
                right_drive_linear_speed = max_linear_speed;
            }
        }


        heading_scale_factor = cos(degToRad(angular_error));
        left_drive_linear_speed = left_drive_linear_speed * heading_scale_factor;   //scaling linear speeds by cos of angular speeds (drive train slows at large angular errors)
        right_drive_linear_speed = right_drive_linear_speed * heading_scale_factor;

        if(fabs(left_drive_linear_speed) < min_linear_speed){
            if(left_drive_linear_speed < 0){
                left_drive_linear_speed = -1 * min_linear_speed;
                right_drive_linear_speed = -1 * min_linear_speed;
            } else {
                left_drive_linear_speed = min_linear_speed;
                right_drive_linear_speed = min_linear_speed;
            }
        }

        if((left_drive_linear_speed > (prev_left_drive_linear_speed+max_accel))&&(max_accel != 0)){
            left_drive_linear_speed  = prev_left_drive_linear_speed  + max_accel;
            right_drive_linear_speed = prev_right_drive_linear_speed + max_accel;
        }
        //printf("left:%f right:%f\n",left_drive_total_speed,right_drive_total_speed);
        left_drive_total_speed  = left_drive_linear_speed + left_drive_angular_speed;
        right_drive_total_speed = right_drive_linear_speed - right_drive_angular_speed;

        //printf("left:%f right:%f error:%f i:%f\n",left_drive_linear_speed,right_drive_linear_speed,distance_error,integral_distance_error);
        left_drive.spin(fwd,left_drive_total_speed,pct);
        right_drive.spin(fwd,right_drive_total_speed,pct);

        if(initial_condition == true){
            if(y_pos < (slope*x_pos + y_intercept)){
                if(min_linear_speed == 0){
                    left_drive.stop();
                    right_drive.stop();
                }
                //printf("crossed line\n");
                break;
            }
        }

        if(initial_condition == false){
            if(y_pos > (slope*x_pos + y_intercept)){
                if(min_linear_speed == 0){
                    left_drive.stop();
                    right_drive.stop();
                }
                //printf("crossed line\n");
                break;
            }
        }

        current_time = master_timer.time(seconds);
        if(current_time > time_limit){
            if(min_linear_speed == 0){
                left_drive.stop();
                right_drive.stop();
            }
            printf("timeout\n");
            break;
        }
        prev_left_drive_linear_speed = left_drive_linear_speed;
        prev_right_drive_linear_speed = right_drive_linear_speed;
        vex::task::sleep(10);
    }
    distance_error = sqrt(pow((target_x_pos-x_pos),2)+pow((target_y_pos-y_pos),2));
    printf("drive to point finished distance from target: %f\n",distance_error);
}


void drive_to_point_reverse (double target_x_pos, double target_y_pos, float max_time, float linear_kP, float linear_kI, float linear_kD, float angular_kP, float angular_kI, float angular_kD, float max_linear_speed, float min_linear_speed, float max_angular_speed, float dead_zone_distance, float max_accel){
    left_drive.setStopping(brake);
    right_drive.setStopping(brake);
    float current_heading = 0;
    float distance_error = 0;
    float previous_distance_error = 0;
    float integral_distance_error = 0;
    float derivative_distance_error = 0;

    float angular_error;
    float previous_angular_error = 0;
    float integral_angular_error = 0;
    float derivative_angular_error = 0;

    float left_drive_angular_speed = 0;
    float right_drive_angular_speed = 0;

    float left_drive_linear_speed = 0;
    float right_drive_linear_speed = 0;

    float prev_left_drive_linear_speed = global_min_speed;
    float prev_right_drive_linear_speed = global_min_speed;

    global_min_speed = min_linear_speed;

    float left_drive_total_speed = 0;
    float right_drive_total_speed = 0;

    float time_limit = max_time + master_timer.time(seconds);
    float current_time = master_timer.time();


    float delta_x = target_x_pos - x_pos;       //center current position at origin
    float delta_y = target_y_pos - y_pos;       //center current position at origin
    double final_theta;

    float heading_scale_factor;

    if(delta_y == 0)
        delta_y = 0.00000001; 
    float slope = -1 * (delta_x / delta_y); //slope of the perpendicular line which passes through the target point
    float y_intercept = target_y_pos -(slope * target_x_pos);

    bool initial_condition = false; //true = above line, false = below line
    if (y_pos > (slope*x_pos + y_intercept))
        initial_condition = true;
    else
        initial_condition = false;

    max_accel = max_accel / 100;

    while(true){
        delta_x = target_x_pos - x_pos;       //center current position at origin
        delta_y = target_y_pos - y_pos;       //center current position at origin
        final_theta = atan2(delta_x,delta_y); //calculate angle, atan2 gives values between -π and π, use delta_x, delta_y to define positive y-axis as 0 and clockwise as positive (reflection about line y=x)
        if(final_theta < 0)                   //make all angles positive
            final_theta += (2 * M_PI);

        final_theta = radToDeg(final_theta);
        current_heading = inertial_sensor.heading(degrees);
        angular_error = final_theta - current_heading + 180;       //calculate angle error

        //printf("angular_error:%f\n",angular_error);
        if(angular_error > 180){                             //fix angle to smaller angle
            angular_error -= 360;
        }else if (angular_error <-180){
            angular_error += 360;
        }

        if(angular_error < -90){            //restrict angular error t0 -90 to 90 for cosine function
            angular_error = -90;
        }

        if(angular_error > 90){
            angular_error = 90;
        }

        if(fabs(angular_error) <= 4){                        //calculate integral error, cap is to reduce integral windup
            integral_angular_error += angular_error;
        }else{
            integral_angular_error = 0;
        }

        //printf("angular_error:%f\n",angular_error);

        derivative_angular_error = angular_error - previous_angular_error;   //calculate derivative error
        previous_angular_error = angular_error;                      //update previous error
        
        left_drive_angular_speed = angular_error * angular_kP + integral_angular_error * angular_kI + derivative_angular_error * angular_kD;  //set left drive speed
        right_drive_angular_speed = angular_error * angular_kP + integral_angular_error * angular_kI + derivative_angular_error * angular_kD; //set right drive speed

        if(fabs(left_drive_angular_speed) > max_angular_speed){
            if(left_drive_angular_speed < 0){
                left_drive_angular_speed  = -1 * max_angular_speed;
                right_drive_angular_speed = -1 * max_angular_speed; 
            } else {
                left_drive_angular_speed  = max_angular_speed;
                right_drive_angular_speed = max_angular_speed;
            }
        }

        /////////////////////////////////
        distance_error = sqrt(pow((target_x_pos-x_pos),2)+pow((target_y_pos-y_pos),2));
        //printf("x:%fy:%fx:%fy:%f  | distance_error:%f\n",target_x_pos,target_y_pos,x_pos,y_pos,distance_error);
        derivative_distance_error = distance_error - previous_distance_error;
        previous_distance_error = distance_error;

        if(fabs(distance_error) <= 2){
            integral_distance_error += distance_error;
        }else{
            integral_distance_error = 0;
        }

        if(fabs(integral_distance_error) > 5){
            if(integral_distance_error > 0){
                integral_distance_error = 5;
            } else {
                integral_distance_error = -5;
            }
        }

        left_drive_linear_speed = distance_error * linear_kP + integral_distance_error * linear_kI + derivative_distance_error * linear_kD;
        right_drive_linear_speed = distance_error * linear_kP + integral_distance_error * linear_kI + derivative_distance_error * linear_kD;

        
        if(fabs(left_drive_linear_speed) > max_linear_speed){
            if(left_drive_linear_speed < 0 ){
                left_drive_linear_speed  = -1 * max_linear_speed;
                right_drive_linear_speed = -1 * max_linear_speed;
            } else {
                left_drive_linear_speed = max_linear_speed;
                right_drive_linear_speed = max_linear_speed;
            }
        }

        if(distance_error < dead_zone_distance){
            left_drive_angular_speed = 0;
            right_drive_angular_speed = 0;
            angular_error = 0;
            angular_error = 0;
        }

        heading_scale_factor = cos(degToRad(angular_error));
        left_drive_linear_speed = left_drive_linear_speed * heading_scale_factor;   //scaling linear speeds by cos of angular speeds (drive train slows at large angular errors)
        right_drive_linear_speed = right_drive_linear_speed * heading_scale_factor;

        if(fabs(left_drive_linear_speed) < min_linear_speed){
            if(left_drive_linear_speed < 0){
                left_drive_linear_speed = -1 * min_linear_speed;
                right_drive_linear_speed = -1 * min_linear_speed;
            } else {
                left_drive_linear_speed = min_linear_speed;
                right_drive_linear_speed = min_linear_speed;
            }
        }


        if((left_drive_linear_speed > (prev_left_drive_linear_speed+max_accel))&&(max_accel != 0)){
            left_drive_linear_speed  = prev_left_drive_linear_speed  + max_accel;
            right_drive_linear_speed = prev_right_drive_linear_speed + max_accel;
            
        }

        //to go from positive to negative, add 180 degrees to the heading, change from forward to reverse, and change the signs of left and right drive
        left_drive_total_speed  = left_drive_linear_speed - left_drive_angular_speed;
        right_drive_total_speed = right_drive_linear_speed + right_drive_angular_speed;

        //printf("left:%f right:%f error:%f i:%f\n",left_drive_linear_speed,right_drive_linear_speed,distance_error,integral_distance_error);
        left_drive.spin(reverse,left_drive_total_speed,pct);
        right_drive.spin(reverse,right_drive_total_speed,pct);

        if(initial_condition == true){
            if(y_pos < (slope*x_pos + y_intercept)){
                if(min_linear_speed == 0){
                    left_drive.stop();
                    right_drive.stop();
                }
                break;
            }
        }

        if(initial_condition == false){
            if(y_pos > (slope*x_pos + y_intercept)){
                if(min_linear_speed == 0){
                    left_drive.stop();
                    right_drive.stop();
                }
                break;
            }
        }

        current_time = master_timer.time(seconds);
        if(current_time > time_limit){
            if(min_linear_speed == 0){
                left_drive.stop();
                right_drive.stop();
            }
            break;
        }
        prev_left_drive_linear_speed = left_drive_linear_speed;
        prev_right_drive_linear_speed = right_drive_linear_speed;
        vex::task::sleep(10);
    }
    distance_error = sqrt(pow((target_x_pos-x_pos),2)+pow((target_y_pos-y_pos),2));
    printf("drive to point reverse finished distance from target: %f\n",distance_error);
}

void drive_to_point_reverse_with_heading (double target_x_pos, double target_y_pos, float target_heading, float max_time, float linear_kP, float linear_kI, float linear_kD, float angular_kP, float angular_kI, float angular_kD, float max_linear_speed, float min_linear_speed, float max_angular_speed, float dead_zone_distance, float max_accel){
    left_drive.setStopping(brake);
    right_drive.setStopping(brake);
    float current_heading = 0;
    float distance_error = 0;
    float previous_distance_error = 0;
    float integral_distance_error = 0;
    float derivative_distance_error = 0;

    float angular_error;
    float previous_angular_error = 0;
    float integral_angular_error = 0;
    float derivative_angular_error = 0;

    float left_drive_angular_speed = 0;
    float right_drive_angular_speed = 0;

    float left_drive_linear_speed = 0;
    float right_drive_linear_speed = 0;

    float prev_left_drive_linear_speed = global_min_speed;
    float prev_right_drive_linear_speed = global_min_speed;

    global_min_speed = min_linear_speed;

    float left_drive_total_speed = 0;
    float right_drive_total_speed = 0;

    float time_limit = max_time + master_timer.time(seconds);
    float current_time = master_timer.time();


    float delta_x = target_x_pos - x_pos;       //center current position at origin
    float delta_y = target_y_pos - y_pos;       //center current position at origin
    double final_theta;

    float heading_scale_factor;

    if(delta_y == 0)
        delta_y = 0.00000001; 
    float slope = -1 * (delta_x / delta_y); //slope of the perpendicular line which passes through the target point
    float y_intercept = target_y_pos -(slope * target_x_pos);

    bool initial_condition = false; //true = above line, false = below line
    if (y_pos > (slope*x_pos + y_intercept))
        initial_condition = true;
    else
        initial_condition = false;

    max_accel = max_accel / 100;

    while(true){
        delta_x = target_x_pos - x_pos;       //center current position at origin
        delta_y = target_y_pos - y_pos;       //center current position at origin
        final_theta = atan2(delta_x,delta_y); //calculate angle, atan2 gives values between -π and π, use delta_x, delta_y to define positive y-axis as 0 and clockwise as positive (reflection about line y=x)
        if(final_theta < 0)                   //make all angles positive
            final_theta += (2 * M_PI);

        final_theta = radToDeg(final_theta);
        current_heading = inertial_sensor.heading(degrees);
        
        if(distance_error > dead_zone_distance){
            angular_error = final_theta - current_heading +180;       //calculate angle error
            // printf("finding heading\n");
        } else {
            // printf("heading lock\n");
            angular_error = target_heading - current_heading+180;
        }
        
        
     
        
        //printf("angular_error:%f\n",angular_error);
        if(angular_error > 180){                             //fix angle to smaller angle
            angular_error -= 360;
        }else if (angular_error <-180){
            angular_error += 360;
        }

        if(angular_error < -90){            //restrict angular error t0 -90 to 90 for cosine function
            angular_error = -90;
        }

        if(angular_error > 90){
            angular_error = 90;
        }
        

        if(fabs(angular_error) <= 4){                        //calculate integral error, cap is to reduce integral windup
            integral_angular_error += angular_error;
        }else{
            integral_angular_error = 0;
        }

        //printf("angular_error:%f\n",angular_error);

        derivative_angular_error = angular_error - previous_angular_error;   //calculate derivative error
        previous_angular_error = angular_error;                      //update previous error
        
        left_drive_angular_speed = angular_error * angular_kP + integral_angular_error * angular_kI + derivative_angular_error * angular_kD;  //set left drive speed
        right_drive_angular_speed = angular_error * angular_kP + integral_angular_error * angular_kI + derivative_angular_error * angular_kD; //set right drive speed

        if(fabs(left_drive_angular_speed) > max_angular_speed){
            if(left_drive_angular_speed < 0){
                left_drive_angular_speed  = -1 * max_angular_speed;
                right_drive_angular_speed = -1 * max_angular_speed; 
            } else {
                left_drive_angular_speed  = max_angular_speed;
                right_drive_angular_speed = max_angular_speed;
            }
        }

        /////////////////////////////////
        distance_error = sqrt(pow((target_x_pos-x_pos),2)+pow((target_y_pos-y_pos),2));
        //printf("x:%fy:%fx:%fy:%f  | distance_error:%f\n",target_x_pos,target_y_pos,x_pos,y_pos,distance_error);
        derivative_distance_error = distance_error - previous_distance_error;
        previous_distance_error = distance_error;

        if(fabs(distance_error) <= 2){
            integral_distance_error += distance_error;
        }else{
            integral_distance_error = 0;
        }

        if(fabs(integral_distance_error) > 5){
            if(integral_distance_error > 0){
                integral_distance_error = 5;
            } else {
                integral_distance_error = -5;
            }
        }

        left_drive_linear_speed = distance_error * linear_kP + integral_distance_error * linear_kI + derivative_distance_error * linear_kD;
        right_drive_linear_speed = distance_error * linear_kP + integral_distance_error * linear_kI + derivative_distance_error * linear_kD;

        
        if(fabs(left_drive_linear_speed) > max_linear_speed){
            if(left_drive_linear_speed < 0 ){
                left_drive_linear_speed  = -1 * max_linear_speed;
                right_drive_linear_speed = -1 * max_linear_speed;
            } else {
                left_drive_linear_speed = max_linear_speed;
                right_drive_linear_speed = max_linear_speed;
            }
        }

        // if(distance_error < dead_zone_distance){
        //     left_drive_angular_speed = 0;
        //     right_drive_angular_speed = 0;
        //     angular_error = 0;
        //     angular_error = 0;
        // }

        heading_scale_factor = cos(degToRad(angular_error));
        left_drive_linear_speed = left_drive_linear_speed * heading_scale_factor;   //scaling linear speeds by cos of angular speeds (drive train slows at large angular errors)
        right_drive_linear_speed = right_drive_linear_speed * heading_scale_factor;

        if(fabs(left_drive_linear_speed) < min_linear_speed){
            if(left_drive_linear_speed < 0){
                left_drive_linear_speed = -1 * min_linear_speed;
                right_drive_linear_speed = -1 * min_linear_speed;
            } else {
                left_drive_linear_speed = min_linear_speed;
                right_drive_linear_speed = min_linear_speed;
            }
        }


        if((left_drive_linear_speed > (prev_left_drive_linear_speed+max_accel))&&(max_accel != 0)){
            left_drive_linear_speed  = prev_left_drive_linear_speed  + max_accel;
            right_drive_linear_speed = prev_right_drive_linear_speed + max_accel;
            
        }

        //to go from positive to negative, add 180 degrees to the heading, change from forward to reverse, and change the signs of left and right drive
        left_drive_total_speed  = left_drive_linear_speed - left_drive_angular_speed;
        right_drive_total_speed = right_drive_linear_speed + right_drive_angular_speed;

        //printf("left:%f right:%f error:%f i:%f\n",left_drive_linear_speed,right_drive_linear_speed,distance_error,integral_distance_error);
        left_drive.spin(reverse,left_drive_total_speed,pct);
        right_drive.spin(reverse,right_drive_total_speed,pct);

        if(initial_condition == true){
            if(y_pos < (slope*x_pos + y_intercept)){
                if(min_linear_speed == 0){
                    left_drive.stop();
                    right_drive.stop();
                }
                break;
            }
        }

        if(initial_condition == false){
            if(y_pos > (slope*x_pos + y_intercept)){
                if(min_linear_speed == 0){
                    left_drive.stop();
                    right_drive.stop();
                }
                break;
            }
        }

        current_time = master_timer.time(seconds);
        if(current_time > time_limit){
            if(min_linear_speed == 0){
                left_drive.stop();
                right_drive.stop();
            }
            break;
        }
        prev_left_drive_linear_speed = left_drive_linear_speed;
        prev_right_drive_linear_speed = right_drive_linear_speed;
        vex::task::sleep(10);
    }
    distance_error = sqrt(pow((target_x_pos-x_pos),2)+pow((target_y_pos-y_pos),2));
    printf("drive to point reverse finished distance from target: %f\n",distance_error);
}

void move_to_pose(double target_x_pos, double target_y_pos, float target_angle, float lead_distance, float setback_distance, float max_time, float linear_kP, float linear_kI, float linear_kD, float angular_kP, float angular_kI, float angular_kD, float max_linear_speed, float min_linear_speed, float max_angular_speed, float dead_zone_distance, float carrot_distance, float max_accel){
    left_drive.setStopping(brake);
    right_drive.setStopping(brake);
    float current_heading = inertial_sensor.heading(degrees);

    float initial_distance_error = sqrt(pow((target_x_pos-x_pos),2)+pow((target_y_pos-y_pos),2));
    float distance_error = 0;
    float previous_distance_error = 0;
    float integral_distance_error = 0;
    float derivative_distance_error = 0;

    float distance_from_carrot = 0;

    float angular_error;
    float previous_angular_error = 0;
    float integral_angular_error = 0;
    float derivative_angular_error = 0;

    float left_drive_angular_speed = 0;
    float right_drive_angular_speed = 0;

    float left_drive_linear_speed = 0;
    float right_drive_linear_speed = 0;

    float prev_left_drive_linear_speed = global_min_speed;
    float prev_right_drive_linear_speed = global_min_speed;

    global_min_speed = min_linear_speed;

    float left_drive_total_speed = 0;
    float right_drive_total_speed = 0;

    float time_limit = max_time + master_timer.time(seconds);
    float current_time = master_timer.time();

    float carrot_x_pos = target_x_pos - sin(degToRad(target_angle)) * (lead_distance * distance_error/initial_distance_error + setback_distance);
    float carrot_y_pos = target_y_pos - cos(degToRad(target_angle)) * (lead_distance * distance_error/initial_distance_error + setback_distance);

    float delta_x = target_x_pos - x_pos;       //center current position at origin
    float delta_y = target_y_pos - y_pos;       //center current position at origin
    double final_theta;

    float heading_scale_factor;

    bool state = 0; //state = 0 means chase setback point, state = 1 means chase target point

    if(delta_y == 0)
        delta_y = 0.00000001; 
    
    float modified_angle_for_tan = -1*degToRad(target_angle) + M_PI/2;

    float slope = 0;
    if((modified_angle_for_tan != M_PI/2)&&(modified_angle_for_tan != 3*M_PI/2))
        slope = tan(modified_angle_for_tan + M_PI/2); //slope of the perpendicular line which passes through the target point
    else
        slope = 99999999999;

    float y_intercept = target_y_pos -(slope * target_x_pos);

    bool initial_condition = false; //true = above line, false = below line
    if (y_pos > (slope*x_pos + y_intercept))
        initial_condition = true;
    else
        initial_condition = false;

    max_accel = max_accel / 100; // convert max_accel from %/1sec to %/10msec

    while(true){
        distance_error = sqrt(pow((target_x_pos-x_pos),2)+pow((target_y_pos-y_pos),2));

        if(state == 0){
            carrot_x_pos = target_x_pos - sin(degToRad(target_angle)) * (lead_distance * distance_error/initial_distance_error + setback_distance);
            carrot_y_pos = target_y_pos - cos(degToRad(target_angle)) * (lead_distance * distance_error/initial_distance_error + setback_distance);
        }
        delta_x = carrot_x_pos - x_pos;       //center current position at origin
        delta_y = carrot_y_pos - y_pos;       //center current position at origin

        distance_from_carrot = sqrt((delta_x*delta_x)+(delta_y*delta_y));

        if(distance_from_carrot < carrot_distance)
            state = 1;
        
        //printf("x:%f y:%f\n",delta_x,delta_y);
        final_theta = atan2(delta_x,delta_y); //calculate angle, atan2 gives values between -π and π, use delta_x, delta_y to define positive y-axis as 0 and clockwise as positive (reflection about line y=x)
        //printf("final_theta:%f\n",final_theta);
        if(final_theta < 0)                   //make all angles positive
            final_theta += (2 * M_PI);

        final_theta = radToDeg(final_theta);

        if(state == 1 )
            final_theta = target_angle;

        current_heading = inertial_sensor.heading(degrees);
        angular_error = final_theta - current_heading;       //calculate angle error
        //printf("F:%f C:%f\n",final_theta,current_heading);

        //printf("angular_error:%f\n",angular_error);
        if(angular_error > 180){                             //fix angle to smaller angle
            angular_error -= 360;
        }else if (angular_error <-180){
            angular_error += 360;
        }

        if(angular_error < -90){            //restrict angular error t0 -90 to 90 for cosine function
            angular_error = -90;
        }

        if(angular_error > 90){
            angular_error = 90;
        }

        if(fabs(angular_error) <= 4){                        //calculate integral error, cap is to reduce integral windup
            integral_angular_error += angular_error;
        }else{
            integral_angular_error = 0;
        }

        //printf("angular_error:%f\n",angular_error);

        derivative_angular_error = angular_error - previous_angular_error;   //calculate derivative error
        previous_angular_error = angular_error;                      //update previous error
        
        left_drive_angular_speed = angular_error * angular_kP + integral_angular_error * angular_kI + derivative_angular_error * angular_kD;  //set left drive speed
        right_drive_angular_speed = angular_error * angular_kP + integral_angular_error * angular_kI + derivative_angular_error * angular_kD; //set right drive speed

        if(fabs(left_drive_angular_speed) > max_angular_speed){
            if(left_drive_angular_speed < 0){
                left_drive_angular_speed  = -1 * max_angular_speed;
                right_drive_angular_speed = -1 * max_angular_speed; 
            } else {
                left_drive_angular_speed  = max_angular_speed;
                right_drive_angular_speed = max_angular_speed;
            }
        }

        /////////////////////////////////
        derivative_distance_error = distance_error - previous_distance_error;
        previous_distance_error = distance_error;

        if(fabs(distance_error) <= 2){
            integral_distance_error += distance_error;
        }else{
            integral_distance_error = 0;
        }

        if(fabs(integral_distance_error) > 5){
            if(integral_distance_error > 0){
                integral_distance_error = 5;
            } else {
                integral_distance_error = -5;
            }
        }



        left_drive_linear_speed = distance_error * linear_kP + integral_distance_error * linear_kI + derivative_distance_error * linear_kD;
        right_drive_linear_speed = distance_error * linear_kP + integral_distance_error * linear_kI + derivative_distance_error * linear_kD;


        if(fabs(left_drive_linear_speed) > max_linear_speed){
            if(left_drive_linear_speed < 0 ){
                left_drive_linear_speed  = -1 * max_linear_speed;
                right_drive_linear_speed = -1 * max_linear_speed;
            } else {
                left_drive_linear_speed = max_linear_speed;
                right_drive_linear_speed = max_linear_speed;
            }
        }

        // if(distance_error < dead_zone_distance){
        //     left_drive_angular_speed = 0;
        //     right_drive_angular_speed = 0;
        //     angular_error = 0;
        // }

        heading_scale_factor = cos(degToRad(angular_error));
        left_drive_linear_speed = left_drive_linear_speed * heading_scale_factor;   //scaling linear speeds by cos of angular speeds (drive train slows at large angular errors)
        right_drive_linear_speed = right_drive_linear_speed * heading_scale_factor;

        if(fabs(left_drive_linear_speed) < min_linear_speed){
            if(left_drive_linear_speed < 0){
                left_drive_linear_speed = -1 * min_linear_speed;
                right_drive_linear_speed = -1 * min_linear_speed;
            } else {
                left_drive_linear_speed = min_linear_speed;
                right_drive_linear_speed = min_linear_speed;
            }
        }

        if((left_drive_linear_speed > (prev_left_drive_linear_speed+max_accel))&&(max_accel != 0)){
            left_drive_linear_speed  = prev_left_drive_linear_speed  + max_accel;
            right_drive_linear_speed = prev_right_drive_linear_speed + max_accel;
        }
        //printf("left:%f right:%f\n",left_drive_total_speed,right_drive_total_speed);
        left_drive_total_speed  = left_drive_linear_speed + left_drive_angular_speed;
        right_drive_total_speed = right_drive_linear_speed - right_drive_angular_speed;

        //printf("left:%f right:%f error:%f i:%f\n",left_drive_linear_speed,right_drive_linear_speed,distance_error,integral_distance_error);
        left_drive.spin(fwd,left_drive_total_speed,pct);
        right_drive.spin(fwd,right_drive_total_speed,pct);

        if(initial_condition == true){
            if(y_pos < ((slope*x_pos + y_intercept))&&(distance_error < dead_zone_distance)){
                if(min_linear_speed == 0){
                    left_drive.stop();
                    right_drive.stop();
                }
                //printf("crossed line\n");
                break;
            }
        }

        if(initial_condition == false){
            if((y_pos > (slope*x_pos + y_intercept))&&(distance_error < dead_zone_distance)){
                if(min_linear_speed == 0){
                    left_drive.stop();
                    right_drive.stop();
                }
                //printf("crossed line\n");
                break;
            }
        }

        current_time = master_timer.time(seconds);
        if(current_time > time_limit){
            if(min_linear_speed == 0){
                left_drive.stop();
                right_drive.stop();
            }
            printf("timeout\n");
            break;
        }
        prev_left_drive_linear_speed = left_drive_linear_speed;
        prev_right_drive_linear_speed = right_drive_linear_speed;
        //printf("Cx:%f Cy:%f dis:%f\n",carrot_x_pos,carrot_y_pos,distance_from_carrot,distance_error);

        // if(initial_condition == true){
        //     if(y_pos < ((slope*x_pos + y_intercept)))
        //         printf("on other side of line");
        //     else
        //         printf("on wrong side of line");
        //     if(distance_error < dead_zone_distance)
        //         printf(" close enough\n");
        //     else
        //         printf(" not close enough\n");
        // }

        // if(initial_condition == false){
        //     if((y_pos > (slope*x_pos + y_intercept)))
        //         printf("on other side of line");
        //     else
        //         printf("on wrong side of line");
        //     if(distance_error < dead_zone_distance)
        //         printf(" close enough\n");
        //     else
        //         printf(" not close enough\n");   
        // }
        //printf("X:%f Y:%f Cx:%f Cy:%f dis:%f\n",x_pos,y_pos,carrot_x_pos,carrot_y_pos,distance_error);

        vex::task::sleep(10);
    }
    distance_error = sqrt(pow((target_x_pos-x_pos),2)+pow((target_y_pos-y_pos),2));
    printf("move to pose finished distance from target: %f\n",distance_error);
}

void move_to_pose_reverse(double target_x_pos, double target_y_pos, float target_angle, float lead_distance, float setback_distance, float max_time, float linear_kP, float linear_kI, float linear_kD, float angular_kP, float angular_kI, float angular_kD, float max_linear_speed, float min_linear_speed, float max_angular_speed, float dead_zone_distance, float carrot_distance, float max_accel){
    left_drive.setStopping(brake);
    right_drive.setStopping(brake);
    float current_heading = inertial_sensor.heading(degrees);

    float initial_distance_error = sqrt(pow((target_x_pos-x_pos),2)+pow((target_y_pos-y_pos),2));
    float distance_error = 0;
    float previous_distance_error = 0;
    float integral_distance_error = 0;
    float derivative_distance_error = 0;

    float distance_from_carrot = 0;

    float angular_error;
    float previous_angular_error = 0;
    float integral_angular_error = 0;
    float derivative_angular_error = 0;

    float left_drive_angular_speed = 0;
    float right_drive_angular_speed = 0;

    float left_drive_linear_speed = 0;
    float right_drive_linear_speed = 0;

    float prev_left_drive_linear_speed = global_min_speed;
    float prev_right_drive_linear_speed = global_min_speed;

    global_min_speed = min_linear_speed;

    float left_drive_total_speed = 0;
    float right_drive_total_speed = 0;

    float time_limit = max_time + master_timer.time(seconds);
    float current_time = master_timer.time();

    float carrot_x_pos = target_x_pos + sin(degToRad(target_angle)) * (lead_distance * distance_error/initial_distance_error + setback_distance);
    float carrot_y_pos = target_y_pos + cos(degToRad(target_angle)) * (lead_distance * distance_error/initial_distance_error + setback_distance);

    float delta_x = target_x_pos - x_pos;       //center current position at origin
    float delta_y = target_y_pos - y_pos;       //center current position at origin
    double final_theta;

    float heading_scale_factor;

    bool state = 0; //state = 0 means chase setback point, state = 1 means chase target point

    if(delta_y == 0)
        delta_y = 0.00000001; 
    
    float modified_angle_for_tan = -1*degToRad(target_angle) + M_PI/2;

    float slope = 0;
    if((modified_angle_for_tan != M_PI/2)&&(modified_angle_for_tan != 3*M_PI/2))
        slope = tan(modified_angle_for_tan + M_PI/2); //slope of the perpendicular line which passes through the target point
    else
        slope = 99999999999;

    float y_intercept = target_y_pos -(slope * target_x_pos);

    bool initial_condition = false; //true = above line, false = below line
    if (y_pos > (slope*x_pos + y_intercept))
        initial_condition = true;
    else
        initial_condition = false;

    max_accel = max_accel / 100; // convert max_accel from %/1sec to %/10msec

    while(true){
        distance_error = sqrt(pow((target_x_pos-x_pos),2)+pow((target_y_pos-y_pos),2));

        if(state == 0){
            carrot_x_pos = target_x_pos + sin(degToRad(target_angle)) * (lead_distance * distance_error/initial_distance_error + setback_distance);
            carrot_y_pos = target_y_pos + cos(degToRad(target_angle)) * (lead_distance * distance_error/initial_distance_error + setback_distance);
        }
        
        delta_x = carrot_x_pos - x_pos;       //center current position at origin
        delta_y = carrot_y_pos - y_pos;       //center current position at origin

        distance_from_carrot = sqrt((delta_x*delta_x)+(delta_y*delta_y));

        if(distance_from_carrot < carrot_distance)
            state = 1;
        
        //printf("x:%f y:%f\n",delta_x,delta_y);
        final_theta = atan2(delta_x,delta_y); //calculate angle, atan2 gives values between -π and π, use delta_x, delta_y to define positive y-axis as 0 and clockwise as positive (reflection about line y=x)
        //printf("final_theta:%f\n",final_theta);
        if(final_theta < 0)                   //make all angles positive
            final_theta += (2 * M_PI);

        final_theta = radToDeg(final_theta);

        if(state == 1 )
            final_theta = target_angle+180;

        current_heading = inertial_sensor.heading(degrees);
        angular_error = final_theta - current_heading;       //calculate angle error

        angular_error += 180;
        //printf("F:%f C:%f\n",final_theta,current_heading);

        //printf("angular_error:%f\n",angular_error);
        if(angular_error > 180){                             //fix angle to smaller angle
            angular_error -= 360;
        }else if (angular_error <-180){
            angular_error += 360;
        }

        if(angular_error < -90){            //restrict angular error t0 -90 to 90 for cosine function
            angular_error = -90;
        }

        if(angular_error > 90){
            angular_error = 90;
        }

        if(fabs(angular_error) <= 4){                        //calculate integral error, cap is to reduce integral windup
            integral_angular_error += angular_error;
        }else{
            integral_angular_error = 0;
        }

        //printf("angular_error:%f\n",angular_error);

        derivative_angular_error = angular_error - previous_angular_error;   //calculate derivative error
        previous_angular_error = angular_error;                      //update previous error
        
        left_drive_angular_speed = angular_error * angular_kP + integral_angular_error * angular_kI + derivative_angular_error * angular_kD;  //set left drive speed
        right_drive_angular_speed = angular_error * angular_kP + integral_angular_error * angular_kI + derivative_angular_error * angular_kD; //set right drive speed

        if(fabs(left_drive_angular_speed) > max_angular_speed){
            if(left_drive_angular_speed < 0){
                left_drive_angular_speed  = -1 * max_angular_speed;
                right_drive_angular_speed = -1 * max_angular_speed; 
            } else {
                left_drive_angular_speed  = max_angular_speed;
                right_drive_angular_speed = max_angular_speed;
            }
        }

        /////////////////////////////////
        derivative_distance_error = distance_error - previous_distance_error;
        previous_distance_error = distance_error;

        if(fabs(distance_error) <= 2){
            integral_distance_error += distance_error;
        }else{
            integral_distance_error = 0;
        }

        if(fabs(integral_distance_error) > 5){
            if(integral_distance_error > 0){
                integral_distance_error = 5;
            } else {
                integral_distance_error = -5;
            }
        }



        left_drive_linear_speed = distance_error * linear_kP + integral_distance_error * linear_kI + derivative_distance_error * linear_kD;
        right_drive_linear_speed = distance_error * linear_kP + integral_distance_error * linear_kI + derivative_distance_error * linear_kD;


        if(fabs(left_drive_linear_speed) > max_linear_speed){
            if(left_drive_linear_speed < 0 ){
                left_drive_linear_speed  = -1 * max_linear_speed;
                right_drive_linear_speed = -1 * max_linear_speed;
            } else {
                left_drive_linear_speed = max_linear_speed;
                right_drive_linear_speed = max_linear_speed;
            }
        }

        // if(distance_error < dead_zone_distance){
        //     left_drive_angular_speed = 0;
        //     right_drive_angular_speed = 0;
        //     angular_error = 0;
        // }

        heading_scale_factor = cos(degToRad(angular_error));
        left_drive_linear_speed = left_drive_linear_speed * heading_scale_factor;   //scaling linear speeds by cos of angular speeds (drive train slows at large angular errors)
        right_drive_linear_speed = right_drive_linear_speed * heading_scale_factor;

        if(fabs(left_drive_linear_speed) < min_linear_speed){
            if(left_drive_linear_speed < 0){
                left_drive_linear_speed = -1 * min_linear_speed;
                right_drive_linear_speed = -1 * min_linear_speed;
            } else {
                left_drive_linear_speed = min_linear_speed;
                right_drive_linear_speed = min_linear_speed;
            }
        }

        if((left_drive_linear_speed > (prev_left_drive_linear_speed+max_accel))&&(max_accel != 0)){
            left_drive_linear_speed  = prev_left_drive_linear_speed  + max_accel;
            right_drive_linear_speed = prev_right_drive_linear_speed + max_accel;
        }
        //printf("left:%f right:%f\n",left_drive_total_speed,right_drive_total_speed);
        left_drive_total_speed  = left_drive_linear_speed - left_drive_angular_speed;
        right_drive_total_speed = right_drive_linear_speed + right_drive_angular_speed;

        //printf("left:%f right:%f error:%f i:%f\n",left_drive_linear_speed,right_drive_linear_speed,distance_error,integral_distance_error);
        left_drive.spin(reverse,left_drive_total_speed,pct);
        right_drive.spin(reverse,right_drive_total_speed,pct);

        if(initial_condition == true){
            if(y_pos < ((slope*x_pos + y_intercept))&&(distance_error < dead_zone_distance)){
                if(min_linear_speed == 0){
                    left_drive.stop();
                    right_drive.stop();
                }
                printf("crossed line\n");
                break;
            }
        }

        if(initial_condition == false){
            if((y_pos > (slope*x_pos + y_intercept))&&(distance_error < dead_zone_distance)){
                if(min_linear_speed == 0){
                    left_drive.stop();
                    right_drive.stop();
                }
                printf("crossed line\n");
                break;
            }
        }

        current_time = master_timer.time(seconds);
        if(current_time > time_limit){
            if(min_linear_speed == 0){
                left_drive.stop();
                right_drive.stop();
            }
            printf("timeout\n");
            break;
        }
        prev_left_drive_linear_speed = left_drive_linear_speed;
        prev_right_drive_linear_speed = right_drive_linear_speed;
        //printf("X:%f Y:%f Cx:%f Cy:%f dis:%f\n",x_pos,y_pos,carrot_x_pos,carrot_y_pos,distance_error);

        // if(initial_condition == true){
        //     if(y_pos < ((slope*x_pos + y_intercept)))
        //         printf("on other side of line");
        //     else
        //         printf("on wrong side of line");
        //     if(distance_error < dead_zone_distance)
        //         printf(" close enough\n");
        //     else
        //         printf(" not close enough\n");
        // }

        // if(initial_condition == false){
        //     if((y_pos > (slope*x_pos + y_intercept)))
        //         printf("on other side of line");
        //     else
        //         printf("on wrong side of line");
        //     if(distance_error < dead_zone_distance)
        //         printf(" close enough\n");
        //     else
        //         printf(" not close enough\n");   
        // }


        vex::task::sleep(10);
    }
    distance_error = sqrt(pow((target_x_pos-x_pos),2)+pow((target_y_pos-y_pos),2));
    printf("move to pose finished distance from target: %f\n",distance_error);
}

