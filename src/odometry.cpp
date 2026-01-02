#include "robot_config.h"
#include "trig.h"
#include "odometry.h"

double x_pos = 0;

double y_pos = 0;

double theta = 0; //angle in degrees

double initial_theta = 0;

double vertical_wheel_diameter = 0;

double lateral_wheel_diameter = 0;

double vertical_encoder_distance_from_arc_center = 0; 

double lateral_encoder_distance_from_arc_center = 0;

double vertical_encoder_pos = 0;

double prev_vertical_encoder_pos = 0;

double lateral_encoder_pos = 0;

double prev_lateral_encoder_pos = 0;

double theta_pos = 0;

double prev_theta_pos = 0;

double drivebase_gear_ratio = 1;

double delta_vertical_encoder_pos = 0;

double delta_lateral_encoder_pos = 0;

double delta_theta = 0;

double arc_radius = 0;

double lateral_arc_radius = 0;

double local_delta_y = 0;

double local_delta_x = 0;

double transform_theta = 0;

double global_delta_x = 0;

double global_delta_y = 0;

void odom_setup(double vertical_encoder_distance_from_center,double lateral_encoder_distance_from_center, double vertical_wheel_diam, double lateral_wheel_diam, double gear_ratio){
    vertical_encoder_distance_from_arc_center = vertical_encoder_distance_from_center; //the horitzontal distance from the center vertical tracking wheel to the tracking center, vertical is defined as parallel to the forward/reverse direction of the robot
    lateral_encoder_distance_from_arc_center = lateral_encoder_distance_from_center;   //the vertical distance from the center of the horizontal tracking wheel to the tracking center, horizontal is defined as perpendicular to the vertical direction
    lateral_wheel_diameter = lateral_wheel_diam;           //diameter of tracking wheels               //left is negative, right is positive | back is negative, front is positive
    vertical_wheel_diameter = vertical_wheel_diam;
    drivebase_gear_ratio = (1/gear_ratio); //need to take out if using tracking wheels | gear ratio = driven / driving

    // left_middle_motor.resetPosition();     //reset tracking encoders
    inertial_sensor.resetHeading();        //reset inertial sensor
    vertical_tracking.resetPosition();
    lateral_tracking.resetPosition();
}

void set_pose(double initial_x_pos,double initial_y_pos,double starting_theta){
    x_pos = initial_x_pos; //set initial x pose
    y_pos = initial_y_pos; //set initial y pose
    theta = starting_theta;
    inertial_sensor.setHeading(starting_theta,degrees);
}

// float lateral_accumulator = 0;

void update_pose(){
    vertical_encoder_pos = vertical_tracking.position(degrees); //update vertical encoder position
    lateral_encoder_pos = lateral_tracking.position(degrees);
    //printf("vertical_encoder_pos:%f\n",vertical_encoder_pos);                             //update lateral encoder position (need to change, no lateral encoder)
    theta_pos = inertial_sensor.heading(degrees);          //update inertial sensor position
    //printf("theta_pos:%f\n",theta_pos);

    delta_vertical_encoder_pos = vertical_encoder_pos - prev_vertical_encoder_pos; //calculate change in vertical encoder position
    //printf("delta_vertical_encoder:%f\n",delta_vertical_encoder_pos);
    delta_lateral_encoder_pos  = lateral_encoder_pos - prev_lateral_encoder_pos;   //calculate change in lateral encoder position

    delta_theta = theta_pos - prev_theta_pos;                                      //calculate change in inertial sensor position
    //printf("%f:%f:delta theta:%f\n",theta_pos,prev_theta_pos,delta_theta);

    //printf("gear ratio:%f\n",drivebase_gear_ratio);
    delta_vertical_encoder_pos = degToRad(delta_vertical_encoder_pos) * (vertical_wheel_diameter/2); //convert angle to linear distance
    
    //delta_vertical_encoder_pos = 3.25 * M_PI * delta_vertical_encoder_pos * 3/5; //convert angle to linear distance
    //printf("drivebase gear ratio:%f\n",drivebase_gear_ratio);

    //delta_vertical_encoder_pos = wheel_diameter * M_PI * delta_vertical_encoder_pos * drivebase_gear_ratio; //convert angle to linear distance
    delta_lateral_encoder_pos = degToRad(delta_lateral_encoder_pos) * (lateral_wheel_diameter/2);
    if(delta_theta == 0){ //maybe try fabs(delta_theta) < epsilon, need to see what noise values there are first
        local_delta_y = delta_vertical_encoder_pos;
        local_delta_x = delta_lateral_encoder_pos;
    } else {
        arc_radius = delta_vertical_encoder_pos / degToRad(delta_theta) - vertical_encoder_distance_from_arc_center; //calculates the radius one side of the turning arc (assumes encoder is on left)
        lateral_arc_radius = delta_lateral_encoder_pos / degToRad(delta_theta) - lateral_encoder_distance_from_arc_center;//calculates the radius of the lateral turning arc
    
        if(delta_vertical_encoder_pos == 0) //accounts for noise from the inertial sensor when robot is not moving
            arc_radius = 0;
        if(delta_lateral_encoder_pos == 0)
            lateral_arc_radius = 0;


    //printf("arc_radius:%f\n",arc_radius);

    //printf("vert:%f horz:%f\n",arc_radius,lateral_arc_radius);

        local_delta_y = 2 * arc_radius * sin(degToRad(delta_theta) / 2);          //calculates the y distance in the rotated axes using the chord length formula
        local_delta_x = 2 * lateral_arc_radius * sin(degToRad(delta_theta) / 2);  //need lateral tracking wheel
    }

    //printf("arc_radius:%f\n",arc_radius);
    //printf("l_delta_y:%f l_delta_x:%f\n",local_delta_y,local_delta_x);
    //(M_PI/2)-atan((sin(degToRad(delta_theta))/(1-cos(degToRad(delta_theta)) = delta_theta / 2
    transform_theta = -1*(degToRad(prev_theta_pos) + (degToRad(delta_theta)/2)); //calculates the angle for the linear transform

    global_delta_x = local_delta_x * cos(transform_theta) - local_delta_y * sin(transform_theta);                   //calculates dx/dt in the global coordinates from linear transform
    global_delta_y = local_delta_x * sin(transform_theta) + local_delta_y * cos(transform_theta);                   //calculates dy/dt in the global coordinates from linear transform
    
    //printf("local y:%f local x: %f\n",global_delta_y,global_delta_x);
    
    x_pos += global_delta_x; //update global x pos
    y_pos += global_delta_y; //update global y pos
    //printf("X:%f Y:%f θ:%f\n",x_pos,y_pos,theta_pos);
    //update

    prev_vertical_encoder_pos = vertical_encoder_pos; //update previous vertical encoder pos
    prev_lateral_encoder_pos  = lateral_encoder_pos;  //update previous lateral encoder pos
    prev_theta_pos = theta_pos;                       //update previous theta pos
    // lateral_accumulator += delta_lateral_encoder_pos;
    vex::task::sleep(10);
}

void odom_thread(){
    while(true)
    update_pose();
}

void odom_telemetry(){
    //float left_motor_pos = left_middle_motor.position(degrees);
    while(true){
        // left_motor_pos = left_middle_motor.position(degrees);
        // left_motor_pos = degToRad(left_motor_pos) * (wheel_diameter/2) * drivebase_gear_ratio;
        // printf("%f\n",left_motor_pos);

        print_odom_pos();
        vex::task::sleep(40);
    }
}

void print_odom_pos(){
    printf("X:%f Y:%f θ:%f\n\n",x_pos,y_pos,theta_pos);
}
