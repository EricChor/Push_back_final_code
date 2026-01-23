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

double theta_ccw_rad = 0.0;
double prev_theta_ccw_rad = 0.0;

void update_pose() {
    // Read sensors
    vertical_encoder_pos = vertical_tracking.position(vex::degrees);
    lateral_encoder_pos  = lateral_tracking.position(vex::degrees);
    theta_pos            = inertial_sensor.heading(vex::degrees); // CW-positive, 0..360

    // Encoder deltas (deg)
    double dVert_deg = vertical_encoder_pos - prev_vertical_encoder_pos;
    double dLat_deg  = lateral_encoder_pos  - prev_lateral_encoder_pos;

    // IMU delta (deg), CW-positive
    double dTheta_cw_deg = theta_pos - prev_theta_pos;
    if (fabs(dTheta_cw_deg) > 180.0) {
        if (dTheta_cw_deg > 0) dTheta_cw_deg -= 360.0;
        else                   dTheta_cw_deg += 360.0;
    }

    // Convert encoder deltas to inches
    double dVert_in = degToRad(dVert_deg) * (vertical_wheel_diameter / 2.0);
    double dLat_in  = degToRad(dLat_deg)  * (lateral_wheel_diameter  / 2.0);

    // Convert CW-positive delta to CCW-positive radians for the math
    double dTheta = -degToRad(dTheta_cw_deg);   // IMPORTANT: negate

    // Build a CCW-positive heading for transform (store and integrate it)
    // This avoids having to convert absolute 0..360 headings each time.
    theta_ccw_rad += dTheta; // theta_ccw_rad is a persistent state variable (global/static)

    const double theta_eps = 1e-4;  // rad (tune)
    const double move_eps  = 1e-3;  // inches (tune)

    // Stationary gate: if wheels didn’t move, don’t let IMU jitter create translation
    if (fabs(dVert_in) < move_eps && fabs(dLat_in) < move_eps) {
        prev_vertical_encoder_pos = vertical_encoder_pos;
        prev_lateral_encoder_pos  = lateral_encoder_pos;
        prev_theta_pos            = theta_pos;
        vex::task::sleep(10);
        return;
    }

    // Local deltas (robot frame)
    double local_dx = 0.0;
    double local_dy = 0.0;

    if (fabs(dTheta) < theta_eps) {
        // Small-angle stable limit:
        // local ≈ Δencoder - offset * Δθ
        local_dy = dVert_in - vertical_encoder_distance_from_arc_center * dTheta;
        local_dx = dLat_in  - lateral_encoder_distance_from_arc_center  * dTheta;
    } else {
        // Arc/chord model
        double s = 2.0 * sin(dTheta / 2.0);

        double rVert = (dVert_in / dTheta) - vertical_encoder_distance_from_arc_center;
        double rLat  = (dLat_in  / dTheta) - lateral_encoder_distance_from_arc_center;

        local_dy = rVert * s;
        local_dx = rLat  * s;
    }

    // Transform local -> global using average heading over the interval (CCW-positive)
    double transform_theta = -(prev_theta_ccw_rad + dTheta / 2.0);

    double global_dx = local_dx * cos(transform_theta) - local_dy * sin(transform_theta);
    double global_dy = local_dx * sin(transform_theta) + local_dy * cos(transform_theta);

    x_pos += global_dx;
    y_pos += global_dy;

    // Update previous
    prev_vertical_encoder_pos = vertical_encoder_pos;
    prev_lateral_encoder_pos  = lateral_encoder_pos;
    prev_theta_pos            = theta_pos;

    prev_theta_ccw_rad = theta_ccw_rad;  // keep prev in sync (persistent)
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
