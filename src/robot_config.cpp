#include "robot_config.hpp"
#include "main.h"

// --- Global Accumulators for all 3 AS5600s ---
double dist_left_v = 0;  int last_left_v = -1;
double dist_right_v = 0; int last_right_v = -1;
double dist_horiz_x = 0; int last_horiz_x = -1;

// Helper function to handle AS5600 math
float update_encoder(int port, double &total, int &last) {
    int current = pros::adi::AnalogIn(port).get_value();
    if (last == -1) last = current;
    int delta = current - last;
    if (delta > 2048) delta -= 4096;
    else if (delta < -2048) delta += 4096;
    total += (delta / 4095.0) * (2.75 * M_PI); // Adjust for your wheel size
    last = current;
    return total;
}

// --- Motors ---
pros::MotorGroup left_motor_group({-2, -3, -1}, pros::MotorGears::green);
pros::MotorGroup right_motor_group({10, 8, 9}, pros::MotorGears::green);
pros::Motor catapult_arm(7, pros::MotorGears::red);
pros::Motor intake(4, pros::MotorGears::green);
pros::Motor matchloader(5, pros::MotorGears::red);
pros::Motor discore(12, pros::MotorGears::green);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// --- Drivetrain & Odometry Setup ---
lemlib::Drivetrain drivetrain(&left_motor_group, &right_motor_group, 14.5, lemlib::Omniwheel::NEW_325, 400, 2);

// Wrapper functions for LemLib
float get_left_v_dist()  { return update_encoder('C', dist_left_v, last_left_v); }
float get_right_v_dist() { return update_encoder('D', dist_right_v, last_right_v); }
float get_horiz_x_dist() { return update_encoder('E', dist_horiz_x, last_horiz_x); }

// --- LemLib Initialization ---
// Vertical wheels track Y and Heading. Horizontal tracks X.
lemlib::TrackingWheel vert_left(get_left_v_dist, 2, -4.600); 
lemlib::TrackingWheel vert_right(get_right_v_dist, 2, 1.700);
lemlib::TrackingWheel horizontal(get_horiz_x_dist, 2, -4.500);

// lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_325, -5.75);
// lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_325, -2.5);

lemlib::OdomSensors sensors(&vert_left, &vert_right, &horizontal, nullptr, nullptr);
lemlib::ControllerSettings lateral_controller(10, 0, 3, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angular_controller(2, 0, 10, 0, 0, 0, 0, 0, 0);
lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.019);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttle_curve, &steer_curve);