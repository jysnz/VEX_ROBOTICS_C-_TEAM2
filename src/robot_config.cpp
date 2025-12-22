#include "robot_config.hpp"

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
pros::adi::Ultrasonic ultrasonic('A', 'B');
pros::Imu imu(17);
pros::Rotation horizontal_encoder(20);
pros::adi::Encoder vertical_encoder('C', 'D', true);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_325, -5.75);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_325, -2.5);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr, &imu);
lemlib::ControllerSettings lateral_controller(10, 0, 3, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angular_controller(2, 0, 10, 0, 0, 0, 0, 0, 0);
lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.019);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttle_curve, &steer_curve);