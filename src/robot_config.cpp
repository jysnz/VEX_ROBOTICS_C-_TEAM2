#include "robot_config.hpp"
#include "main.h"

// --- Motors ---
pros::MotorGroup left_motor_group({-2, -3, -1}, pros::MotorGears::green);
pros::MotorGroup right_motor_group({10, 8, 9}, pros::MotorGears::green);
pros::Motor catapult_arm(7, pros::MotorGears::red);
pros::Motor intake(4, pros::MotorGears::green);
pros::Motor matchloader(5, pros::MotorGears::red);
pros::Motor discore(12, pros::MotorGears::green);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::adi::Ultrasonic ultrasonic('D', 'E');

// --- Drivetrain & Odometry Setup ---
lemlib::Drivetrain drivetrain(&left_motor_group, &right_motor_group, 14.5,
                              lemlib::Omniwheel::NEW_325, 500, 4);

lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr,
                            nullptr);
lemlib::ControllerSettings lateral_controller(6, 0.001, 2, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angular_controller(1.20, 0.100, 10.20, 0, 0, 0, 0, 0, 0);
lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.019);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors, &throttle_curve, &steer_curve);