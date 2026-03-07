#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include "pros/motors.hpp"
#include "pros/adi.hpp"
#include "pros/rtos.hpp"
#include "pros/misc.h"
#include "lemlib/api.hpp"

// ============== DRIVETRAIN MOTORS ==============
extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;

// ============== MECHANISM MOTORS ==============
extern pros::Motor catapult_arm;
extern pros::Motor intake;
extern pros::Motor intake2;
extern pros::Motor intake3;
extern pros::Motor matchloader;
extern pros::Motor discore;
extern pros::Motor switchScore;

// ============== SENSORS ==============
extern pros::Imu imu;                          // Inertial measurement unit
extern pros::Rotation horizontal_encoder;      // Horizontal tracking wheel
extern pros::adi::Encoder vertical_encoder;    // Vertical tracking wheel

// ============== DRIVER CONTROLLER ==============
extern pros::Controller controller;

// ============== LEMLIB CHASSIS COMPONENTS ==============
extern lemlib::Drivetrain drivetrain;
extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern lemlib::TrackingWheel vertical_tracking_wheel;
extern lemlib::OdomSensors sensors;
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::ExpoDriveCurve throttle_curve;
extern lemlib::ExpoDriveCurve steer_curve;
extern lemlib::Chassis chassis;

// ============== MOTOR INITIALIZATION FUNCTION ==============
/**
 * Initializes all motors, sensors, and mechanical components
 */
void initialize_motors();

#endif // MOTOR_CONFIG_H
