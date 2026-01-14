#ifndef _ROBOT_CONFIG_HPP_
#define _ROBOT_CONFIG_HPP_

#include "main.h"
#include "lemlib/api.hpp"

// --- Constants ---
extern const double wheelDiameter;
extern const double trackWidth;
extern const double ticksPerRev;
extern const int MAX_VOLTAGE;

void update_lateral_pid(float p, float i, float d);
void update_angular_pid(float p, float i, float d);

// --- Motors & Controllers ---
extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;
extern pros::Motor catapult_arm;
extern pros::Motor intake;
extern pros::Motor matchloader;
extern pros::Motor discore;
extern pros::Controller controller;

// --- Sensors ---
extern pros::adi::Ultrasonic ultrasonic;
// [CHANGE] Declare the GPS sensor here
extern pros::Gps gps_sensor; 

// --- Chassis ---
extern lemlib::Chassis chassis;

#endif // _ROBOT_CONFIG_HPP_