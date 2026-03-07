#include "utils.h"
#include "constants.h"
#include "motor_config.h"
#include <iostream>

// ============== MATH UTILITIES ==============
double sign(double x) {
    return (x > 0) - (x < 0);
}

// ============== UNIT CONVERSION ==============
float ticksToInches(float ticks) {
    return (ticks / Drivetrain::TICKS_PER_REV) * Drivetrain::PI * Drivetrain::WHEEL_DIAMETER;
}

double inchesToDegrees(double inches) {
    double wheelCircumference = Drivetrain::PI * Drivetrain::WHEEL_DIAMETER;
    double rotations = inches / wheelCircumference;
    return rotations * 360.0;  // degrees
}

int turn_deg_to_ms(double degrees) {
    return static_cast<int>((degrees / 90.0) * Drivetrain::MS_PER_90_DEG);
}

// ============== SENSOR UTILITIES ==============
double get_distance_cm() {
    pros::ADIUltrasonic ultrasonic('A', 'B');
    int raw_value = ultrasonic.get_value();

    // PROS returns -1 if the sensor is out of range or not plugged in
    if (raw_value <= 0) {
        return -1.0;
    }

    // Convert millimeters (PROS default) to centimeters
    return (double)raw_value / 10.0;
}

void ultrasonicSense() {
    pros::ADIUltrasonic ultrasonic('A', 'B');
    while (true) {
        std::cout << "Distance: " << ultrasonic.get_value();
        pros::delay(10);
    }
}
