#ifndef _ODOMETRY_HPP_
#define _ODOMETRY_HPP_

#include "main.h"

// AS5600 Wrapper Class
class AS5600 {
private:
    pros::adi::AnalogIn sensor;
    int32_t total_ticks = 0;
    int last_raw = 0;
    const int DEADBAND = 5; // Ignore fluctuations smaller than 5 ticks

public:
    AS5600(char port);
    void update();
    double get_inches(double diameter);
    int get_raw(); 
    void reset();
};

// Global Position Variables (accessible in main.cpp)
extern double robot_x;
extern double robot_y;
extern double robot_theta;

// Raw Debug Variables
extern int forward_odom_raw;
extern int heading_odom_raw;
extern int sideways_odom_raw;

// Function to start the tracking task
void start_odom();
void setPose(double newX, double newY, double newTheta);
void driveToPointWithLogging(double targetX, double targetY, double timeout);
void drawPIDGraph(double error, int timeStep, bool isTurning);
void driveForwardPID(double targetDistance, double maxSpeed, double timeout);
void turnToAnglePID(double targetAngleDeg, double maxSpeed, double timeout);
double get_smooth_error(double target, double start);

#endif