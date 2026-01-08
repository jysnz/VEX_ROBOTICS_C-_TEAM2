#ifndef _ODOMETRY_HPP_
#define _ODOMETRY_HPP_

#include "main.h"

// --- AS5600 Wrapper Class ---
// Updated to match the noise-filtered, wrapping-handling implementation
class AS5600 {
private:
    pros::adi::AnalogIn sensor;
    int last_raw;
    double total_ticks = 0;
    bool reversed;

    // Filter variables
    double filtered_val = 0;
    const double alpha = 0.7; // Smoothing factor

public:
    // Constructor now takes the 'reversed' flag
    AS5600(char port, bool is_reversed);

    void update();
    void calibrate();
    
    // No longer needs diameter passed in (uses global constant in .cpp)
    double get_inches(); 
    
    void reset();
    int get_raw();
};

// --- Global Position Variables ---
// These are updated by the background task
extern double robot_x;
extern double robot_y;
extern double robot_theta; // In Radians

// --- Sensor Objects ---
// Exposed so you can access them in main.cpp if needed
extern AS5600 forward_odom;
extern AS5600 heading_odom;
extern AS5600 sideways_odom;

// --- Function Prototypes ---

// Background Task Function
void odom_task_fn(void* ignore);

// Visualization
void drawPIDGraph(double error, int timeStep, bool isTurning);

// PID Movements
void driveForwardPID(double targetDistance, double maxSpeed, double timeout);
void turnToAnglePID(double targetAngleDeg, double maxSpeed, double timeout);
void debug_task_fn(void* ignore);
int get_raw();


#endif // _ODOMETRY_HPP_