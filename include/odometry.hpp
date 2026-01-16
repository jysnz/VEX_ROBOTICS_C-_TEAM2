#ifndef _ODOMETRY_HPP_
#define _ODOMETRY_HPP_

#include "main.h"

// ==========================================
//          PID CONFIG STRUCTURE
// ==========================================
struct PIDConfig {
    double kP;
    double kI;
    double kD;
};
extern double prev_L;
extern double prev_R;

// Global config objects (defined in .cpp)
extern PIDConfig forwardPID_Consts;
extern PIDConfig turnPID_Consts;

// --- AS5600 Wrapper Class ---
class AS5600 {
private:
    pros::adi::AnalogIn sensor;
    int last_raw;
    double total_ticks = 0;
    bool reversed;

    // Filter variables
    double filtered_val = 0;
    const double alpha = 0.7; 

public:
    AS5600(char port, bool is_reversed);
    void update();
    void calibrate();
    double get_inches(); 
    void reset();
    int get_raw();
};

// --- Global Position Variables ---
extern double robot_x;
extern double robot_y;
extern double robot_theta; // In Radians
extern pros::Mutex odom_mutex;

// --- Sensor Objects ---
// Updated to reflect the 2-wheel vertical setup
extern AS5600 forward_odom; // Left Tracker
extern AS5600 heading_odom; // Right Tracker

// --- Function Prototypes ---

// Background Tasks
void odom_task_fn(void* ignore);
void debug_task_fn(void* ignore);

// Visualization
void resetGraph();
void drawTargetGraph(double target, double current, double scale);

// Position Correction Helpers
void resetOdometry();
void fixPositionWithUltrasonic();
void fixPoseWithGPS();

// PID Movements
void driveReversePID(double targetDistance, double maxSpeed, double timeout);
void driveForwardPID(double targetDistance, double maxSpeed, double timeout);
void turnToAnglePID(double targetAngleDeg, double maxSpeed, double timeout);

// REAL-TIME TUNER
void tuningLoop();

#endif // _ODOMETRY_HPP_