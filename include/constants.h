#ifndef CONSTANTS_H
#define CONSTANTS_H

// ============== DRIVETRAIN CONSTANTS ==============
namespace Drivetrain {
    const double WHEEL_DIAMETER = 3.25;        // inches
    const double TRACK_WIDTH = 12.0;           // distance between wheels
    const double TICKS_PER_REV = 360.0;        // motor degrees per revolution
    const float PI = 3.14159;
    const double MS_PER_90_DEG = 660.0;        // ms to turn 90 degrees at TURN_SPEED
    const double STOP_TOLERANCE = 5.0;         // Stop when within +/- 5 motor degrees
    
    extern double TURN_CALIBRATION;            // dynamically adjustable
    
    // PID constants for drivetrain
    namespace PID {
        const float KP = 1.0;
        const float KI = 0.0;
        const float KD = 0.5;
    }
    
    // Acceleration/deceleration settings
    namespace Motion {
        const double ACCEL_RATE = 2.0;
        const double DECEL_START = 0.6;        // Start decelerating at 60% of distance
        const double MIN_FINAL_SPEED = 10.0;
        const double VEL_THRESH = 5.0;         // rpm threshold for wall detection
        const int STOP_TIME_MS = 150;          // time to consider stopped
        
        // Voltage-based settings
        const double ACCEL_STEP = 300.0;       // voltage per 10ms
        const double MIN_VOLTAGE = 1200.0;     // prevents stall
        const int RAMP_DECREMENT = 200;        // voltage ramp down step
    }
}

// ============== CATAPULT CONSTANTS ==============
namespace Catapult {
    const int LOAD_POS = 0;
    const int FIRE_POS = -600;
    const int SPEED = 200;
    
    const int STALL_TIME = 250;
    const int CHECK_DELAY = 10;
    const int MAX_ATTEMPTS = 10;
}

// ============== MECHANISM THRESHOLDS ==============
namespace Mechanisms {
    const int POSITION_TOLERANCE = 10;         // degrees
    const int STALL_VELOCITY_THRESHOLD = 3;    // rpm
    const int RESET_TIMEOUT = 2000;            // ms
}

// ============== ODOMETRY CONSTANTS ==============
namespace Odometry {
    const int UPDATE_DELAY = 10;               // ms between updates
}

#endif // CONSTANTS_H
