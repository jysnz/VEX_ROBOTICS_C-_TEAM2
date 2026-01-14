#include "odometry.hpp"
#include <cmath>
#include <algorithm> // For std::clamp
#include "robot_config.hpp" // Access to motor groups and ultrasonic
#include <cfloat> // <--- ADD THIS for DBL_MAX

// ==========================================
//          CONFIGURATION CONSTANTS
// ==========================================
// MEASURE THESE EXACTLY
const double WHEEL_DIAMETER = 3.05; 
// Distance between the two tracking wheels
// TUNE THIS: If robot spins 360 but says 350, DECREASE. If 370, INCREASE.
double TRACK_WIDTH = 5.94; 

// Distance from tracking center to Ultrasonic Sensor (in inches)
const double ULTRASONIC_OFFSET = 5.0; 

// ==========================================
//          GLOBAL VARIABLES
// ==========================================
double robot_x = 0;
double robot_y = 0;
double robot_theta = 0; // Radians
bool useGpsHealing = false; // Starts OFF for pure PID tuning

pros::Mutex odom_mutex;

// Tuning Graph Helper
int graph_x = 0;
int testMode = 0;

// PID Defaults (Starting Values)
PIDConfig forwardPID_Consts = { 6.00, 0.001, 2.00 };
PIDConfig turnPID_Consts    = { 2.50, 0.005, 12.00 }; // Adjusted for 2-wheel

// Toggle for the Tuner
bool tuningForward = true; 

// ==========================================
//          AS5600 IMPLEMENTATION
// ==========================================
AS5600::AS5600(char port, bool is_reversed) : sensor(port), reversed(is_reversed) {
    last_raw = 0;
    total_ticks = 0;
    filtered_val = 0;
}

void AS5600::calibrate() {
    last_raw = sensor.get_value();
    pros::delay(20); 
    last_raw = sensor.get_value();
    reset();
}

int AS5600::get_raw() {
    return sensor.get_value();
}

void AS5600::update() {
    int current_raw = sensor.get_value();
    int delta = current_raw - last_raw;

    // WRAP LOGIC
    if (delta > 2048) delta -= 4096; 
    else if (delta < -2048) delta += 4096; 

    // ACCUMULATE
    if (reversed) total_ticks -= delta;
    else total_ticks += delta;

    last_raw = current_raw;
}

double AS5600::get_inches() {
    double raw_inches = (total_ticks / 4096.0) * (WHEEL_DIAMETER * M_PI);
    // Simple filter
    filtered_val = (0.7 * raw_inches) + (0.3 * filtered_val);
    return filtered_val;
}

void AS5600::reset() {
    total_ticks = 0;
    filtered_val = 0;
    last_raw = sensor.get_value();
}

// ==========================================
//          SENSOR OBJECTS
// ==========================================   
// Check these ports/directions match your physical wiring!
AS5600 forward_odom('C', true);   // Treated as LEFT Wheel
AS5600 heading_odom('B', false);  // Treated as RIGHT Wheel

// GPS SENSOR DEFINITION
// Port 20, X Offset 0, Y Offset -6 (Mounted on back)
// CHECK YOUR PORT AND MOUNTING!
pros::Gps gps_sensor(20, 0.00, -0.152, 180); 

// Note: 'ultrasonic' is already defined in robot_config.cpp

// ==========================================
//          ODOMETRY TASK (2-Wheel)
// ==========================================
void odom_task_fn(void* ignore) {
    // 1. Initial Sensor Calibration
    forward_odom.calibrate();
    heading_odom.calibrate();
    
    double prev_L = 0;
    double prev_R = 0;

    while (true) {
        // 2. Update Encoder Values
        forward_odom.update();
        heading_odom.update();

        double cur_L = forward_odom.get_inches();
        double cur_R = heading_odom.get_inches();

        // Calculate travel distance since last loop
        double dL = cur_L - prev_L;
        double dR = cur_R - prev_R;

        prev_L = cur_L;
        prev_R = cur_R;

        // --- 3. STANDARD 2-WHEEL ENCODER MATH ---
        // Calculate heading change (delta theta) based on wheel difference
        double delta_theta = (dL - dR) / TRACK_WIDTH;

        // Calculate distance traveled by the center of the robot
        double center_dist = (dL + dR) / 2.0;

        // Use the average orientation during the step for higher accuracy
        double avg_theta = robot_theta + (delta_theta / 2.0);

        // 4. Update Position using Mutex for thread safety
        odom_mutex.take();
        
        // Update local belief based on encoder movement
        robot_x += center_dist * std::cos(avg_theta);
        robot_y += center_dist * std::sin(avg_theta);
        robot_theta += delta_theta;

        // --- 5. GPS AUTO-CORRECTION (HEALING) BLOCK ---
        // Use individual getters for PROS 4 compatibility
        // Only run if the toggle is ON and quality is high
        if (useGpsHealing) {
            double statusX = gps_sensor.get_position_x();
            double statusY = gps_sensor.get_position_y();
            
            // Ensure the GPS actually has a lock (returns DBL_MAX if lost)
            if (statusX != DBL_MAX && statusY != DBL_MAX) {
                // Convert GPS Meters to Inches
                double gpsX = statusX * 39.37;
                double gpsY = statusY * 39.37;
                
                // Get Absolute Heading (degrees to radians)
                double gpsH = gps_sensor.get_heading() * (M_PI / 180.0);

                // COMPLEMENTARY FILTER:
                // We slowly pull the current robot position toward the GPS truth.
                // alpha = 0.05 means 5% correction per loop (prevents jumping).
                const double alpha = 0.05;

                robot_x = (robot_x * (1.0 - alpha)) + (gpsX * alpha);
                robot_y = (robot_y * (1.0 - alpha)) + (gpsY * alpha);
                
                // Simple linear interpolation for theta
                robot_theta = (robot_theta * (1.0 - alpha)) + (gpsH * alpha);
            }
        }
        
        odom_mutex.give();

        // 6. Loop Frequency (100Hz)
        pros::delay(10);
    }
}
// ==========================================
//          RESET HELPERS
// ==========================================

// 1. Hard Reset
void resetOdometry() {
    odom_mutex.take();
    forward_odom.reset();
    heading_odom.reset();
    robot_x = 0;
    robot_y = 0;
    robot_theta = 0;
    odom_mutex.give();
    pros::delay(50);
}

// 2. Ultrasonic Wall Reset
void fixPositionWithUltrasonic() {
    // ultrasonic is extern from robot_config.hpp
    int dist_mm = ultrasonic.get_value();
    
    if (dist_mm <= 0 || dist_mm > 1000) return; // Invalid read

    double dist_inches = dist_mm / 25.4;

    odom_mutex.take();
    // Assume we are backing into the "bottom" wall (Y=0)
    // Robot Y = Distance + Offset
    robot_y = dist_inches + ULTRASONIC_OFFSET;
    robot_theta = 0; // Snap angle to 0
    odom_mutex.give();

    pros::lcd::print(0, "US FIX: Y=%.2f", robot_y);
}

// 3. GPS Absolute Fix
void fixPoseWithGPS() {
    // In PROS 4, get_status() might be unavailable. 
    // We use the specific position getters which are more stable.
    double statusX = gps_sensor.get_position_x(); 
    double statusY = gps_sensor.get_position_y();
    double statusH = gps_sensor.get_heading();

    // The GPS returns DBL_MAX if it cannot see the field strips
    if (statusX != DBL_MAX && statusY != DBL_MAX) {
        
        // VEX GPS outputs meters; convert to inches for your odometry
        double gX = statusX * 39.37; 
        double gY = statusY * 39.37;
        
        odom_mutex.take();
        robot_x = gX;
        robot_y = gY;
        // GPS heading is in degrees; convert to radians for your math
        robot_theta = statusH * (M_PI / 180.0);
        odom_mutex.give();
        
        pros::lcd::print(0, "GPS FIX: %.1f, %.1f", gX, gY);
    } else {
        pros::lcd::print(0, "GPS LOST! (Check Strips)");
    }
}

// ==========================================
//          VISUALIZATION
// ==========================================
void resetGraph() {
    graph_x = 0;
    pros::screen::erase();
    // Grid
    pros::screen::set_pen(0x444444);
    pros::screen::draw_line(0, 120, 480, 120); 
}

void drawTargetGraph(double target, double current, double scale) {
    static double prevTarget = 0;
    static double prevCurrent = 0;
    
    if (graph_x >= 480) return;

    int targetY = 120 - (int)(target * scale);
    int currentY = 120 - (int)(current * scale);
    int prevTargetY = 120 - (int)(prevTarget * scale);
    int prevCurrentY = 120 - (int)(prevCurrent * scale);

    auto clamp = [](int v){ return (v < 0) ? 0 : ((v > 239) ? 239 : v); };
    targetY = clamp(targetY); currentY = clamp(currentY);
    prevTargetY = clamp(prevTargetY); prevCurrentY = clamp(prevCurrentY);

    if (graph_x > 0) {
        pros::screen::set_pen(0xFF0000); 
        pros::screen::draw_line(graph_x - 1, prevTargetY, graph_x, targetY);
        pros::screen::set_pen(0x00FF00); 
        pros::screen::draw_line(graph_x - 1, prevCurrentY, graph_x, currentY);
    }
    prevTarget = target;
    prevCurrent = current;
    graph_x++;
}

// ==========================================
//          DEBUG DASHBOARD
// ==========================================
void debug_task_fn(void* ignore) {
    while (true) {
        pros::lcd::print(4, "X:%.1f Y:%.1f A:%.1f", robot_x, robot_y, robot_theta * 180 / M_PI);
        // Display Raw Encoder Values
        pros::lcd::print(5, "L:%.1f R:%.1f", forward_odom.get_inches(), heading_odom.get_inches());
        pros::delay(50);
    }
}

// ==========================================
//          PID: FORWARD (Slew + Drift Fix)
// ==========================================
void driveForwardPID(double targetDistance, double maxSpeed, double fixedHeadingDeg, double timeout) {
    // Auto-Reset for relative moves
    resetOdometry();
    resetGraph();

    double kP = forwardPID_Consts.kP;
    double kI = forwardPID_Consts.kI;
    double kD = forwardPID_Consts.kD;

    double error = 0, prevError = 0, integral = 0, derivative = 0;
    
    // SLEW RATE (Soft Start)
    double appliedPower = 0; 
    double slewStep = 8.0; 

    uint32_t startTime = pros::millis();

    while (pros::millis() - startTime < timeout) {
        // Distance is Average of Left and Right
        double currentDist = (forward_odom.get_inches() + heading_odom.get_inches()) / 2.0;
        
        error = targetDistance - currentDist;
        drawTargetGraph(targetDistance, currentDist, 2.5);

        // PID
        if (std::abs(error) < 4.0) integral += error; else integral = 0;
        if ((error > 0 && prevError < 0) || (error < 0 && prevError > 0)) integral = 0;
        
        derivative = error - prevError;
        double targetPower = (error * kP) + (integral * kI) + (derivative * kD);

        // --- SLEW RATE LIMITER ---
        if (targetPower > appliedPower + slewStep) appliedPower += slewStep;
        else if (targetPower < appliedPower - slewStep) appliedPower -= slewStep;
        else appliedPower = targetPower;

        appliedPower = std::clamp(appliedPower, -maxSpeed, maxSpeed);

        // --- HEADING CORRECTION ---
        // Keep robot straight (at 0)
        double headingCorrection = (0 - robot_theta) * (180.0 / M_PI) * 2.5;

        left_motor_group.move_velocity(appliedPower + headingCorrection);
        right_motor_group.move_velocity(appliedPower - headingCorrection);

        if (std::abs(error) < 0.5 && std::abs(derivative) < 0.1) break;

        prevError = error;
        pros::delay(20);
    }
    
    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);
    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

// ==========================================
//          PID: TURN (Slew Rate)
// ==========================================
void turnToAnglePID(double targetAngleDeg, double maxSpeed, double timeout) {
    resetOdometry();
    resetGraph();
    
    double kP = turnPID_Consts.kP;
    double kI = turnPID_Consts.kI;
    double kD = turnPID_Consts.kD;

    double targetRad = targetAngleDeg * (M_PI / 180.0);
    double error = 0, prevError = 0, integral = 0, derivative = 0;
    
    double appliedPower = 0;
    double slewStep = 10.0;

    uint32_t startTime = pros::millis();

    while (pros::millis() - startTime < timeout) {
        double errorRad = targetRad - robot_theta;
        double errorDeg = errorRad * (180.0 / M_PI);
        
        drawTargetGraph(targetAngleDeg, robot_theta * (180.0/M_PI), 1.0);

        if (std::abs(errorDeg) < 10.0) integral += errorDeg; else integral = 0;
        derivative = errorDeg - prevError;

        double targetPower = (errorDeg * kP) + (integral * kI) + (derivative * kD);
        
        // Slew Rate
        if (targetPower > appliedPower + slewStep) appliedPower += slewStep;
        else if (targetPower < appliedPower - slewStep) appliedPower -= slewStep;
        else appliedPower = targetPower;

        appliedPower = std::clamp(appliedPower, -maxSpeed, maxSpeed);

        left_motor_group.move_velocity(appliedPower);
        right_motor_group.move_velocity(-appliedPower);

        if (std::abs(errorDeg) < 0.5 && std::abs(derivative) < 0.1) break;

        prevError = errorDeg;
        pros::delay(20);
    }

    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);
}

// ==========================================
//          PID: REVERSE
// ==========================================
void driveReversePID(double targetDistance, double maxSpeed, double timeout) {
    driveForwardPID(-targetDistance, maxSpeed, 0, timeout);
}

// ==========================================
//          TUNING & CONTROL LOOP
// ==========================================
void tuningLoop() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::lcd::initialize();
    resetGraph(); 
    resetOdometry();

    const char* modeNames[] = { "Dr 24", "Dr 48", "Tn 90", "Tn 180" };
    int currentParam = 0; // 0=P, 1=I, 2=D
    double step = 0.1;

    while (true) {
        bool isTurn = (testMode >= 2);
        PIDConfig* cfg = isTurn ? &turnPID_Consts : &forwardPID_Consts;

        const char* pMarker = (currentParam == 0) ? ">P" : " P";
        const char* iMarker = (currentParam == 1) ? ">I" : " I";
        const char* dMarker = (currentParam == 2) ? ">D" : " D";

        master.print(0, 0, "M:%s %s:%.2f", modeNames[testMode], pMarker, cfg->kP);
        pros::delay(50);
        master.print(1, 0, "%s:%.3f %s:%.2f", iMarker, cfg->kI, dMarker, cfg->kD);
        pros::delay(50);
        master.print(2, 0, "St:%.1f Y:%.1f", step, robot_y);

        // --- INPUTS ---
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            testMode++; if(testMode > 3) testMode = 0;
        }
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            currentParam++; if(currentParam > 2) currentParam = 0;
        }
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) step *= 10;
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) step /= 10;

        double change = 0;
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) change = step;
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) change = -step;

        if (currentParam == 0) cfg->kP += change;
        else if (currentParam == 1) cfg->kI += change;
        else if (currentParam == 2) cfg->kD += change;

        // --- ACTIONS ---

        // A: RUN TEST (Auto Resets)
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            if (testMode == 0) driveForwardPID(24, 80, 0, 2000);
            if (testMode == 1) driveForwardPID(48, 80, 0, 3000);
            if (testMode == 2) turnToAnglePID(90, 60, 2000);
            if (testMode == 3) turnToAnglePID(180, 60, 2500);
        }

        // Y: ULTRASONIC FIX (When near wall)
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            fixPositionWithUltrasonic();
            master.rumble("-");
        }

        // Left + B: GPS FIX
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && 
            master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            fixPoseWithGPS();
            master.rumble(".");
        }
        
        // B: MANUAL RESET
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            resetOdometry();
        }

        pros::delay(20);
    }
}