#include "odometry.hpp"
#include <cmath>
#include "robot_config.hpp"

// ==========================================
//          CONFIGURATION
// ==========================================
const double WHEEL_DIAMETER = 3.25;
const double FORWARD_OFFSET = -0.66; // Left Vertical Wheel
const double HEADING_OFFSET = 1.64;  // Right Vertical Wheel
const double SIDEWAYS_OFFSET = 6.55; // Horizontal Wheel

// TUNE THIS MANUALLY using the "Spin Test"
double TRACK_WIDTH = 14.75; //std::abs(FORWARD_OFFSET - HEADING_OFFSET);

// ==========================================
//          GLOBAL VARIABLES
// ==========================================
double robot_x = 0;
double robot_y = 0;
double robot_theta = 0; // In Radians

// ==========================================
//          AS5600 IMPLEMENTATION
// ==========================================

// 1. Constructor: NO DELAYS HERE. Only simple variable setup.
AS5600::AS5600(char port, bool is_reversed) : sensor(port), reversed(is_reversed) {
    last_raw = 0;
    total_ticks = 0;
    filtered_val = 0;
}

// 2. Calibrate: Call this in initialize() to safely stabilize sensors
void AS5600::calibrate() {
    last_raw = sensor.get_value();
    pros::delay(20); // Safe to delay here because OS is running
    last_raw = sensor.get_value();
    reset();
}

int AS5600::get_raw() {
    return sensor.get_value();
}

void AS5600::update() {
    int current_raw = sensor.get_value();
    
    // Calculate the change
    int delta = current_raw - last_raw;

    // --- 1. DEADBAND (Fixes Ghost Movement) ---
    // If the change is tiny (noise), ignore it completely.
    if (std::abs(delta) < 5) { 
        // Do NOT update last_raw, keep waiting for real movement
        return; 
    }

    // --- 2. WRAP LOGIC (Fixes Sawtooth) ---
    // If delta is huge (> 2048), it means we crossed the 0/4095 line.
    if (delta > 2048) {
        delta -= 4096; // We wrapped backward
    } 
    else if (delta < -2048) {
        delta += 4096; // We wrapped forward
    }

    // --- 3. ACCUMULATE ---
    if (reversed) {
        total_ticks -= delta;
    } else {
        total_ticks += delta;
    }

    // Only update this if we actually accepted the movement
    last_raw = current_raw;
}

// 4. Get Inches (with filter)
double AS5600::get_inches() {
    double raw_inches = (total_ticks / 4096.0) * (WHEEL_DIAMETER * M_PI);
    // Low Pass Filter
    filtered_val = (alpha * raw_inches) + ((1.0 - alpha) * filtered_val);
    return filtered_val;
}

// 5. Reset
void AS5600::reset() {
    total_ticks = 0;
    filtered_val = 0;
    last_raw = sensor.get_value();
}

// ==========================================
//          SENSOR OBJECTS
// ==========================================   
// PUSH TEST: Push forward. If L/R goes negative, change false->true.
AS5600 forward_odom('C', true); 
AS5600 heading_odom('B', false);  
AS5600 sideways_odom('A', true);

// ==========================================
//          ODOMETRY BACKGROUND TASK
// ==========================================
void odom_task_fn(void* ignore) {
    double prev_F = 0, prev_H = 0, prev_S = 0;

    while (true) {
        // Update Sensors
        forward_odom.update();
        heading_odom.update();
        sideways_odom.update();

        // Get current values
        double cur_F = forward_odom.get_inches();
        double cur_H = heading_odom.get_inches();
        double cur_S = sideways_odom.get_inches();

        // Calculate Deltas
        double dF = cur_F - prev_F;
        double dH = cur_H - prev_H;
        double dS = cur_S - prev_S;

        prev_F = cur_F; prev_H = cur_H; prev_S = cur_S;

        // Calculate Heading Change
        // This relies on both wheels, so they must agree on Track Width
        double delta_theta = (dF - dH) / TRACK_WIDTH;

        // --- THE FIX FOR "FIGHTING" WHEELS ---
        // Instead of using just dF, we use the average of dF and dH.
        // This represents the physical center of the robot.
        double center_dist = (dF + dH) / 2.0; 

        // Calculate Local Position
        double local_x, local_y;

        if (delta_theta == 0) {
            local_y = center_dist; 
            local_x = dS;
        } else {
            // ARC CALCULATION
            // We use center_dist directly. No FORWARD_OFFSET needed here 
            // because center_dist is ALREADY at the center!
            local_y = 2 * std::sin(delta_theta / 2.0) * (center_dist / delta_theta);

            // Sideways still needs the offset because the back wheel isn't at the center
            local_x = 2 * std::sin(delta_theta / 2.0) * ((dS / delta_theta) + SIDEWAYS_OFFSET);
        }
        // -------------------------------------

        // Rotate to Global
        double avg_theta = robot_theta + (delta_theta / 2.0);
        robot_x += local_y * std::sin(avg_theta) + local_x * std::cos(avg_theta);
        robot_y += local_y * std::cos(avg_theta) - local_x * std::sin(avg_theta);
        robot_theta += delta_theta;

        pros::delay(10); 
    }
}

// ==========================================
//          DEBUG DASHBOARD
// ==========================================
void debug_task_fn(void* ignore) {
    while (true) {
        // Line 0: Global Position
        pros::lcd::print(0, "X: %.2f  Y: %.2f  Ang: %.1f", 
            robot_x, robot_y, robot_theta * 180 / M_PI);
        
        // Line 1: RAW TICKS for Left(C), Right(B), Sideways(A)
        // Now you can see if B is actually counting!
        pros::lcd::print(1, "L:%d  R:%d  S:%d", 
            forward_odom.get_raw(), 
            heading_odom.get_raw(), 
            sideways_odom.get_raw()
        );
        
        // Line 2: INCHES for Left, Right, Sideways
        pros::lcd::print(2, "L:%.1f  R:%.1f  S:%.1f", 
            forward_odom.get_inches(), 
            heading_odom.get_inches(), 
            sideways_odom.get_inches()
        );

        pros::delay(50);
    }
}

// ==========================================
//          DEBUG GRAPH
// ==========================================
void drawPIDGraph(double error, int timeStep, bool isTurn) {
    int centerY = 120;
    double scale = isTurn ? 3.0 : 40.0; 
    double value = isTurn ? (error * 180.0 / M_PI) : error;

    int y = centerY - (int)(value * scale);
    if (y < 0) y = 0; if (y > 239) y = 239;
    int x = timeStep % 480;

    if (x == 0) { 
        pros::screen::erase(); 
        pros::screen::set_pen(0x444444); 
        pros::screen::draw_line(0, centerY, 480, centerY); 
    }
    
    pros::screen::set_pen(std::abs(value) < (isTurn ? 1.0 : 0.5) ? 0x00FF00 : 0xFF0000);
    pros::screen::draw_pixel(x, y);
}

// ==========================================
//          PID: DRIVE FORWARD
// ==========================================
// ==========================================
//          PID: DRIVE FORWARD (WITH GRAPH)
// ==========================================
void driveForwardPID(double targetDistance, double maxSpeed, double timeout) {
    // 1. PID Constants
    double kP = 8.0; 
    double kI = 0.0; // Keep 0.0 unless you really need it
    double kD = 15;
    double kP_Heading = 10.0; 
    double startI = 3.0; 

    // 2. Variables
    double error = 0, prevError = 0, integral = 0, derivative = 0;
    double startX = robot_x; 
    double startY = robot_y;
    double targetTheta = robot_theta; 
    
    int settleTimer = 0; 
    int timeStep = 0; // <--- NEEDED FOR GRAPHING

    uint32_t startTime = pros::millis();

    // 3. The Loop
    while (pros::millis() - startTime < timeout) {
        // --- Calculate Distance Error ---
        double distTraveled = std::hypot(robot_x - startX, robot_y - startY);
        if (targetDistance < 0) distTraveled = -distTraveled;
        error = targetDistance - distTraveled;

        // --- Calculate Heading Error ---
        double headingError = targetTheta - robot_theta;
        while (headingError > M_PI) headingError -= 2 * M_PI;
        while (headingError < -M_PI) headingError += 2 * M_PI;
        
        // Relax heading correction when close to target to stop "Twisting"
        double currentHeadingKP = (std::abs(error) < 2.0) ? 2.0 : kP_Heading;
        double turnCorrection = headingError * currentHeadingKP;

        // --- PID Calculations ---
        if (std::abs(error) < startI) integral += error; else integral = 0;
        if ((error > 0 && prevError < 0) || (error < 0 && prevError > 0)) integral = 0;
        
        // ** ANTI-DRIFT FIX **
        // If extremely close, kill integral to prevent sliding past target
        if (std::abs(error) < 0.5) integral = 0;

        derivative = error - prevError;

        double masterPower = (error * kP) + (integral * kI) + (derivative * kD);

        // --- Power Management ---
        // Minimum power boost (Removed when very close to target)
        if (std::abs(masterPower) < 10 && std::abs(error) > 1.0) { 
             masterPower = (masterPower > 0) ? 10 : -10;
        }

        // Cap speed
        if (masterPower > maxSpeed) masterPower = maxSpeed;
        if (masterPower < -maxSpeed) masterPower = -maxSpeed;
        
        // Move Motors
        left_motor_group.move_velocity(masterPower + turnCorrection);
        right_motor_group.move_velocity(masterPower - turnCorrection);

        // --- DRAW THE GRAPH ---
        // 'false' means we are graphing Distance, not Turning
        drawPIDGraph(error, timeStep++, false); 

        // --- Exit Conditions ---
        if (std::abs(error) < 0.5) {
            settleTimer += 20; 
        } else {
            settleTimer = 0; 
        }

        if (settleTimer > 100) break;

        prevError = error;
        pros::delay(20);
    }
    
    // Stop and Brake
    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);
    left_motor_group.brake();
    right_motor_group.brake();
}

// ==========================================
//          PID: TURN TO ANGLE
// ==========================================
void turnToAnglePID(double targetAngleDeg, double maxSpeed, double timeout) {
    double kP = 4.0; double kI = 0.05; double kD = 8.5;
    double startI = 15.0; 
    double error = 0, prevError = 0, integral = 0, derivative = 0;
    double targetRad = targetAngleDeg * (M_PI / 180.0);
    int timeStep = 0;
    uint32_t startTime = pros::millis();

    while (pros::millis() - startTime < timeout) {
        error = targetRad - robot_theta;
        while (error > M_PI) error -= 2 * M_PI;
        while (error < -M_PI) error += 2 * M_PI;

        if (std::abs(error) < (startI * M_PI/180.0)) integral += error; else integral = 0;
        if ((error > 0 && prevError < 0) || (error < 0 && prevError > 0)) integral = 0;
        derivative = error - prevError;

        double power = (error * kP) + (integral * kI) + (derivative * kD);
        if (power > maxSpeed) power = maxSpeed;
        if (power < -maxSpeed) power = -maxSpeed;

        left_motor_group.move_velocity(power);
        right_motor_group.move_velocity(-power);

        drawPIDGraph(error, timeStep++, true);
        if (std::abs(error) < (1.0 * M_PI/180.0) && std::abs(derivative) < 0.05) break;
        prevError = error;
        pros::delay(20);
    }
    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);
}

