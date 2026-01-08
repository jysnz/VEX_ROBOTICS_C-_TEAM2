#include "odometry.hpp"
#include <cmath>
#include "robot_config.hpp"

// ==========================================
//          CONFIGURATION
// ==========================================
const double WHEEL_DIAMETER = 3.05;
const double FORWARD_OFFSET = -0.66; // Left Vertical Wheel
const double HEADING_OFFSET = 1.64;  // Right Vertical Wheel
const double SIDEWAYS_OFFSET = 6.55; // Horizontal Wheel

// TUNE THIS MANUALLY using the "Spin Test"
double TRACK_WIDTH = 14.75; 
int graph_x = 0;

// ==========================================
//          GLOBAL VARIABLES
// ==========================================
double robot_x = 0;
double robot_y = 0;
double robot_theta = 0; // In Radians

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

    if (std::abs(delta) < 5) return; // DEADBAND

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
    filtered_val = (alpha * raw_inches) + ((1.0 - alpha) * filtered_val);
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
AS5600 forward_odom('C', true); 
AS5600 heading_odom('B', false);  
AS5600 sideways_odom('A', true);

// ==========================================
//          ODOMETRY BACKGROUND TASK
// ==========================================
void odom_task_fn(void* ignore) {
    double prev_F = 0, prev_H = 0, prev_S = 0;

    while (true) {
        forward_odom.update();
        heading_odom.update();
        sideways_odom.update();

        double cur_F = forward_odom.get_inches();
        double cur_H = heading_odom.get_inches();
        double cur_S = sideways_odom.get_inches();

        double dF = cur_F - prev_F;
        double dH = cur_H - prev_H;
        double dS = cur_S - prev_S;

        prev_F = cur_F; prev_H = cur_H; prev_S = cur_S;

        double delta_theta = (dF - dH) / TRACK_WIDTH;
        double center_dist = (dF + dH) / 2.0; 

        double local_x, local_y;
        if (delta_theta == 0) {
            local_y = center_dist; 
            local_x = dS;
        } else {
            local_y = 2 * std::sin(delta_theta / 2.0) * (center_dist / delta_theta);
            local_x = 2 * std::sin(delta_theta / 2.0) * ((dS / delta_theta) + SIDEWAYS_OFFSET);
        }

        double avg_theta = robot_theta + (delta_theta / 2.0);
        robot_x += local_y * std::sin(avg_theta) + local_x * std::cos(avg_theta);
        robot_y += local_y * std::cos(avg_theta) - local_x * std::sin(avg_theta);
        robot_theta += delta_theta;

        pros::delay(10); 
    }
}

void resetGraph() {
    graph_x = 0;
    pros::screen::erase();
    
    // Draw Background Grid
    pros::screen::set_pen(0x444444); // Dark Gray
    // Draw Horizontal Grid Lines (Center, Top, Bottom)
    pros::screen::draw_line(0, 120, 480, 120); // 0 (Center)
    pros::screen::draw_line(0, 60,  480, 60);  // Positive
    pros::screen::draw_line(0, 180, 480, 180); // Negative
    
    // Draw Labels (Optional)
    pros::screen::set_pen(0xFFFFFF);
    pros::screen::print(pros::E_TEXT_SMALL, 5, 55, "Pos");
    pros::screen::print(pros::E_TEXT_SMALL, 5, 115, "0");
    pros::screen::print(pros::E_TEXT_SMALL, 5, 175, "Neg");
}

// ==========================================
//          DEBUG DASHBOARD
// ==========================================
void debug_task_fn(void* ignore) {
    while (true) {
        pros::lcd::print(0, "X: %.2f  Y: %.2f  Ang: %.1f", robot_x, robot_y, robot_theta * 180 / M_PI);
        pros::lcd::print(1, "L:%d  R:%d  S:%d", forward_odom.get_raw(), heading_odom.get_raw(), sideways_odom.get_raw());
        pros::lcd::print(2, "L:%.1f  R:%.1f  S:%.1f", forward_odom.get_inches(), heading_odom.get_inches(), sideways_odom.get_inches());
        pros::delay(50);
    }
}

void drawTargetGraph(double target, double current, double scale) {
    // Static variables to connect lines from previous frame
    static double prevTarget = 0;
    static double prevCurrent = 0;
    
    // Prevent drawing off screen
    if (graph_x >= 480) return;

    // --- Calculate Y Positions ---
    // Screen (0,0) is Top-Left. 
    // We want 120 to be the center.
    // Value * Scale determines height. Subtracted from 120 to flip "Up".
    int targetY = 120 - (int)(target * scale);
    int currentY = 120 - (int)(current * scale);
    int prevTargetY = 120 - (int)(prevTarget * scale);
    int prevCurrentY = 120 - (int)(prevCurrent * scale);

    // --- Clamp to Screen Limits ---
    auto clamp = [](int v){ return (v < 0) ? 0 : ((v > 239) ? 239 : v); };
    targetY = clamp(targetY); currentY = clamp(currentY);
    prevTargetY = clamp(prevTargetY); prevCurrentY = clamp(prevCurrentY);

    // --- Draw Lines ---
    if (graph_x > 0) {
        // RED Line = Target (The "Step")
        pros::screen::set_pen(0xFF0000); 
        pros::screen::draw_line(graph_x - 1, prevTargetY, graph_x, targetY);

        // GREEN Line = Actual (The "Curve")
        pros::screen::set_pen(0x00FF00); 
        pros::screen::draw_line(graph_x - 1, prevCurrentY, graph_x, currentY);
    }

    // Save state for next loop
    prevTarget = target;
    prevCurrent = current;
    graph_x++;
}

// ==========================================
//          PID: DRIVE REVERSE (Separate)
// ==========================================
void driveReversePID(double targetDistance, double maxSpeed, double timeout) {
    // --- SEPARATE TUNING CONSTANTS ---
    double kP = 5.0; 
    double kI = 0.015; 
    double kD = 3.26; 
    
    // TRICK: Reverse usually needs STRONGER heading correction to stay straight
    // Try increasing this if it still drifts (e.g., 15.0 or 20.0)
    double kP_Heading = 15.0; 

    double startI = 3.0; 

    // Setup
    double startX = robot_x; 
    double startY = robot_y;
    double targetTheta = robot_theta;
    double error = 0, prevError = 0, integral = 0, derivative = 0;
    
    // FORCE TARGET NEGATIVE
    // Even if you type "24", we treat it as "-24"
    double absTarget = -std::abs(targetDistance);

    double graphScale = 2.5; 

    uint32_t startTime = pros::millis();

    while (pros::millis() - startTime < timeout) {
        // 1. Calculate Distance
        double distTraveled = std::hypot(robot_x - startX, robot_y - startY);
        
        // Since we are reversing, traveled distance is negative relative to start
        double currentPos = -distTraveled;
        
        error = absTarget - currentPos;

        // Update Graph (Draws the step down)
        drawTargetGraph(absTarget, currentPos, graphScale); 

        // 2. Linear PID
        if (std::abs(error) < startI) integral += error; else integral = 0;
        if ((error > 0 && prevError < 0) || (error < 0 && prevError > 0)) integral = 0;
        // Anti-Drift
        if (std::abs(error) < 0.5) integral = 0;
        
        derivative = error - prevError;
        double masterPower = (error * kP) + (integral * kI) + (derivative * kD);

        // Min Power Boost for Reverse
        if (std::abs(masterPower) < 10 && std::abs(error) > 1.0) { 
            masterPower = (masterPower > 0) ? 10 : -10;
        }

        // Cap speed
        if (masterPower > maxSpeed) masterPower = maxSpeed;
        if (masterPower < -maxSpeed) masterPower = -maxSpeed;
        
        // 3. Heading Correction
        double headingError = targetTheta - robot_theta;
        while (headingError > M_PI) headingError -= 2 * M_PI;
        while (headingError < -M_PI) headingError += 2 * M_PI;
        double currentHeadingKP = kP_Heading;   
        double turnCorrection = headingError * currentHeadingKP;

        // --- CRITICAL REVERSE LOGIC ---
        // We MUST invert the turn correction when driving backward
        // Left side needs to slow down (add positive) to turn Left while reversing
        turnCorrection = -turnCorrection;

        left_motor_group.move_velocity(masterPower + turnCorrection);
        right_motor_group.move_velocity(masterPower - turnCorrection);

        // 4. Exit Conditions
        if (std::abs(error) < 0.5 && std::abs(derivative) < 0.5) break;

        prevError = error;
        pros::delay(20);
    }
    
    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

// ==========================================
//          PID: DRIVE FORWARD
// ==========================================

//KP = 4.0
//KI = 0.0
//KD = 0.79
void driveForwardPID(double targetDistance, double maxSpeed, double timeout) {
    double kP = 4.0; 
    double kI = 0.0; 
    double kD = 0.91; 
    double kP_Heading = 10.0; 
    double startI = 3.0; 

    // Setup
    double startX = robot_x; 
    double startY = robot_y;
    double targetTheta = robot_theta;
    double error = 0, prevError = 0, integral = 0, derivative = 0;
    
    // Graphing
    double graphScale = 2.5; 

    uint32_t startTime = pros::millis();

    while (pros::millis() - startTime < timeout) {
        // 1. Calculate Distance
        double distTraveled = std::hypot(robot_x - startX, robot_y - startY);
        if (targetDistance < 0) distTraveled = -distTraveled;
        
        error = targetDistance - distTraveled;

        // Update Graph
        drawTargetGraph(targetDistance, distTraveled, graphScale); 

        // 2. PID Calculations
        if (std::abs(error) < startI) integral += error; else integral = 0;
        if ((error > 0 && prevError < 0) || (error < 0 && prevError > 0)) integral = 0;
        // Anti-Drift: Kill integral if VERY close
        if (std::abs(error) < 0.5) integral = 0;

        derivative = error - prevError;
        double masterPower = (error * kP) + (integral * kI) + (derivative * kD);

        // Min Power Boost
        if (std::abs(masterPower) < 10 && std::abs(error) > 1.0) { 
             masterPower = (masterPower > 0) ? 10 : -10;
        }

        // Cap speed
        if (masterPower > maxSpeed) masterPower = maxSpeed;
        if (masterPower < -maxSpeed) masterPower = -maxSpeed;
        
        // 3. Heading Correction
        double headingError = targetTheta - robot_theta;
        while (headingError > M_PI) headingError -= 2 * M_PI;
        while (headingError < -M_PI) headingError += 2 * M_PI;
        
        double currentHeadingKP = (std::abs(error) < 2.0) ? 2.0 : kP_Heading;
        double turnCorrection = headingError * currentHeadingKP;

        // --- THE FIX IS HERE ---
        // If we are reversing, we must invert the steering direction
        if (targetDistance < 0) {
            turnCorrection = -turnCorrection;
        }

        left_motor_group.move_velocity(masterPower + turnCorrection);
        right_motor_group.move_velocity(masterPower - turnCorrection);

        // 4. Exit Conditions
        // Wait until error is small AND speed (derivative) is near zero
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
//          PID: TURN TO ANGLE
// ==========================================
void turnToAnglePID(double targetAngleDeg, double maxSpeed, double timeout) {
    // --- Config ---
    double kP = 4.0; double kI = 0.05; double kD = 8.5;
    
    // Graphing Scale: 90 degrees = ~60 pixels height (Zoomed out compared to drive)
    double graphScale = 0.8; 

    // --- Setup ---
    double targetRad = targetAngleDeg * (M_PI / 180.0);
    double error = 0, prevError = 0, integral = 0, derivative = 0;
    
    // We need to track "relative" angle for the graph to look nice (starting at 0)
    // OR we can plot absolute. Let's plot Error-based Target vs Actual.
    // Target = Angle desired, Current = Angle moved.
    double startTheta = robot_theta;

    uint32_t startTime = pros::millis();

    while (pros::millis() - startTime < timeout) {
        error = targetRad - robot_theta;
        while (error > M_PI) error -= 2 * M_PI;
        while (error < -M_PI) error += 2 * M_PI;

        // For Graphing: We want to show 0 -> 90
        // Current Angle relative to start of turn:
        double currentRelDeg = (robot_theta - startTheta) * (180.0 / M_PI);
        // Target Angle relative to start of turn:
        double targetRelDeg = (targetRad - startTheta) * (180.0 / M_PI);
        
        // --- UPDATE GRAPH ---
        drawTargetGraph(targetRelDeg, currentRelDeg, graphScale);

        // PID Calc
        if (std::abs(error) < (10.0 * M_PI/180)) integral += error; else integral = 0;
        if ((error > 0 && prevError < 0) || (error < 0 && prevError > 0)) integral = 0;
        derivative = error - prevError;

        double power = (error * kP) + (integral * kI) + (derivative * kD);
        
        if (power > maxSpeed) power = maxSpeed;
        if (power < -maxSpeed) power = -maxSpeed;

        left_motor_group.move_velocity(power);
        right_motor_group.move_velocity(-power);

        if (std::abs(error) < (1.0 * M_PI/180) && std::abs(derivative) < 0.05) break;

        prevError = error;
        pros::delay(20);
    }
     left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}