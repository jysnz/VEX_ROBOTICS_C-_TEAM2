#include "odometry.hpp"
#include <cmath>
#include <algorithm> // For std::clamp
#include "robot_config.hpp" // Ensure this exists for motor groups

// ==========================================
//          CONFIGURATION CONSTANTS
// ==========================================
const double WHEEL_DIAMETER = 3.05;
const double FORWARD_OFFSET = -3.22; 
const double HEADING_OFFSET = 2.72;  
const double SIDEWAYS_OFFSET = 6.55; 

// TUNE THIS MANUALLY using the "Spin Test"
double TRACK_WIDTH = 5.94;

// ==========================================
//          GLOBAL VARIABLES
// ==========================================
double robot_x = 0;
double robot_y = 0;
double robot_theta = 0; // Radians

// Tuning Graph Helper
int graph_x = 0;
int testMode = 0;

// PID Defaults (Starting Values)
PIDConfig forwardPID_Consts = { 6.00, 0.001, 2.00 };
PIDConfig turnPID_Consts    = { 1.20, 0.010,  10.20};

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
//          ODOMETRY TASK
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

// ==========================================
//          VISUALIZATION
// ==========================================
void resetGraph() {
    graph_x = 0;
    pros::screen::erase();
    
    // Draw Background Grid
    pros::screen::set_pen(0x444444); // Dark Gray
    pros::screen::draw_line(0, 120, 480, 120); // Center
    pros::screen::draw_line(0, 60,  480, 60);  // Upper
    pros::screen::draw_line(0, 180, 480, 180); // Lower
}

void drawTargetGraph(double target, double current, double scale) {
    static double prevTarget = 0;
    static double prevCurrent = 0;
    
    if (graph_x >= 480) return;

    // Calculate Y positions (Inverted because screen 0 is top)
    int targetY = 120 - (int)(target * scale);
    int currentY = 120 - (int)(current * scale);
    int prevTargetY = 120 - (int)(prevTarget * scale);
    int prevCurrentY = 120 - (int)(prevCurrent * scale);

    // Clamp
    auto clamp = [](int v){ return (v < 0) ? 0 : ((v > 239) ? 239 : v); };
    targetY = clamp(targetY); currentY = clamp(currentY);
    prevTargetY = clamp(prevTargetY); prevCurrentY = clamp(prevCurrentY);

    if (graph_x > 0) {
        pros::screen::set_pen(0xFF0000); // RED = Target
        pros::screen::draw_line(graph_x - 1, prevTargetY, graph_x, targetY);

        pros::screen::set_pen(0x00FF00); // GREEN = Actual
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
        pros::lcd::print(5, "L:%.1f R:%.1f S:%.1f", forward_odom.get_inches(), heading_odom.get_inches(), sideways_odom.get_inches());
        pros::delay(50);
    }
}

// ==========================================
//          PID: FORWARD (Adjustable)
// ==========================================
void driveForwardPID(double targetDistance, double maxSpeed, double fixedHeadingDeg, double timeout) {
    // USE GLOBAL CONFIGS
    double kP = forwardPID_Consts.kP;
    double kI = forwardPID_Consts.kI;
    double kD = forwardPID_Consts.kD;

    double startHeadingKP = 10.0;  
    double endHeadingKP = 0.0;    
    double fadeDistance = 5.0;    
    double finalParkingKP = 2.0;   
    
    double startI = 3.0; 
    double accelStep = 6.0;      
    double appliedPower = 0.0;   

    double startX = robot_x; 
    double startY = robot_y;
    double targetTheta = fixedHeadingDeg * (M_PI / 180.0);

    double error = 0, prevError = 0, integral = 0, derivative = 0;
    double graphScale = 2.5; 

    uint32_t startTime = pros::millis();

    while (pros::millis() - startTime < timeout) {
        double distTraveled = std::hypot(robot_x - startX, robot_y - startY);
        if (targetDistance < 0) distTraveled = -distTraveled;
        error = targetDistance - distTraveled;

        drawTargetGraph(targetDistance, distTraveled, graphScale);

        if (std::abs(error) < startI) integral += error; else integral = 0;
        if ((error > 0 && prevError < 0) || (error < 0 && prevError > 0)) integral = 0;
        if (std::abs(error) < 0.5) integral = 0;

        derivative = error - prevError;
        double targetPower = (error * kP) + (integral * kI) + (derivative * kD);

        // Boost min power
        if (std::abs(targetPower) < 10 && std::abs(error) > 1.0)
            targetPower = (targetPower > 0) ? 10 : -10;

        targetPower = std::clamp(targetPower, -maxSpeed, maxSpeed);

        // Slew
        if (targetPower > appliedPower + accelStep) appliedPower += accelStep;
        else if (targetPower < appliedPower - accelStep) appliedPower -= accelStep;
        else appliedPower = targetPower;

        // Heading
        double headingError = targetTheta - robot_theta;
        while (headingError > M_PI) headingError -= 2 * M_PI;
        while (headingError < -M_PI) headingError += 2 * M_PI;

        double currentHeadingKP = 0.0;
        if (std::abs(error) < 2.0) currentHeadingKP = finalParkingKP;
        else if (std::abs(distTraveled) < fadeDistance) {
            double progress = std::abs(distTraveled) / fadeDistance;
            currentHeadingKP = startHeadingKP - (progress * (startHeadingKP - endHeadingKP));
        } else {
            currentHeadingKP = endHeadingKP;
        }

        double turnCorrection = headingError * currentHeadingKP;
        if (targetDistance < 0) turnCorrection = -turnCorrection;

        left_motor_group.move_velocity(appliedPower + turnCorrection);
        right_motor_group.move_velocity(appliedPower - turnCorrection);

        if (std::abs(error) < 0.5 && std::abs(derivative) < 0.1) break;

        prevError = error;
        pros::delay(20);
    }
    
    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);
}

// ==========================================
//          PID: TURN (Adjustable)
// ==========================================
void turnToAnglePID(double targetAngleDeg, double maxSpeed, double timeout) {
    resetGraph();
    
    // USE GLOBAL CONFIGS
    double kP = turnPID_Consts.kP;
    double kI = turnPID_Consts.kI;
    double kD = turnPID_Consts.kD;

    double graphScale = 0.8; 

    double targetRad = targetAngleDeg * (M_PI / 180.0);
    double startTheta = robot_theta;
    uint32_t startTime = pros::millis();

    double error = 0, prevError = 0, integral = 0, derivative = 0;
    
    // Init prevError for derivative kick
    double initialErrorRad = targetRad - robot_theta;
    while (initialErrorRad > M_PI) initialErrorRad -= 2 * M_PI;
    while (initialErrorRad < -M_PI) initialErrorRad += 2 * M_PI;
    prevError = initialErrorRad * (180.0 / M_PI);

    while (pros::millis() - startTime < timeout) {
        double errorRad = targetRad - robot_theta;
        while (errorRad > M_PI) errorRad -= 2 * M_PI;
        while (errorRad < -M_PI) errorRad += 2 * M_PI;

        double errorDeg = errorRad * (180.0 / M_PI);
        
        // Graphing (Relative to start)
        double currentRelDeg = (robot_theta - startTheta) * (180.0 / M_PI);
        double targetRelDeg = (targetRad - startTheta) * (180.0 / M_PI);
        drawTargetGraph(targetRelDeg, currentRelDeg, graphScale);

        if (std::abs(errorDeg) < 10.0) integral += errorDeg; 
        else integral = 0;
        
        if ((errorDeg > 0 && prevError < 0) || (errorDeg < 0 && prevError > 0)) integral = 0;
        
        derivative = errorDeg - prevError;

        double power = (errorDeg * kP) + (integral * kI) + (derivative * kD);
        
        if (power > maxSpeed) power = maxSpeed;
        if (power < -maxSpeed) power = -maxSpeed;

        left_motor_group.move_velocity(power);
        right_motor_group.move_velocity(-power);

        if (std::abs(errorDeg) < 1.0 && std::abs(derivative) < 0.1) break;

        prevError = errorDeg;
        pros::delay(20);
    }

    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);
    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

// ==========================================
//          PID: REVERSE (Fixed Constants)
// ==========================================
void driveReversePID(double targetDistance, double maxSpeed, double timeout) {
    // You can also change these to use global constants if you want
    double kP = 5.0; 
    double kI = 0.015; 
    double kD = 3.26; 
    double kP_Heading = 15.0; 

    double startI = 3.0; 
    double startX = robot_x; 
    double startY = robot_y;
    double targetTheta = robot_theta;
    double error = 0, prevError = 0, integral = 0, derivative = 0;
    
    double absTarget = -std::abs(targetDistance);
    double graphScale = 2.5; 

    uint32_t startTime = pros::millis();

    while (pros::millis() - startTime < timeout) {
        double distTraveled = std::hypot(robot_x - startX, robot_y - startY);
        double currentPos = -distTraveled;
        error = absTarget - currentPos;

        drawTargetGraph(absTarget, currentPos, graphScale); 

        if (std::abs(error) < startI) integral += error; else integral = 0;
        if ((error > 0 && prevError < 0) || (error < 0 && prevError > 0)) integral = 0;
        if (std::abs(error) < 0.5) integral = 0;
        
        derivative = error - prevError;
        double masterPower = (error * kP) + (integral * kI) + (derivative * kD);

        if (std::abs(masterPower) < 10 && std::abs(error) > 1.0) masterPower = (masterPower > 0) ? 10 : -10;
        if (masterPower > maxSpeed) masterPower = maxSpeed;
        if (masterPower < -maxSpeed) masterPower = -maxSpeed;
        
        double headingError = targetTheta - robot_theta;
        while (headingError > M_PI) headingError -= 2 * M_PI;
        while (headingError < -M_PI) headingError += 2 * M_PI;

        double turnCorrection = headingError * kP_Heading; 
        turnCorrection = -turnCorrection; // Invert for reverse

        left_motor_group.move_velocity(masterPower + turnCorrection);
        right_motor_group.move_velocity(masterPower - turnCorrection);

        if (std::abs(error) < 0.5 && std::abs(derivative) < 0.5) break;

        prevError = error;
        pros::delay(20);
    }
    
    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

// ==========================================
//          REAL-TIME TUNER LOGIC
// ==========================================
// ==========================================
//          CUSTOM PRECISION TUNER
// ==========================================
void tuningLoop() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::lcd::initialize();
    resetGraph(); 

    const char* modeNames[] = { "Dr 24", "Dr 12", "SeqD", "Tn 90", "Tn 45", "SeqT" };
    
    // UI State
    int currentParam = 0; // 0=P, 1=I, 2=D
    int digitIndex = 2;   // Tracks which digit we are on
    
    uint32_t lastPrintTime = 0;

    // --- CONFIGURATION PER PARAMETER ---
    // We define different digits/weights for P/D vs I
    
    // P & D Config: Format %06.2f (e.g., 012.34)
    // Digits: 100, 10, 1, 0.1, 0.01
    double weightsPD[] = { 100.0, 10.0, 1.0, 0.1, 0.01 };
    int cursorMapPD[]  = {  2,    3,    4,   6,    7   }; // Screen character positions
    int maxIndexPD = 4; // 0 to 4

    // I Config: Format %05.3f (e.g., 0.123)
    // Digits: 1, 0.1, 0.01, 0.001
    double weightsI[] = { 1.0, 0.1, 0.01, 0.001 };
    int cursorMapI[]  = {  2,   4,    5,     6  }; // Screen character positions ("0.123")
    int maxIndexI = 3; // 0 to 3

    while (true) {
        bool isTurnMode = (testMode >= 3);
        PIDConfig* currentConfig = isTurnMode ? &turnPID_Consts : &forwardPID_Consts;
        
        // Determine which settings to use based on currentParam
        double* currentWeights;
        int* currentMap;
        int currentMaxIndex;
        
        if (currentParam == 1) { // kI
            currentWeights = weightsI;
            currentMap = cursorMapI;
            currentMaxIndex = maxIndexI;
        } else { // kP or kD
            currentWeights = weightsPD;
            currentMap = cursorMapPD;
            currentMaxIndex = maxIndexPD;
        }

        // Safety: If we switched params and index is out of bounds, fix it
        if (digitIndex > currentMaxIndex) digitIndex = currentMaxIndex;

        // --- 1. DISPLAY ---
        if (pros::millis() - lastPrintTime > 100) {
            // Line 0: Mode
            master.print(0, 0, "M:%s", modeNames[testMode]); 
            pros::delay(50); 

            // Line 1: Value (Formatted Differently)
            if (currentParam == 0) { // P
                master.print(1, 0, "P:%06.2f", currentConfig->kP);
            } 
            else if (currentParam == 1) { // I
                master.print(1, 0, "I:%05.3f", currentConfig->kI);
            } 
            else { // D
                master.print(1, 0, "D:%06.2f", currentConfig->kD);
            }
            pros::delay(50);

            // Line 2: The Cursor
            char cursorStr[20];
            memset(cursorStr, ' ', 19); cursorStr[19] = '\0';
            
            // Place caret using the map
            int screenPos = currentMap[digitIndex];
            cursorStr[screenPos] = '^';
            
            master.print(2, 0, "%s", cursorStr);
            
            lastPrintTime = pros::millis();
        }

        // --- 2. INPUTS ---

        // CHANGE MODE (X)
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            testMode++; if(testMode > 5) testMode = 0;
            resetGraph();
        }

        // SELECT PARAMETER (Left / Right)
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            currentParam++; if(currentParam > 2) currentParam = 0;
            digitIndex = 2; // Reset cursor to "1s" place for convenience
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            currentParam--; if(currentParam < 0) currentParam = 2;
            digitIndex = 2;
        }

        // SELECT DIGIT (R1 / R2)
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) { // Move Right (Precise)
            digitIndex++; 
            if(digitIndex > currentMaxIndex) digitIndex = currentMaxIndex;
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) { // Move Left (Coarse)
            digitIndex--;
            if(digitIndex < 0) digitIndex = 0;
        }

        // EDIT VALUE (Up / Down)
        bool up = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP);
        bool down = master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN);

        if (up || down) {
            double change = currentWeights[digitIndex];
            if (down) change = -change;

            if (currentParam == 0) currentConfig->kP += change;
            else if (currentParam == 1) currentConfig->kI += change;
            else if (currentParam == 2) currentConfig->kD += change;

            // Clamp to 0
            if(currentConfig->kP < 0) currentConfig->kP = 0;
            if(currentConfig->kI < 0) currentConfig->kI = 0;
            if(currentConfig->kD < 0) currentConfig->kD = 0;
        }

        // RUN TEST (A)
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            resetGraph();
            if (testMode == 0) driveForwardPID(24.0, 100, 0, 2000);
            else if (testMode == 1) driveForwardPID(12.0, 100, 0, 1500);
            else if (testMode == 2) { driveForwardPID(12.0, 100, 0, 1500); pros::delay(300); driveForwardPID(12.0, 100, 0, 1500); pros::delay(300); driveReversePID(24.0, 100, 2500); }
            else if (testMode == 3) turnToAnglePID(90.0, 50, 1500);
            else if (testMode == 4) turnToAnglePID(45.0, 50, 1500);
            else if (testMode == 5) { turnToAnglePID(45.0, 50, 1500); pros::delay(500); turnToAnglePID(90.0, 50, 1500); pros::delay(500); turnToAnglePID(0.0, 50, 2000); }
        }

        // RESET ODOM (B)
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            robot_x = 0; robot_y = 0; robot_theta = 0;
            resetGraph();
        }
        pros::delay(20);
    }
}

// Controls:
// X: Toggle between tuning Forward or Turning.
// Left / Right: Select kP, kI, or kD.
// Up / Down: Increase or Decrease the value.
// A: Run the test (Drive 24" or Turn 90deg).
// B: Reset Odometry to (0,0,0) (Use this after physically pulling the robot back).