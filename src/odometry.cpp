#include "odometry.hpp"
#include <cmath>
#include "robot_config.hpp"

// Global Definitions
double robot_x = 0;
double robot_y = 0;
double robot_theta = 0;

int forward_odom_raw = 0;
int heading_odom_raw = 0;
int sideways_odom_raw = 0;

// Initialize Sensors
AS5600 forward_odom('A');
AS5600 heading_odom('B');
AS5600 sideways_odom('C');

// --- PHYSICAL CONSTANTS (ADJUST THESE) ---
const double WHEEL_DIAMETER = 3.2; 
const double FORWARD_OFFSET = -0.66;  // Dist from center to forward wheel (inches)
const double HEADING_OFFSET = 1.64; // Dist from center to heading wheel (inches)
const double SIDEWAYS_OFFSET = 6.55; // Dist from center to sideways wheel (inches)
const double TRACK_WIDTH = FORWARD_OFFSET - HEADING_OFFSET; //

// --- AS5600 CLASS METHODS ---
AS5600::AS5600(char port) : sensor(port) {
    last_raw = sensor.get_value();
}

void AS5600::update() {
    int current_raw = sensor.get_value();
    int delta = current_raw - last_raw;

    if (delta > 2048) delta -= 4096;
    else if (delta < -2048) delta += 4096;

    if (std::abs(delta) > 10) { // Increased deadband for analog noise
        // IF THE ROBOT MOVES BACKWARD, CHANGE += TO -= BELOW
        total_ticks -= delta; 
        last_raw = current_raw;
    }
}

double AS5600::get_inches(double diameter) {
    return (total_ticks / 4096.0) * (diameter * M_PI); //
}

int AS5600::get_raw() {
    return sensor.get_value();
}

void AS5600::reset() {
    total_ticks = 0;
    last_raw = sensor.get_value();
}

// --- ODOMETRY TASK ---
void odom_task_fn() {
    double prev_F = 0, prev_H = 0, prev_S = 0;

    while (true) {
        // Update raw values for debugging
        forward_odom_raw = forward_odom.get_raw();
        heading_odom_raw = heading_odom.get_raw();
        sideways_odom_raw = sideways_odom.get_raw();

        // Update tracking logic
        forward_odom.update();
        heading_odom.update();
        sideways_odom.update();

        double cur_F = forward_odom.get_inches(WHEEL_DIAMETER);
        double cur_H = heading_odom.get_inches(WHEEL_DIAMETER);
        double cur_S = sideways_odom.get_inches(WHEEL_DIAMETER);

        double dF = cur_F - prev_F;
        double dH = cur_H - prev_H;
        double dS = cur_S - prev_S;

        // Calculate heading change
        double delta_theta = (dF - dH) / TRACK_WIDTH;

        double local_x, local_y;
        if (delta_theta == 0) {
            local_y = dF;
            local_x = dS;
        } else {
            // Arc-based translation
            local_y = 2 * std::sin(delta_theta / 2.0) * ((dF / delta_theta) + FORWARD_OFFSET);
            local_x = 2 * std::sin(delta_theta / 2.0) * ((dS / delta_theta) + SIDEWAYS_OFFSET);
        }

        // Global transformation
        double avg_theta = robot_theta + (delta_theta / 2.0);
        robot_x += local_y * std::sin(avg_theta) + local_x * std::cos(avg_theta);
        robot_y += local_y * std::cos(avg_theta) - local_x * std::sin(avg_theta);
        robot_theta += delta_theta;

        prev_F = cur_F; prev_H = cur_H; prev_S = cur_S;
        pros::delay(10);
    }
}

void drawPIDGraph(double error, int timeStep, bool isTurning) {
    int centerLine = 120; // Middle of the 240px screen
    int yPos;
    
    // SCALE CHECK: 
    // If error is 24 inches, 24 * 4 = 96. 120 - 96 = 24 (Visible).
    // If error is 50 inches, 50 * 4 = 200. 120 - 200 = -80 (INVISIBLE!).
    if (isTurning) {
        yPos = centerLine - (int)(error * (180.0 / M_PI) * 1.0); // 1px per degree
    } else {
        yPos = centerLine - (int)(error * 4.0); // 4px per inch
    }

    // CLAMPING: Ensures the pixel is ALWAYS on the screen
    if (yPos < 2) yPos = 2;
    if (yPos > 237) yPos = 237;

    int xPos = (timeStep % 480); // Wrap around the 480px width

    // Only erase the screen when we wrap back to the start (xPos 0)
    if (xPos == 0) {
        pros::screen::erase();
        // Redraw the center target line after erasing
        pros::screen::set_pen(0xFFFFFF); // White
        pros::screen::draw_line(0, centerLine, 480, centerLine);
    }
    
    // Draw the Error Pixel
    pros::screen::set_pen(0x00FF00); // Bright Green
    pros::screen::draw_pixel(xPos, yPos);
    
    // Draw a small vertical bar instead of a single pixel to make it easier to see
    pros::screen::draw_line(xPos, yPos - 1, xPos, yPos + 1);
}

void setPose(double newX, double newY, double newTheta) {
    // Stop the task briefly or use a mutex to prevent race conditions
    robot_x = newX;
    robot_y = newY;
    robot_theta = newTheta;
    
    // Reset the internal sensor counts to match the new starting point
    forward_odom.reset();
    heading_odom.reset();
    sideways_odom.reset();
}

// Inside driveForwardPID
double get_smooth_error(double target, double start) {
    static double history[5] = {0,0,0,0,0};
    double current = target - (forward_odom.get_inches(2.75) - start);
    
    // Shift history
    for(int i=4; i>0; i--) history[i] = history[i-1];
    history[0] = current;

    // Average the last 5 readings
    double sum = 0;
    for(int i=0; i<5; i++) sum += history[i];
    return sum / 5.0;
}

// --- PID MOVEMENT ---
void driveForwardPID(double targetDistance, double maxSpeed, double timeout) {
    const double kP = 3.5;      // MUCH LOWER to stop vibration
    const double kD = 5.0;      // Higher to "dampen" the noise
    const double kMin = 18.0;   
    
    double error = 0, prevError = 0;
    uint32_t startTime = pros::millis();
    double startForward = forward_odom.get_inches(2.75);
    int timeStep = 0;

    // Draw the initial center line
    pros::screen::set_pen(0xFFFFFF);
    pros::screen::draw_line(0, 120, 480, 120);

    while (pros::millis() - startTime < timeout) {
        // Use the smoothed error to stop the "back and forth"
        error = get_smooth_error(targetDistance, startForward);

        if (std::abs(error) < 0.6) break; 

        double derivative = error - prevError;
        double power = (error * kP) + (derivative * kD);

        // Anti-Vibration: If power is tiny, just stop
        if (std::abs(power) < 6.0) power = 0;
        else if (std::abs(power) < kMin) power = (error > 0) ? kMin : -kMin;

        if (power > maxSpeed) power = maxSpeed;
        if (power < -maxSpeed) power = -maxSpeed;

        left_motor_group.move_velocity(power);
        right_motor_group.move_velocity(power);

        drawPIDGraph(error, timeStep++, false);
        
        prevError = error;
        pros::delay(20);
    }
    left_motor_group.move_velocity(0); 
    right_motor_group.move_velocity(0);
}

void turnToAnglePID(double targetAngleDeg, double maxSpeed, double timeout) {
    double kP = 40.0, kI = 0.0, kD = 5.0, startI = 10.0 * (M_PI/180.0);
    double error = 0, prevError = 0, integral = 0;
    uint32_t startTime = pros::millis();
    double targetRad = targetAngleDeg * (M_PI/180.0);
    int timeStep = 0;

    while (pros::millis() - startTime < timeout) {
        error = targetRad - robot_theta;
        while (error > M_PI) error -= 2 * M_PI;
        while (error < -M_PI) error += 2 * M_PI;
        if (std::abs(error) < (1.0 * M_PI/180.0)) break;

        if (std::abs(error) < startI) integral += error; else integral = 0;
        double derivative = error - prevError;
        double power = (error * kP) + (integral * kI) + (derivative * kD);

        if (power > maxSpeed) power = maxSpeed;
        if (power < -maxSpeed) power = -maxSpeed;

        left_motor_group.move_velocity(power);
        right_motor_group.move_velocity(-power);

        drawPIDGraph(error, timeStep, true);
        timeStep++;
        prevError = error;
        pros::delay(20);
    }
    left_motor_group.move_velocity(0); right_motor_group.move_velocity(0);
}


void start_odom() {
    pros::Task odom_task(odom_task_fn);
}