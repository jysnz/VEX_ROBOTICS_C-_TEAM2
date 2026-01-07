#include "odometry.hpp"
#include <cmath>

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

    // Handle wrap-around (0 to 4095)
    if (delta > 2048) delta -= 4096;
    else if (delta < -2048) delta += 4096;

    // Deadband Filter: Only move if delta is significant
    if (std::abs(delta) > DEADBAND) {
        total_ticks += delta;
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

void drawPIDGraph(double error, int timeStep) {
    // Map the error to screen coordinates (Screen is 480x240)
    int yPos = 120 - (int)(error * 5); // 5 pixels per inch
    int xPos = (timeStep % 480);       // Move across the screen

    if (xPos == 0) pros::screen::erase(); // Clear screen for next pass
    
    pros::screen::set_pen(0x00FF00); // Green color for PID graph
    pros::screen::draw_pixel(xPos, yPos);
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



void start_odom() {
    pros::Task odom_task(odom_task_fn);
}