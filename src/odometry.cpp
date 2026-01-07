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

    // Handle the 0-4096 wrap
    if (delta > 2048) delta -= 4096;
    else if (delta < -2048) delta += 4096;

    // REMOVED the "if (abs(delta) > 10)" check. 
    // We need to count EVERY tick, or the PID will stutter at low speeds.
    
    // DIRECTION CHECK:
    // Push the robot FORWARD with your hand. 
    // If 'total_ticks' goes NEGATIVE, change "+=" to "-=" below.
    total_ticks += delta; 
    
    last_raw = current_raw;
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

void drawPIDGraph(double error, int timeStep, bool isTurn) {
    // Screen is 480x240. Center Y is 120.
    int centerY = 120;
    
    // SCALING (Zoom level)
    // For Drive: 1 inch error = 40 pixels (High Zoom)
    // For Turn:  1 degree error = 3 pixels
    double scale = isTurn ? 3.0 : 40.0;
    
    // If turning, convert radians to degrees first
    double value = isTurn ? (error * 180.0 / M_PI) : error;

    // Calculate Y position (Invert because screen Y=0 is top)
    int y = centerY - (int)(value * scale);

    // CLAMP values to keep them on screen
    if (y < 0) y = 0;
    if (y > 239) y = 239;

    // Wrap X axis (Time) so it loops continuously
    int x = timeStep % 480;

    // Clear the "sweep bar" (erase upcoming pixels to make it readable)
    if (x == 0) {
        pros::screen::set_pen(0x000000); // Black
        pros::screen::erase();           // Clear screen on loop
        
        // Draw the Target Line (Center)
        pros::screen::set_pen(0x444444); // Dark Gray
        pros::screen::draw_line(0, centerY, 480, centerY);
    }
    
    // Draw the Error Point
    // Green = Close enough, Red = Far away
    if (std::abs(value) < (isTurn ? 1.0 : 0.5)) {
        pros::screen::set_pen(0x00FF00); // Green (Good!)
    } else {
        pros::screen::set_pen(0xFF0000); // Red (Bad)
    }
    
    pros::screen::draw_pixel(x, y);
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

void driveForwardPID(double targetDistance, double maxSpeed, double timeout) {
    // --- PID CONSTANTS ---
    double kP = 8.0;      
    double kI = 0.1;      
    double kD = 7.5;      
    double startI = 3.0;  

    double error = 0;
    double prevError = 0;
    double integral = 0;
    double derivative = 0;

    int timeStep = 0;
    
    // 1. CAPTURE STARTING GLOBAL POSITION
    // These come from your odom_task_fn running in the background
    double startX = robot_x;
    double startY = robot_y;

    uint32_t startTime = pros::millis();

    while (pros::millis() - startTime < timeout) {
        
        // 2. CALCULATE DISTANCE TRAVELED (Hypotenuse)
        // This effectively uses ALL THREE encoders because robot_x/y 
        // are calculated using forward, heading, and sideways sensors.
        double distTraveled = std::hypot(robot_x - startX, robot_y - startY);
        
        // 3. DIRECTION CHECK (Dot Product approximation)
        // std::hypot always gives a positive number. 
        // If we are driving backwards (negative target), we need to make distance negative.
        // Simple check: relative angle to target. 
        // For simple DriveForward, we can just rely on the sign of targetDistance.
        if (targetDistance < 0) {
            distTraveled = -distTraveled;
        }

        // 4. Calculate Error
        error = targetDistance - distTraveled;

        // --- PID LOGIC (Same as before) ---
        if (std::abs(error) < startI) integral += error;
        else integral = 0;
        
        if ((error > 0 && prevError < 0) || (error < 0 && prevError > 0)) integral = 0;

        derivative = error - prevError;
        double power = (error * kP) + (integral * kI) + (derivative * kD);

        if (power > maxSpeed) power = maxSpeed;
        if (power < -maxSpeed) power = -maxSpeed;

        left_motor_group.move_velocity(power);
        right_motor_group.move_velocity(power);

        if (std::abs(error) < 0.5 && std::abs(derivative) < 0.1) break;

        drawPIDGraph(error, timeStep, false); // false = Driving (Inches)
        timeStep++;

        prevError = error;
        pros::delay(20);
    }
    
    left_motor_group.move_velocity(0); 
    right_motor_group.move_velocity(0);
}

void turnToAnglePID(double targetAngleDeg, double maxSpeed, double timeout) {
    // --- TUNING (Different from Drive!) ---
    // Turns usually need higher kP because friction opposes turning more than driving.
    double kP = 4.0;      
    double kI = 0.05;     
    double kD = 8.5;      
    double startI = 15.0; // Start integrating when within 15 degrees

    double error = 0;
    double prevError = 0;
    double integral = 0;
    double derivative = 0;

    int timeStep = 0;
    
    // Convert target to Radians because robot_theta is in Radians
    double targetRad = targetAngleDeg * (M_PI / 180.0);

    uint32_t startTime = pros::millis();

    while (pros::millis() - startTime < timeout) {
        
        // 1. CALCULATE ERROR
        error = targetRad - robot_theta;

        // 2. SHORTEST PATH LOGIC (Angle Wrapping)
        // This ensures the robot takes the shortest turn. 
        // Example: If at 350° and target is 10°, error becomes +20°, not -340°.
        while (error > M_PI) error -= 2 * M_PI;
        while (error < -M_PI) error += 2 * M_PI;

        // 3. Integral Logic
        if (std::abs(error) < (startI * M_PI / 180.0)) {
            integral += error;
        } else {
            integral = 0;
        }
        // Reset integral if we cross the target
        if ((error > 0 && prevError < 0) || (error < 0 && prevError > 0)) integral = 0;

        // 4. Derivative
        derivative = error - prevError;

        // 5. Calculate Power
        double power = (error * kP) + (integral * kI) + (derivative * kD);
        
        // (Optional) Boost power if error is small but robot is stuck
        // This helps overcome static friction (stiction) at the very end
        if (std::abs(error) > (1.0 * M_PI/180.0) && std::abs(power) < 15.0) {
            power = (power > 0) ? 15.0 : -15.0;
        }

        // 6. Cap Power
        if (power > maxSpeed) power = maxSpeed;
        if (power < -maxSpeed) power = -maxSpeed;

        // 7. Move Motors (Turn in Place)
        // Left goes forward, Right goes backward (or vice versa)
        left_motor_group.move_velocity(power);   
        right_motor_group.move_velocity(-power);

        // 8. Exit Condition
        // Exit if error is less than 1 degree AND velocity is near zero
        if (std::abs(error) < (1.0 * M_PI / 180.0) && std::abs(derivative) < 0.05) {
            break;
        }

        drawPIDGraph(error, timeStep, true); // true = Turning (Degrees)
        timeStep++;

        prevError = error;
        pros::delay(20);
    }
    
    left_motor_group.move_velocity(0); 
    right_motor_group.move_velocity(0);
}


void start_odom() {
    pros::Task odom_task(odom_task_fn);
}