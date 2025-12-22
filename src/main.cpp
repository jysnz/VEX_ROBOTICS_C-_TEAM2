#include "main.h"
#include "lemlib/api.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>

ASSET(path_jerryio_txt);

// --- Robot state ---
double x = 0.0, y = 0.0, theta = 0.0, heading = 0.0;

// --- Constants ---
const double wheelDiameter = 3.25; // inches
const double trackWidth = 24;    // distance between wheels
const double ticksPerRev = 360.0;  // motor degrees per revolution
double turnCalibration = 1.80;
const double STOP_TOLERANCE = 5.0; // Stop when within +/- 5 motor degrees of the target

const float PI = 3.14159;

// PID constants
float kP = 1.0;
float kI = 0.0;
float kD = 0.5;

double prevLeft = 0.0;
double prevRight = 0.0;
const int MAX_VOLTAGE = 11000; 
const double TURN_MULTIPLIER = 2.85;

// --- Motors ---
pros::MotorGroup left_motor_group({-2, -3, -1}, pros::MotorGears::green);
pros::MotorGroup right_motor_group({10, 8, 9}, pros::MotorGears::green);

pros::Motor catapult_arm(7, pros::MotorGears::red);
pros::Motor intake(4, pros::MotorGears::green);
pros::Motor matchloader(5, pros::MotorGears::red);
pros::Motor discore(12, pros::MotorGears::green);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// --- Drivetrain ---
lemlib::Drivetrain drivetrain(&left_motor_group,          // left motor group
                              &right_motor_group,         // right motor group
                              14,                         // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              400,                        // drivetrain rpm
                              2                           // horizontal drift
);

pros::adi::Ultrasonic ultrasonic('A', 'B');

// --- Odometry ---
pros::Imu imu(17);
pros::Rotation horizontal_encoder(20);
pros::adi::Encoder vertical_encoder('C', 'D', true);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder,
                                                lemlib::Omniwheel::NEW_325,
                                                -5.75);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder,
                                              lemlib::Omniwheel::NEW_325, -2.5);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical wheel
                            nullptr,                  // second vertical (none)
                            &horizontal_tracking_wheel, // horizontal wheel
                            nullptr, // second horizontal (none)
                            &imu     // imu
);

// Lateral & Angular PID
lemlib::ControllerSettings lateral_controller(10, 0, 3, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angular_controller(2, 0, 10, 0, 0, 0, 0, 0, 0);

// Expo drive curves
lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.019);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);

// Chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors, &throttle_curve, &steer_curve);


// --- Helper functions ---
float ticksToInches(float ticks) {
  return (ticks / ticksPerRev) * PI * wheelDiameter;
}

// SETTINGS
// Cap voltage at 11000mV (approx 85% power). 
// This ensures performance is the same at 100% batt and 60% batt.
// Helper function to convert RPM/Velocity (0-200) to Voltage (0-12000)
// 200 RPM ~= 12000 mV
double velocityToVoltage(double velocity) {
    return (velocity / 200.0) * 12000.0;
}

// --- New drive_for_inches function ---
double inchesToDegrees(double inches) {
    double wheelCircumference = PI * wheelDiameter;
    double rotations = inches / wheelCircumference;
    return rotations * 360.0; // degrees
}

void drive_to_object(double maxSpeed, double targetInches, int timeoutMs) {
    // 1. Reset start time
    int startTime = pros::millis();

    // 2. Start moving
    left_motor_group.move_velocity(maxSpeed);
    right_motor_group.move_velocity(maxSpeed);

    while (true) {
        // --- READ SENSOR ---
        // PROS ADI Ultrasonic returns Millimeters. 
        // 25.4 mm = 1 inch.
        double currentDistInches = ultrasonic.get_value() / 25.4;

        // If using V5 Distance Sensor instead, use this line:
        // double currentDistInches = distance_sensor.get() / 25.4;

        // --- CHECKS ---
        
        // 1. Check if we are close enough
        // We check > 0 because sometimes sensors return -1 if they see nothing
        if (currentDistInches < targetInches && currentDistInches > 0) {
            break; 
        }

        // 2. Check for timeout (Safety)
        if (pros::millis() - startTime > timeoutMs) {
            break;
        }

        pros::delay(10);
    }

    // 3. Stop
    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);
}

void reverse_to_object_smooth(double maxSpeed, double targetInches, int timeoutMs) {
    int startTime = pros::millis();
    
    // 1. Initial speed and ramping constants
    double currentSpeed = 0;
    const double accelRate = 4.0; // How fast it speeds up per 10ms
    const double minSpeed = 15.0; // Minimum speed to keep the robot moving
    
    // We define a "deceleration zone" in inches. 
    // Example: Start slowing down when within 10 inches of the target.
    const double decelZone = 10.0; 

    while (true) {
        // --- READ SENSOR ---
        double currentDist = ultrasonic.get_value() / 25.4; // Convert mm to inches
        
        // --- 1. HANDLE SPEED (Ramping) ---
        // Calculate how far we are from our stopping point
        double error = currentDist - targetInches;

        if (error > decelZone) {
            // ACCELERATION PHASE
            currentSpeed += accelRate;
            if (currentSpeed > maxSpeed) currentSpeed = maxSpeed;
        } 
        else {
            // DECELERATION PHASE
            // Scale speed based on how close we are to the targetInches
            currentSpeed = maxSpeed * (error / decelZone);
            if (currentSpeed < minSpeed) currentSpeed = minSpeed;
        }

        // --- 2. MOVE MOTORS ---
        // Using negative currentSpeed because we are reversing
        left_motor_group.move_velocity(-currentSpeed);
        right_motor_group.move_velocity(-currentSpeed);

        // --- 3. END CONDITIONS ---
        // Stop if we reach the target distance (with a 0.5 inch tolerance)
        if (currentDist <= targetInches + 0.5 && currentDist > 0) break;

        // Safety Timeout
        if (pros::millis() - startTime > timeoutMs) break;

        pros::delay(10);
    }

    // --- 4. FINAL HARD STOP ---
    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);
}

void drive_for_inches_consistent(double maxSpeedVelocity, double inches) {
    double targetDegrees = inchesToDegrees(inches);
    
    // Convert the user's velocity input (0-200) to Voltage (0-11000)
    // We cap it at MAX_VOLTAGE so it behaves the same on low battery.
    double maxVolts = velocityToVoltage(maxSpeedVelocity);
    if (maxVolts > MAX_VOLTAGE) maxVolts = MAX_VOLTAGE;

    left_motor_group.tare_position();
    right_motor_group.tare_position();

    // SLEW SETTINGS (Acceleration control)
    double currentVolts = 0;
    const double slewStep = 500; // How much voltage to add per loop (Adjust for acceleration)
    
    // DECEL SETTINGS
    const double decelStartRatio = 0.70; // Start slowing down at 70% of distance
    double decelPoint = targetDegrees * decelStartRatio;
    
    while (true) {
        double currentPos = std::abs(left_motor_group.get_position()); // Simplified for brevity

        // 1. CALCULATE DESIRED SPEED
        double targetVolts = maxVolts;

        // Deceleration Logic
        if (currentPos > decelPoint) {
            double remaining = targetDegrees - currentPos;
            // Proportional slow down
            targetVolts = maxVolts * (remaining / (targetDegrees - decelPoint));
            // Minimum voltage to keep moving (friction threshold)
            if (targetVolts < 2000) targetVolts = 2000; 
        }

        // 2. APPLY SLEW RATE (The Soft Start)
        // If we want to go faster than we are currently going, only add a little bit
        if (currentVolts < targetVolts) {
            currentVolts += slewStep;
            if (currentVolts > targetVolts) currentVolts = targetVolts;
        } 
        // If we need to slow down, we can drop voltage instantly (or slew down too)
        else {
            currentVolts = targetVolts;
        }

        // 3. MOVE WITH VOLTAGE
        // Using move_voltage is more "raw" and honest than move_velocity
        left_motor_group.move_voltage(currentVolts);
        right_motor_group.move_voltage(currentVolts);

        // Break condition
        if (currentPos >= targetDegrees - 5) break;
        
        pros::delay(10);
    }

    // Stop
    left_motor_group.move_voltage(0);
    right_motor_group.move_voltage(0);
}

void drive_backward_consistent(double maxSpeedVelocity, double inches) {
    double targetDegrees = -inchesToDegrees(inches); // Negative for reverse
    double maxVolts = velocityToVoltage(maxSpeedVelocity);
    if (maxVolts > MAX_VOLTAGE) maxVolts = MAX_VOLTAGE;

    left_motor_group.tare_position();
    right_motor_group.tare_position();

    double currentVolts = 0;
    const double slewStep = 500;
    const double decelStartRatio = 0.70;
    double decelPoint = targetDegrees * decelStartRatio; // This will be a negative number

    while (true) {
        double currentPos = left_motor_group.get_position(); // Don't use abs() here to track negative

        double targetVolts = -maxVolts; // Target is negative voltage

        // Deceleration (logic inverted for negative numbers)
        if (currentPos < decelPoint) {
            double remaining = targetDegrees - currentPos;
            targetVolts = -maxVolts * (remaining / (targetDegrees - decelPoint));
            if (targetVolts > -2000) targetVolts = -2000; 
        }

        // Slew Rate (Descending towards negative)
        if (currentVolts > targetVolts) {
            currentVolts -= slewStep;
            if (currentVolts < targetVolts) currentVolts = targetVolts;
        } else {
            currentVolts = targetVolts;
        }

        left_motor_group.move_voltage(currentVolts);
        right_motor_group.move_voltage(currentVolts);

        if (currentPos <= targetDegrees + 5) break;
        pros::delay(10);
    }
    left_motor_group.move_voltage(0);
    right_motor_group.move_voltage(0);
}

void turn_time_consistent(double maxSpeedVelocity, int durationMs, bool turnLeft) {
    // 1. CONVERT VELOCITY TO VOLTAGE 
    // We cap it at a value the battery can always reach (e.g., 10000mV or 10V)
    double targetVolts = velocityToVoltage(maxSpeedVelocity);
    if (targetVolts > 10000) targetVolts = 10000; 

    // 2. SLEW SETTINGS (For a smooth start)
    double currentVolts = 0;
    const double slewStep = 500; // Voltage added every 10ms
    
    // 3. TIMER SETUP
    uint32_t startTime = pros::millis();

    while (pros::millis() - startTime < durationMs) {
        
        // APPLY SLEW RATE (Soft Start)
        // This prevents the wheels from slipping/skidding at the start of the turn
        if (currentVolts < targetVolts) {
            currentVolts += slewStep;
            if (currentVolts > targetVolts) currentVolts = targetVolts;
        }

        // DECEL LOGIC (Optional: Slow down for the last 150ms for a precise stop)
        uint32_t elapsed = pros::millis() - startTime;
        if (elapsed > (durationMs - 150)) {
            currentVolts *= 0.8; // Simple decay
            if (currentVolts < 2500) currentVolts = 2500; 
        }

        // MOVE
        if (turnLeft) {
            left_motor_group.move_voltage(-currentVolts);
            right_motor_group.move_voltage(currentVolts);
        } else {
            left_motor_group.move_voltage(currentVolts);
            right_motor_group.move_voltage(-currentVolts);
        }

        pros::delay(10);
    }

    // 4. STOP & BRAKE
    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    left_motor_group.move_voltage(0);
    right_motor_group.move_voltage(0);
}

void drive_arc_consistent(double maxSpeedVelocity, double inches, double ratio, bool turnLeft, bool forward) {
    double targetDegrees = inchesToDegrees(inches);
    int dir = forward ? 1 : -1; // Direction multiplier
    
    double maxVolts = velocityToVoltage(maxSpeedVelocity);
    if (maxVolts > 10000) maxVolts = 10000;

    left_motor_group.tare_position();
    right_motor_group.tare_position();

    double currentVolts = 0;
    const double slewStep = 500;
    const double decelStartRatio = 0.70;
    double decelPoint = targetDegrees * decelStartRatio;
    
    while (true) {
        // Track the outside wheel (absolute value to handle reverse)
        double currentPos = turnLeft ? 
            std::abs(right_motor_group.get_position()) : 
            std::abs(left_motor_group.get_position());

        // 1. CALCULATE DESIRED SPEED
        double targetVolts = maxVolts;

        if (currentPos > decelPoint) {
            double remaining = targetDegrees - currentPos;
            targetVolts = maxVolts * (remaining / (targetDegrees - decelPoint));
            if (targetVolts < 2000) targetVolts = 2000; 
        }

        // 2. APPLY SLEW RATE
        if (currentVolts < targetVolts) {
            currentVolts += slewStep;
            if (currentVolts > targetVolts) currentVolts = targetVolts;
        } else {
            currentVolts = targetVolts;
        }

        // 3. APPLY DIRECTION AND RATIO
        double outerVolts = currentVolts * dir;
        double innerVolts = (currentVolts * ratio) * dir;

        // 4. MOVE
        if (turnLeft) {
            left_motor_group.move_voltage(innerVolts);
            right_motor_group.move_voltage(outerVolts);
        } else {
            left_motor_group.move_voltage(outerVolts);
            right_motor_group.move_voltage(innerVolts);
        }

        if (currentPos >= targetDegrees - 5) break;
        
        pros::delay(10);
    }

    left_motor_group.move_voltage(0);
    right_motor_group.move_voltage(0);
}

void updateOdometry() {
  float leftDist = ticksToInches(left_motor_group.get_position());
  float rightDist = ticksToInches(right_motor_group.get_position());
  float distance = (leftDist + rightDist) / 2.0;
  float deltaTheta = (rightDist - leftDist) / trackWidth;

  heading += deltaTheta * (180.0 / PI); // degrees
  x += distance * cos(heading * PI / 180.0);
  y += distance * sin(heading * PI / 180.0);
}

void odometryTask() {
  left_motor_group.tare_position();
  right_motor_group.tare_position();
  prevLeft = 0;
  prevRight = 0;

  while (true) {
    updateOdometry();
    pros::delay(10);
  }
}

void drive_back_and_forth(double times, double speed, double seconds){
    for(int i = 0; i < times; i++){
        left_motor_group.move(-speed);
        right_motor_group.move(-speed);
        pros::delay(seconds);
        left_motor_group.move(speed);
        right_motor_group.move(speed);
        pros::delay(seconds);
    }
}

// --- Initialize ---
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    matchloader.move_absolute(-1700, 100);

    static pros::Task odoTask(odometryTask);

    static pros::Task screen_task([]() {
        const int P_X = 240;
        const int P_W = 240;
        const int BG_COLOR = 0x202020;

        while (true) {
            pros::screen::set_pen(BG_COLOR);
            pros::screen::fill_rect(P_X, 0, 480, 240);

            double bat = pros::battery::get_capacity();
            int bat_y = 20;
            int bat_h = 30;
            int bat_w = 70;
            int icon_x = P_X + 130;

            pros::screen::set_pen(0xFFFFFF);
            pros::screen::print(pros::E_TEXT_LARGE, P_X + 10, bat_y + 3, "BAT: %3.0f%%", bat);

            uint32_t bat_col = (bat > 60) ? 0x00FFFF : (bat > 30 ? 0xFFA500 : 0xFF0000);
            pros::screen::set_pen(0xFFFFFF);
            pros::screen::draw_rect(icon_x, bat_y, icon_x + bat_w, bat_y + bat_h);
            pros::screen::fill_rect(icon_x + bat_w, bat_y + 8, icon_x + bat_w + 5, bat_y + bat_h - 8);

            int fill = (int)((bat / 100.0) * (bat_w - 4));
            pros::screen::set_pen(bat_col);
            pros::screen::fill_rect(icon_x + 2, bat_y + 2, icon_x + 2 + fill, bat_y + bat_h - 2);

            auto drawRow = [&](int row_idx, const char* label, double temp) {
                int row_h = 40;
                int start_y = 70;
                int y = start_y + (row_idx * row_h);

                pros::screen::set_pen(0xFFFFFF);
                pros::screen::print(pros::E_TEXT_MEDIUM, P_X + 10, y + 8, label);

                int bar_x = P_X + 80;
                int bar_w = 100;
                int bar_h = 16;
                int bar_y = y + 6;

                pros::screen::set_pen(0x404040);
                pros::screen::fill_rect(bar_x, bar_y, bar_x + bar_w, bar_y + bar_h);

                double stress = temp / 60.0;
                if(stress > 1.0) stress = 1.0;
                int fill_w = (int)(stress * bar_w);

                uint32_t col = 0x00FF00;
                if(temp > 45) col = 0xFFA500;
                if(temp > 55) col = 0xFF0000;

                pros::screen::set_pen(col);
                pros::screen::fill_rect(bar_x, bar_y, bar_x + fill_w, bar_y + bar_h);

                pros::screen::set_pen(0xFFFFFF);
                pros::screen::print(pros::E_TEXT_SMALL, bar_x + bar_w + 10, y + 8, "%.0fC", temp);
            };

            double d_temp = (left_motor_group.get_temperature() + right_motor_group.get_temperature()) / 2.0;

            drawRow(0, "Drive",  d_temp);
            drawRow(1, "Cata",   catapult_arm.get_temperature());
            drawRow(2, "Intake", intake.get_temperature());
            drawRow(3, "Load",   matchloader.get_temperature());
            drawRow(4, "Disc", discore.get_temperature());

            pros::delay(200);
        }
    });
}

// --- Operator Control ---
void opcontrol() {
    const int MAX_SPEED = 127;
    static bool controlsReversed = false;

    while (true) {
        bool intakeForward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        bool intakeReverse = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool intakePause = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool auton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);

        bool catapultArm = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool discoreDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        bool discoreUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);

        bool matchLoadUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B); 
        bool matchLoadDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

        bool reverseControlTap = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN);

        int move = -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (reverseControlTap) controlsReversed = !controlsReversed;
        if(auton) autonomous();

        if (controlsReversed) move = -move;

        int leftMotorSpeed = std::clamp(move + turn, -MAX_SPEED, MAX_SPEED);
        int rightMotorSpeed = std::clamp(move - turn, -MAX_SPEED, MAX_SPEED);

        left_motor_group.move(leftMotorSpeed);
        right_motor_group.move(rightMotorSpeed);

        if (catapultArm){
            catapult_arm.move_absolute(-600, 400);
            discore.move_velocity(0);
        } 
        else catapult_arm.move_absolute(0, 400);

        if (discoreDown) {
            discore.move_velocity(0);
        }
        else if (discoreUp) discore.move_absolute(-500, 200);

        if (matchLoadUp && !matchLoadDown)     matchloader.move_absolute(0, 100);
        else if (matchLoadDown && !matchLoadUp) matchloader.move_absolute(-1700, 100);

        if (intakeForward && !intakeReverse) intake.move_velocity(200);
        else if (intakeReverse && !intakeForward) intake.move_velocity(-200);
        if (intakePause) intake.move_velocity(0);

        pros::delay(20);
    }
}

// --- Autonomous ---
void autonomous() {

    drive_for_inches_consistent(80, 12);

    pros::delay(1000);

    turn_time_consistent(80, 679, true);

    pros::delay(1000);

    //Forward
    matchloader.move_absolute(0, 100);
    drive_for_inches_consistent(80, 27.2);

    pros::delay(500);

    turn_time_consistent(80, 679, true);

    pros::delay(500);

    //Move to matchload
    intake.move_velocity(-200);
    discore.move_absolute(-500, 200);
    drive_for_inches_consistent(80, 8.5);
    pros::delay(500);
    pros::delay(2000);


    drive_backward_consistent(80, 24.3);

    pros::delay(1000);
    discore.move_absolute(0, 200);
    catapult_arm.move_absolute(-400, 100);
    pros::delay(500);
    catapult_arm.move_absolute(0, 100);
    pros::delay(500);
    catapult_arm.move_absolute(-400, 100);
    pros::delay(500);
    catapult_arm.move_absolute(0, 100);
    intake.move_velocity(0);


    drive_for_inches_consistent(80, 23);
    pros::delay(500);

    discore.move_absolute(-500, 200);
    intake.move_velocity(-200);
    pros::delay(2000);

    drive_backward_consistent(150, 25);

    pros::delay(1000);
    discore.move_absolute(0, 200);
    catapult_arm.move_absolute(-400, 100);
    pros::delay(500);
    catapult_arm.move_absolute(0, 400);
    pros::delay(500);
    catapult_arm.move_absolute(-400, 100);
    pros::delay(500);
    catapult_arm.move_absolute(0, 400);
    intake.move_velocity(0);

    // drive_for_inches_consistent(80, 7);

    // drive_arc_consistent(80, 25, 30, false, false);
    // drive_for_inches_consistent(120, 20);

}
