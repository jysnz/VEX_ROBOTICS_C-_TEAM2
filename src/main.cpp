#include "main.h"
#include "lemlib/api.hpp"
#include "pros/abstract_motor.hpp"
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
const double trackWidth = 12.0;    // distance between wheels
const double ticksPerRev = 360.0;  // motor degrees per revolution
double turnCalibration = 1.80;
const double STOP_TOLERANCE = 5.0; // Stop when within +/- 5 motor degrees of the target

const float PI = 3.14159;

// PID constants
float kP = 1.0;
float kI = 0.0;
float kD = 0.5;

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
                              10,                         // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              400,                        // drivetrain rpm
                              2                           // horizontal drift
);

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

double prevLeft = 0.0;
double prevRight = 0.0;

// --- Helper functions ---
float ticksToInches(float ticks) {
  return (ticks / ticksPerRev) * PI * wheelDiameter;
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
double swingTurnDegrees(double angle) {
    // The stationary left wheels are the center of the arc.
    // The right wheels travel an arc with a radius equal to the trackWidth (12.0 inches).

    // 1. Calculate the arc distance traveled by the moving (right) wheel:
    // Arc Length = (angle / 360) * 2 * PI * Radius
    // Radius = trackWidth
    double radius = trackWidth;
    double arc_distance = (angle / 360.0) * (2.0 * PI * radius);

    // 2. Convert this distance to required motor degrees:
    double wheelCircumference = PI * wheelDiameter;
    double rotations = arc_distance / wheelCircumference;
    return rotations * ticksPerRev; 
}

// ======================================
//   TURN LEFT (SWING TURN - Left Pivot)
// ======================================
void turn_left(double speed, double angle) {
    // Calculate the target encoder degrees for the moving side (right)
    double targetDeg = swingTurnDegrees(angle); 

    // Set the left motor to hold its position
    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    left_motor_group.move_velocity(0); // Ensure it starts stopped

    right_motor_group.tare_position();

    // Right motor moves forward (+) to the target
    right_motor_group.move_relative(targetDeg, speed);

    int startTime = pros::millis();
    while (true) {
        double rightPos = right_motor_group.get_position();
        
        // Check if the moving (right) motor is within tolerance of its target
        bool right_done = std::abs(targetDeg - rightPos) < STOP_TOLERANCE;

        if (right_done) break;

        // Safety timeout (3 seconds)
        if (pros::millis() - startTime > 3000) break;

        pros::delay(10);
    }
    
    // Explicitly stop the moving side
    right_motor_group.move_velocity(0);
    
    // IMPORTANT: Reset the left motor brake mode if it needs to coast later
    // You should probably set the default brake mode in initialize/opcontrol
    // but for safety here, we'll leave it as HOLD so the robot doesn't drift.
}

// --- New drive_for_inches function ---
double inchesToDegrees(double inches) {
    double wheelCircumference = PI * wheelDiameter;
    double rotations = inches / wheelCircumference;
    return rotations * 360.0; // degrees
}

void drive_for_inches(double maxSpeed, double inches) {
    double targetDegrees = inchesToDegrees(inches);

    left_motor_group.tare_position();
    right_motor_group.tare_position();

    const double accelRate = 2.0;
    const double decelStart = 0.6;

    double currentSpeed = 0;
    double decelPoint = targetDegrees * decelStart;

    while (true) {
        double leftPos  = std::abs(left_motor_group.get_position());
        double rightPos = std::abs(right_motor_group.get_position());
        double avgPos   = (leftPos + rightPos) / 2.0;

        // ACCELERATION
        if (avgPos < decelPoint) {
            currentSpeed += accelRate;
            if (currentSpeed > maxSpeed)
                currentSpeed = maxSpeed;
        }
        // DECELERATION
        else {
            double remaining = targetDegrees - avgPos;
            currentSpeed = maxSpeed * (remaining / (targetDegrees - decelPoint));
            if (currentSpeed < 10) currentSpeed = 10; // lower min for smooth stop
        }

        // END CONDITION
        if (avgPos >= targetDegrees - 2) break;

        left_motor_group.move_velocity(currentSpeed);
        right_motor_group.move_velocity(currentSpeed);

        pros::delay(10);
    }

    // ----- SMOOTH FINAL STOP -----
    double lastSpeed = std::max(currentSpeed, 10.0); // start ramp-down from current speed
    while (lastSpeed > 0) {
        left_motor_group.move_velocity(lastSpeed);
        right_motor_group.move_velocity(lastSpeed);
        lastSpeed -= 2;           // small decrement for smooth stop
        if (lastSpeed < 0) lastSpeed = 0;
        pros::delay(10);
    }

    // Hard stop
    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);
}


void drive_backward_for_inches(double maxSpeed, double inches) {
    // targetDegrees is the magnitude of the rotation needed (always positive)
    double targetDegrees = inchesToDegrees(inches);

    left_motor_group.tare_position();
    right_motor_group.tare_position();

    // Constant parameters
    const double accelRate = 2.0;
    const double decelStart = 0.6; // Start decelerating at 60% of the distance

    double currentSpeed = 0;
    double decelPoint = targetDegrees * decelStart;
    
    // We will use a negative speed command to move backward
    double backwardSpeedCommand = 0.0;

    while (true) {
        // Use the absolute value for position tracking, as in the original function.
        // This keeps the acceleration/deceleration logic simple and positive-based.
        double leftPos  = std::abs(left_motor_group.get_position());
        double rightPos = std::abs(right_motor_group.get_position());
        double avgPos   = (leftPos + rightPos) / 2.0;

        // ACCELERATION (same logic as forward)
        if (avgPos < decelPoint) {
            currentSpeed += accelRate;
            if (currentSpeed > maxSpeed)
                currentSpeed = maxSpeed;
        }
        // DECELERATION (same logic as forward)
        else {
            double remaining = targetDegrees - avgPos;
            currentSpeed = maxSpeed * (remaining / (targetDegrees - decelPoint));
            if (currentSpeed < 10) currentSpeed = 10; // lower min for smooth stop
        }

        // Set the final speed command to be negative for backward movement
        backwardSpeedCommand = -currentSpeed;

        // END CONDITION
        if (avgPos >= targetDegrees - 2) break; // Stop a little early

        left_motor_group.move_velocity(backwardSpeedCommand);
        right_motor_group.move_velocity(backwardSpeedCommand);

        pros::delay(10);
    }

    // ----- SMOOTH FINAL STOP (Ramp down to 0) -----
    // We ramp down the NEGATIVE speed towards 0
    double lastSpeedCommand = std::min(backwardSpeedCommand, -10.0); // start ramp-down from current speed
    while (lastSpeedCommand < 0) { // loop while the command is negative
        left_motor_group.move_velocity(lastSpeedCommand);
        right_motor_group.move_velocity(lastSpeedCommand);
        lastSpeedCommand += 2;      // small POSITIVE increment to approach 0
        if (lastSpeedCommand > 0) lastSpeedCommand = 0;
        pros::delay(10);
    }

    // Hard stop
    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);
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

void turn_right(double ms){
    left_motor_group.move(200);
    pros::delay(ms);
    left_motor_group.move(0);
}

void turn_left(double ms){
    right_motor_group.move(200);
    pros::delay(ms);
    right_motor_group.move(0);
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
        else if (discoreUp) discore.move_absolute(-650, 200);

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

    //Move to lower center goal
    matchloader.move_absolute(-1700, 100);
    drive_for_inches(80, 21.5); 
    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    pros::delay(500);

    //Turn left to lower center goal
    right_motor_group.move(100);
    pros::delay(250);
    right_motor_group.move(0);
    // left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // pros::delay(500);

    //Score to lower center goal the payload
    intake.move_velocity(100);
    pros::delay(2000);
    intake.move_velocity(0);

    //Move to matchload
    drive_backward_for_inches(80, 19.5);
    matchloader.move_absolute(0, 100);
    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pros::delay(500);

    //Turn left to face the matchload
    left_motor_group.move(-70);
    pros::delay(980);
    left_motor_group.move(0);
    // left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // pros::delay(500);

    //Get the matchload
    drive_for_inches(60, 8);
    // left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // pros::delay(500);
    intake.move_velocity(-200);
    discore.move_absolute(-650, 200);
    pros::delay(7000);

    //Score to long goal
    drive_backward_for_inches(80, 13.3);
    // left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // pros::delay(500);
    pros::delay(1000);
    catapult_arm.move_absolute(-400, 400);
    discore.move_velocity(0);
    intake.move_velocity(0);
    catapult_arm.move_absolute(0, 400);
}
