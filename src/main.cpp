#include "main.h"
#include "lemlib/api.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>

// ================= ODOMETRY STATE =================
double odom_x = 0.0;      // inches
double odom_y = 0.0;      // inches
double odom_theta = 0.0;  // radians

double last_left_inches = 0.0;
double last_right_inches = 0.0;

// ================= CONSTANTS =================
constexpr double WHEEL_DIAMETER = 3.25;
constexpr double TRACK_WIDTH = 12.0;
constexpr double PI = 3.1415926535;

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

lemlib::OdomSensors sensors(nullptr, // vertical wheel
                            nullptr,                  // second vertical (none)
                            nullptr, // horizontal wheel
                            nullptr, // second horizontal (none)
                            nullptr     // imu
);

// Lateral & Angular PID
lemlib::ControllerSettings lateral_controller(10, 0, 3, 3, 0, 100, 3, 500, 20);
lemlib::ControllerSettings angular_controller(2, 0, 10, 0, 0, 0, 0, 0, 0);

// Expo drive curves
lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.019);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);

// Chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors, &throttle_curve, &steer_curve);

double prevLeft = 0.0;
double prevRight = 0.0;

double motorClamp(double value) {
    if (value > 127) return 127;
    if (value < -127) return -127;
    return value;
}

struct PID {
    double kP, kI, kD;
    double integral = 0;
    double lastError = 0;

    double step(double error) {
        integral += error;
        double derivative = error - lastError;
        lastError = error;
        return kP * error + kI * integral + kD * derivative;
    }

    void reset() {
        integral = 0;
        lastError = 0;
    }
};


double degreesToInches(double deg) {
    return (deg / 360.0) * PI * WHEEL_DIAMETER;
}

void update_odometry() {
    double left_deg  = left_motor_group.get_position();
    double right_deg = right_motor_group.get_position();

    double left_inches  = degreesToInches(left_deg);
    double right_inches = degreesToInches(right_deg);

    double dL = left_inches  - last_left_inches;
    double dR = right_inches - last_right_inches;

    last_left_inches  = left_inches;
    last_right_inches = right_inches;

    double dS = (dL + dR) / 2.0;
    double dTheta = (dR - dL) / TRACK_WIDTH;

    odom_theta += dTheta;

    odom_x += dS * cos(odom_theta);
    odom_y += dS * sin(odom_theta);
}

void odom_task(void*) {
    left_motor_group.tare_position();
    right_motor_group.tare_position();

    last_left_inches = 0;
    last_right_inches = 0;

    while (true) {
        update_odometry();
        pros::delay(10);
    }
}

void drive_straight_pid(double inches, int maxSpeed, bool forward = true) {
    PID distancePID{6.0, 0.0, 20.0};
    PID driftPID{3.0, 0.0, 8.0};

    double startX = odom_x;
    double startY = odom_y;

    left_motor_group.tare_position();
    right_motor_group.tare_position();
    distancePID.reset();
    driftPID.reset();

    while (true) {
        update_odometry();

        double dx = odom_x - startX;
        double dy = odom_y - startY;
        double traveled = sqrt(dx * dx + dy * dy);
        double error = inches - traveled;

        if (fabs(error) < 0.3) break;

        double speed = distancePID.step(error);
        if (!forward) speed = -speed;

        if (speed > maxSpeed) speed = maxSpeed;
        if (speed < -maxSpeed) speed = -maxSpeed;

        double leftPos  = left_motor_group.get_position();
        double rightPos = right_motor_group.get_position();
        double driftError = leftPos - rightPos;

        double correction = driftPID.step(driftError);

        left_motor_group.move(motorClamp(speed - correction));
        right_motor_group.move(motorClamp(speed + correction));

        pros::delay(10);
    }

    left_motor_group.move(0);
    right_motor_group.move(0);
}

void turn_pid(double degrees, int maxSpeed, bool leftTurn = true) {
    PID turnPID{75.0, 0.0, 30.0};
    turnPID.reset();

    double target = odom_theta +
        (leftTurn ? 1 : -1) * degrees * (PI / 180.0);

    while (true) {
        update_odometry();

        double error = target - odom_theta;

        if (fabs(error) < (1.0 * PI / 180.0)) break;

        double speed = turnPID.step(error);

        if (speed > maxSpeed) speed = maxSpeed;
        if (speed < -maxSpeed) speed = -maxSpeed;

        left_motor_group.move(motorClamp(-speed));
        right_motor_group.move(motorClamp(speed));

        pros::delay(10);
    }

    left_motor_group.move(0);
    right_motor_group.move(0);
}

void drive_forward(double inches, int speed) {
    drive_straight_pid(inches, speed, true);
}

void drive_backward(double inches, int speed) {
    drive_straight_pid(inches, speed, false);
}

void turn_left(double degrees, int speed) {
    turn_pid(degrees, speed, true);
}

void turn_right(double degrees, int speed) {
    turn_pid(degrees, speed, false);
}

void drive_back_and_forth(double times, double inches, int speed) {
    for (int i = 0; i < times; i++) {
        drive_forward(inches, speed);
        pros::delay(100);   // small settle delay (optional)
        drive_backward(inches, speed);
        pros::delay(100);
    }
}

// --- Initialize ---
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    matchloader.move_absolute(-1700, 100);

    static pros::Task odomTask(odom_task);

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

    // 1. Set the robot's initial position and heading (e.g., x=0, y=0, heading=90 degrees)
    chassis.setPose(0, 0, 90); 
    
    // Move to lower center goal
    matchloader.move_absolute(-1700, 100);
    chassis.moveToPoint(33, 0, 2000);
    pros::delay(1200);
    chassis.moveToPoint(0, 3, 200);
    pros::delay(500);
    chassis.moveToPoint(33, 0, 2000);
    
    // //Score to lower center goal the payload
    intake.move_velocity(200);
    pros::delay(2000);
    intake.move_velocity(0);

    // //Move to matchload
    // chassis.moveToPoint(0, -30, 2000);
    // chassis.moveToPoint(0, -10, 700);
    // chassis.moveToPoint(20, 0, 2000);

    // //Turn left to face the matchload
    // left_motor_group.move(-100);
    // pros::delay(720);
    // left_motor_group.move(0);

    // //Get the matchload
    // drive_for_inches(80, 9.5);
    // intake.move_velocity(-200);
    // discore.move_absolute(-650, 200);
    // drive_back_and_forth(13,40, 180);
    // pros::delay(1000);
    // intake.move_velocity(0);

    // //Score to long goal
    // drive_backward_for_inches(60, 13);
    // pros::delay(1000);
    // catapult_arm.move_absolute(-300, 400);
    // discore.move_velocity(0);

    // //Backward to face the matchload
    // drive_backward_for_inches(100, 7);

}