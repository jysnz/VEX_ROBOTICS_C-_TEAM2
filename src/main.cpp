// ============== INCLUDE ALL MODULES ==============
#include "main.h"
#include "constants.h"
#include "motor_config.h"
#include "utils.h"
#include "driving.h"
#include "mechanisms.h"
#include "autonomous.h"
#include "odometry.hpp"
#include "lemlib/api.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>

// ============== GLOBAL STATE ==============
double robot_x = 0.0, robot_y = 0.0, robot_theta = 0.0;

// ============== INITIALIZE ==============
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    matchloader.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // Screen task for telemetry display
    static pros::Task screen_task([]() {
        const int P_X = 240;
        const int P_W = 240;
        const int BG_COLOR = 0x202020;

        while (true) {
            pros::screen::set_pen(BG_COLOR);
            pros::screen::fill_rect(P_X, 0, 480, 240);

            // Battery display
            double bat = pros::battery::get_capacity();
            int bat_y = 20;
            int bat_h = 30;
            int bat_w = 70;
            int icon_x = P_X + 130;

            pros::screen::set_pen(0xFFFFFF);
            pros::screen::print(pros::E_TEXT_LARGE, P_X + 10, bat_y + 3,
                                "BAT: %3.0f%%", bat);

            uint32_t bat_col = (bat > 60) ? 0x00FFFF : (bat > 30 ? 0xFFA500 : 0xFF0000);
            pros::screen::set_pen(0xFFFFFF);
            pros::screen::draw_rect(icon_x, bat_y, icon_x + bat_w, bat_y + bat_h);
            pros::screen::fill_rect(icon_x + bat_w, bat_y + 8, icon_x + bat_w + 5,
                                    bat_y + bat_h - 8);

            int fill = (int)((bat / 100.0) * (bat_w - 4));
            pros::screen::set_pen(bat_col);
            pros::screen::fill_rect(icon_x + 2, bat_y + 2, icon_x + 2 + fill,
                                    bat_y + bat_h - 2);

            // Temperature display helper
            auto drawRow = [&](int row_idx, const char *label, double temp) {
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
                if (stress > 1.0) stress = 1.0;
                int fill_w = (int)(stress * bar_w);

                uint32_t col = 0x00FF00;
                if (temp > 45) col = 0xFFA500;
                if (temp > 55) col = 0xFF0000;

                pros::screen::set_pen(col);
                pros::screen::fill_rect(bar_x, bar_y, bar_x + fill_w, bar_y + bar_h);

                pros::screen::set_pen(0xFFFFFF);
                pros::screen::print(pros::E_TEXT_SMALL, bar_x + bar_w + 10, y + 8,
                                    "%.0fC", temp);
            };

            // Display motor temperatures
            double d_temp = (left_motor_group.get_temperature() +
                            right_motor_group.get_temperature()) / 2.0;

            drawRow(0, "Drive", d_temp);
            drawRow(1, "Cata", catapult_arm.get_temperature());
            drawRow(2, "Intake", intake.get_temperature());
            drawRow(3, "Load", matchloader.get_temperature());
            drawRow(4, "Disc", discore.get_temperature());

            pros::delay(200);
        }
    });

    // Start catapult background task
    static pros::Task catapult_task(catapultTask);
}

// ============== OPERATOR CONTROL ==============
void catapultControl() {
    const int MAX_SPEED = 127;

    while (true) {
        left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        // Display position info
        pros::lcd::print(1, "Y: %f", robot_y);

        pros::delay(20);

        // Read controller inputs
        bool intakeControl = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        bool outtakeControl = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        bool catapultArm = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool discoreDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        bool discoreUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);

        bool matchLoadUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
        bool matchLoadDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);

        int move = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Tank drive with cubing for better control
        int leftMotorSpeed = std::clamp(move + turn, -MAX_SPEED, MAX_SPEED);
        int rightMotorSpeed = std::clamp(move - turn, -MAX_SPEED, MAX_SPEED);

        left_motor_group.move(leftMotorSpeed);
        right_motor_group.move(rightMotorSpeed);

        // Catapult control
        if (catapultArm) {
            startCatapultShoot();
        }

        // Discore control
        if (discoreDown) {
            discore.move_absolute(0, 200);
        } else if (discoreUp) {
            discore.move_absolute(-2000, 200);
        }

        // Match loader control
        if (matchLoadUp && !matchLoadDown) {
            matchloader.move_absolute(0, 100);
        } else if (matchLoadDown && !matchLoadUp) {
            matchloader.move_absolute(-600, 100);
        }

        // Intake control
        if (intakeControl) {
            intake();
            switchScore.move_absolute(0, 200);
        }

        pros::delay(20);
    }
}

void opcontrol() {
    catapultControl();
}

// ============== AUTONOMOUS ==============
void autonomous() {
    twoVtwo();
}
