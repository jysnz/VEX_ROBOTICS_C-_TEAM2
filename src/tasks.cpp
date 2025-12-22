#include "tasks.hpp"
#include "helpers.cpp"
#include "liblvgl/display/lv_display.h"

// Access global variables
float tune_kp = 10.0, tune_ki = 0.0, tune_kd = 3.0, tune_start_i = 2.0;
TuningMode currentMode = LATERAL;
int test_index = 0;
const char* test_names[] = {"Drive 12\"", "Drive 24\"", "Turn 90", "Swing 90", "Triple Move"};

lv_obj_t* pid_chart;
lv_chart_series_t* error_series;
lv_obj_t* pid_label;

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

void startScreenTask() {
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

void startTuningUI() {
    // 1. Setup Chart
    pid_chart = lv_chart_create(lv_screen_active());
    lv_obj_set_size(pid_chart, 200, 120);
    lv_obj_align(pid_chart, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_chart_set_type(pid_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_range(pid_chart, LV_CHART_AXIS_PRIMARY_Y, -10, 30); // Range for inches
    error_series = lv_chart_add_series(pid_chart, lv_palette_main(LV_PALETTE_PINK), LV_CHART_AXIS_PRIMARY_Y);

    // 2. Setup Label
    pid_label = lv_label_create(lv_screen_active());
    lv_obj_align(pid_label, LV_ALIGN_BOTTOM_LEFT, 10, -5);
    lv_label_set_recolor(pid_label, true);

    // 3. Update Task
    static pros::Task ui_task([]() {
        while (true) {
            const char* mode_name = (currentMode == LATERAL) ? "#00FFFF LATERAL#" : "#FFA500 ANGULAR#";
            
            // Update Graph: Show Y for Lateral moves, Theta for Angular
            float graph_val = (currentMode == LATERAL) ? chassis.getPose().y : chassis.getPose().theta;
            lv_chart_set_next_value(pid_chart, error_series, (int)graph_val);

            // Update Text
            lv_label_set_text_fmt(pid_label, 
                "MODE: %s  TEST: #00FF00 %s#\nP:%.2f I:%.3f D:%.2f  StI:%.1f", 
                mode_name, test_names[test_index], tune_kp, tune_ki, tune_kd, tune_start_i);

            pros::delay(50);
        }
    });
}