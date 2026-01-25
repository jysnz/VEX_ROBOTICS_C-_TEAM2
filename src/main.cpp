#include "main.h"
#include "lemlib/api.hpp"
#include "odometry.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>

bool drive_task_running = false;
bool drive_backward_task_running = false;

std::vector<double> recordedPaths; // Stores inches
bool recordingActive = false;
double liveInches = 0;

enum CatapultState { CATA_IDLE, CATA_MOVING };

CatapultState catapultState = CATA_IDLE;
bool catapultReversed = false;
int catapultSpeed = 0;

// --- Robot state ---
double x = 0.0, y = 0.0, theta = 0.0, heading = 0.0;

// ms required to turn 90 degrees at TURN_SPEED
const double MS_PER_90_DEG = 660.0;

// --- Constants ---
const double wheelDiameter = 3.25; // inches
const double trackWidth = 12.0;    // distance between wheels
const double ticksPerRev = 360.0;  // motor degrees per revolution
double turnCalibration = 1.80;
const double STOP_TOLERANCE =
    5.0; // Stop when within +/- 5 motor degrees of the target

const float PI = 3.14159;
bool in_motion = false;

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

// Plug 'Ping' into port E, 'Echo' into port F (Change letters as needed)
pros::adi::Ultrasonic ultrasonic('A', 'B');

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

bool catapult_at_endpoint() {
  return std::abs(catapult_arm.get_actual_velocity()) < 2;
}

void catapult_start(int speed, bool reversed = false) {
  if (catapultState != CATA_IDLE)
    return;

  catapultReversed = reversed;
  catapultSpeed = speed;

  int dir = reversed ? -1 : 1;
  catapult_arm.move_velocity(dir * speed);

  catapultState = CATA_MOVING;
}

void catapult_task(void *) {
  while (true) {
    if (catapultState == CATA_MOVING) {
      if (catapult_at_endpoint()) {
        catapult_arm.move_velocity(0);
        catapultState = CATA_IDLE;
      }
    }
    pros::delay(10);
  }
}

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

int turn_deg_to_ms(double degrees) {
  return static_cast<int>((degrees / 90.0) * MS_PER_90_DEG);
}

// --- New drive_for_inches function ---
double inchesToDegrees(double inches) {
  double wheelCircumference = PI * wheelDiameter;
  double rotations = inches / wheelCircumference;
  return rotations * 360.0; // degrees
}

void turn_left_deg(double degrees, int speed, double delay) {
  int timeMs = turn_deg_to_ms(degrees);

  left_motor_group.move_velocity(-speed);
  right_motor_group.move_velocity(speed);

  pros::delay(timeMs);

  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  pros::delay(delay);
}

void turn_right_deg(double degrees, int speed, double delay) {
  int timeMs = turn_deg_to_ms(degrees);

  left_motor_group.move_velocity(speed);
  right_motor_group.move_velocity(-speed);

  pros::delay(timeMs);

  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  pros::delay(delay);
}

bool drive_hit_wall() {
  static int stoppedTime = 0;

  const double VEL_THRESH = 5; // rpm
  const int STOP_TIME_MS = 150;

  double lv = std::abs(left_motor_group.get_actual_velocity());
  double rv = std::abs(right_motor_group.get_actual_velocity());

  if (lv < VEL_THRESH && rv < VEL_THRESH) {
    stoppedTime += 10;
    if (stoppedTime >= STOP_TIME_MS)
      return true;
  } else {
    stoppedTime = 0;
  }

  return false;
}

bool drive_forward_nb_running = false;

void drive_forward_nb_task(void *param) {
  drive_forward_nb_running = true;

  double *args = static_cast<double *>(param);
  double maxSpeed = args[0];
  double inches = args[1];

  double targetDegrees = inchesToDegrees(inches);

  left_motor_group.tare_position();
  right_motor_group.tare_position();

  const double accelRate = 2.0;
  double currentSpeed = 0;

  while (true) {
    double leftPos = std::abs(left_motor_group.get_position());
    double rightPos = std::abs(right_motor_group.get_position());
    double avgPos = (leftPos + rightPos) / 2.0;

    // 🧱 HIT WALL → STOP
    if (drive_hit_wall())
      break;

    // 🎯 DISTANCE REACHED
    if (avgPos >= targetDegrees - 2)
      break;

    // ACCEL ONLY
    currentSpeed += accelRate;
    if (currentSpeed > maxSpeed)
      currentSpeed = maxSpeed;

    left_motor_group.move_velocity(currentSpeed);
    right_motor_group.move_velocity(currentSpeed);

    pros::delay(10);
  }

  // HARD STOP
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  delete[] args;
  drive_forward_nb_running = false;
}

void drive_for_inches_async_nonblocking(double maxSpeed, double inches) {
  if (drive_forward_nb_running)
    return;

  double *args = new double[2]{maxSpeed, inches};
  pros::Task(drive_forward_nb_task, args, "Drive Forward NB");
}

bool drive_backward_nb_running = false;

void drive_backward_nb_task(void *param) {
  drive_backward_nb_running = true;

  double *args = static_cast<double *>(param);
  double maxSpeed = args[0];
  double inches = args[1];

  double targetDegrees = inchesToDegrees(inches);

  left_motor_group.tare_position();
  right_motor_group.tare_position();

  const double accelRate = 2.0;
  double currentSpeed = 0;

  while (true) {
    double leftPos = std::abs(left_motor_group.get_position());
    double rightPos = std::abs(right_motor_group.get_position());
    double avgPos = (leftPos + rightPos) / 2.0;

    // 🧱 HIT WALL → STOP
    if (drive_hit_wall())
      break;

    // 🎯 DISTANCE REACHED
    if (avgPos >= targetDegrees - 2)
      break;

    // ACCEL ONLY
    currentSpeed += accelRate;
    if (currentSpeed > maxSpeed)
      currentSpeed = maxSpeed;

    left_motor_group.move_velocity(-currentSpeed);
    right_motor_group.move_velocity(-currentSpeed);

    pros::delay(10);
  }

  // HARD STOP
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  delete[] args;
  drive_backward_nb_running = false;
}

void drive_backward_for_inches_async_nonblocking(double maxSpeed,
                                                 double inches) {
  if (drive_backward_nb_running)
    return;

  double *args = new double[2]{maxSpeed, inches};
  pros::Task(drive_backward_nb_task, args, "Drive Backward NB");
}

void distance_record_task(void *) {
  while (true) {
    if (recordingActive) {
      double leftDeg = left_motor_group.get_position();
      double rightDeg = right_motor_group.get_position();

      double avgDeg = (leftDeg + rightDeg) / 2.0;
      liveInches = inchesToDegrees(avgDeg);

      // LCD feedback
      pros::lcd::print(1, "Distance: %.2f in", liveInches);
    }

    pros::delay(20);
  }
}

void start_distance_recording() {
  left_motor_group.tare_position();
  right_motor_group.tare_position();

  liveInches = 0;
  recordingActive = true;

  pros::lcd::print(0, "Recording...");
}

void stop_distance_recording() {
  recordingActive = false;
  recordedPaths.push_back(liveInches);

  pros::lcd::print(0, "Saved: %.2f in", liveInches);
}

void distanceTuning() {
  while (true) {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      start_distance_recording();
    }

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      stop_distance_recording();
    }

    pros::delay(10);
  }
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
    double leftPos = std::abs(left_motor_group.get_position());
    double rightPos = std::abs(right_motor_group.get_position());
    double avgPos = (leftPos + rightPos) / 2.0;

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
      if (currentSpeed < 10)
        currentSpeed = 10; // lower min for smooth stop
    }

    // END CONDITION
    if (avgPos >= targetDegrees - 2)
      break;

    left_motor_group.move_velocity(currentSpeed);
    right_motor_group.move_velocity(currentSpeed);

    pros::delay(10);
  }

  // ----- SMOOTH FINAL STOP -----
  double lastSpeed =
      std::max(currentSpeed, 10.0); // start ramp-down from current speed
  while (lastSpeed > 0) {
    left_motor_group.move_velocity(lastSpeed);
    right_motor_group.move_velocity(lastSpeed);
    lastSpeed -= 2; // small decrement for smooth stop
    if (lastSpeed < 0)
      lastSpeed = 0;
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
    // Use the absolute value for position tracking, as in the original
    // function. This keeps the acceleration/deceleration logic simple and
    // positive-based.
    double leftPos = std::abs(left_motor_group.get_position());
    double rightPos = std::abs(right_motor_group.get_position());
    double avgPos = (leftPos + rightPos) / 2.0;

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
      if (currentSpeed < 10)
        currentSpeed = 10; // lower min for smooth stop
    }

    // Set the final speed command to be negative for backward movement
    backwardSpeedCommand = -currentSpeed;

    // END CONDITION
    if (avgPos >= targetDegrees - 2)
      break; // Stop a little early

    left_motor_group.move_velocity(backwardSpeedCommand);
    right_motor_group.move_velocity(backwardSpeedCommand);

    pros::delay(10);
  }

  // ----- SMOOTH FINAL STOP (Ramp down to 0) -----
  // We ramp down the NEGATIVE speed towards 0
  double lastSpeedCommand = std::min(
      backwardSpeedCommand, -10.0); // start ramp-down from current speed
  while (lastSpeedCommand < 0) {    // loop while the command is negative
    left_motor_group.move_velocity(lastSpeedCommand);
    right_motor_group.move_velocity(lastSpeedCommand);
    lastSpeedCommand += 2; // small POSITIVE increment to approach 0
    if (lastSpeedCommand > 0)
      lastSpeedCommand = 0;
    pros::delay(10);
  }

  // Hard stop
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);
}

void drive_backward_task_fn(void *param) {
  drive_backward_task_running = true;

  double *args = static_cast<double *>(param);
  double maxSpeed = args[0];
  double inches = args[1];
  int delayMs = static_cast<int>(args[2]);

  drive_backward_for_inches(maxSpeed, inches);

  // ✅ DELAY BEFORE NEXT MOVEMENT IS ALLOWED
  pros::delay(delayMs);

  delete[] args;
  drive_backward_task_running = false;
}

void drive_backward_inches_async(double maxSpeed, double inches, int delayMs) {
  if (drive_backward_task_running)
    return;

  double *args = new double[3];
  args[0] = maxSpeed;
  args[1] = inches;
  args[2] = static_cast<double>(delayMs);

  pros::Task driveBackwardTask(drive_backward_task_fn, args,
                               "Drive Backward Task");
}

void drive_task_fn(void *param) {
  drive_task_running = true;

  double *args = static_cast<double *>(param);
  double maxSpeed = args[0];
  double inches = args[1];
  int delayMs = static_cast<int>(args[2]);

  drive_for_inches(maxSpeed, inches);

  // ✅ DELAY BEFORE NEXT MOVEMENT IS ALLOWED
  pros::delay(delayMs);

  delete[] args;
  drive_task_running = false;
}

void drive_for_inches_async(double maxSpeed, double inches, int delayMs) {
  if (drive_task_running)
    return; // 🚫 block second start

  double *args = new double[3];
  args[0] = maxSpeed;
  args[1] = inches;
  args[2] = static_cast<double>(delayMs);

  pros::Task driveTask(drive_task_fn, args, "Drive Task");
}

void shoot() {
  catapult_arm.move_absolute(-600, 70);
  discore.move_absolute(0, 200);
  intake.move_velocity(-200);
  pros::delay(300);
  intake.move_velocity(0);
  catapult_arm.move_absolute(0, 40);
}

void catapultControl() {
  const int MAX_SPEED = 127;
  static bool controlsReversed = false;
  while (true) {
    // pros::lcd::print(0, "X: %f", robot_x);
    pros::lcd::print(1, "Y: %f", robot_y);
    // pros::lcd::print(2, "Theta: %f", robot_theta * (180/M_PI)); // Convert to
    // degrees
    pros::delay(20);

    bool intakeForward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    bool intakeReverse = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    bool intakePause = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

    bool catapultArm = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool discoreDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    bool discoreUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);

    bool matchLoadUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    bool matchLoadDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

    bool reverseControlTap =
        controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN);

    int move = -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    if (reverseControlTap)
      controlsReversed = !controlsReversed;

    if (controlsReversed)
      move = -move;

    int leftMotorSpeed = std::clamp(move + turn, -MAX_SPEED, MAX_SPEED);
    int rightMotorSpeed = std::clamp(move - turn, -MAX_SPEED, MAX_SPEED);

    left_motor_group.move(leftMotorSpeed);
    right_motor_group.move(rightMotorSpeed);

    if (catapultArm) {
      shoot();
    } else
      catapult_arm.move_absolute(0, 400);

    if (discoreDown) {
      discore.move_absolute(0, 200);
    } else if (discoreUp)
      discore.move_absolute(800, 200);

    if (matchLoadUp && !matchLoadDown) {
      matchloader.move_absolute(0, 100);
      discore.move_absolute(800, 200);
    } else if (matchLoadDown && !matchLoadUp) {
      matchloader.move_absolute(1400, 100);
      discore.move_absolute(0, 200);
    }

    if (intakeForward && !intakeReverse)
      intake.move_velocity(200);
    else if (intakeReverse && !intakeForward)
      intake.move_velocity(-200);
    if (intakePause)
      intake.move_velocity(0);

    pros::delay(20);
  }
}

void wall_reset(int voltage = 8000, int settleTime = 200) {
  left_motor_group.move_voltage(voltage);
  right_motor_group.move_voltage(voltage);

  int stalledTime = 0;

  while (stalledTime < settleTime) {
    double lv = std::abs(left_motor_group.get_actual_velocity());
    double rv = std::abs(right_motor_group.get_actual_velocity());

    if (lv < 3 && rv < 3) {
      stalledTime += 10;
    } else {
      stalledTime = 0;
    }

    pros::delay(10);
  }

  left_motor_group.move_voltage(0);
  right_motor_group.move_voltage(0);

  pros::delay(50);
}

// --- Initialize ---
void initialize() {
  catapult_start(200, true); // reset
  pros::lcd::initialize();
  chassis.calibrate();
  setPose(0, 0, 0);
  start_odom();
  matchloader.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  static pros::Task catapultTask(catapult_task);
  pros::Task recordTask(distance_record_task);

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
      pros::screen::print(pros::E_TEXT_LARGE, P_X + 10, bat_y + 3,
                          "BAT: %3.0f%%", bat);

      uint32_t bat_col =
          (bat > 60) ? 0x00FFFF : (bat > 30 ? 0xFFA500 : 0xFF0000);
      pros::screen::set_pen(0xFFFFFF);
      pros::screen::draw_rect(icon_x, bat_y, icon_x + bat_w, bat_y + bat_h);
      pros::screen::fill_rect(icon_x + bat_w, bat_y + 8, icon_x + bat_w + 5,
                              bat_y + bat_h - 8);

      int fill = (int)((bat / 100.0) * (bat_w - 4));
      pros::screen::set_pen(bat_col);
      pros::screen::fill_rect(icon_x + 2, bat_y + 2, icon_x + 2 + fill,
                              bat_y + bat_h - 2);

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
        if (stress > 1.0)
          stress = 1.0;
        int fill_w = (int)(stress * bar_w);

        uint32_t col = 0x00FF00;
        if (temp > 45)
          col = 0xFFA500;
        if (temp > 55)
          col = 0xFF0000;

        pros::screen::set_pen(col);
        pros::screen::fill_rect(bar_x, bar_y, bar_x + fill_w, bar_y + bar_h);

        pros::screen::set_pen(0xFFFFFF);
        pros::screen::print(pros::E_TEXT_SMALL, bar_x + bar_w + 10, y + 8,
                            "%.0fC", temp);
      };

      double d_temp = (left_motor_group.get_temperature() +
                       right_motor_group.get_temperature()) /
                      2.0;

      drawRow(0, "Drive", d_temp);
      drawRow(1, "Cata", catapult_arm.get_temperature());
      drawRow(2, "Intake", intake.get_temperature());
      drawRow(3, "Load", matchloader.get_temperature());
      drawRow(4, "Disc", discore.get_temperature());

      pros::delay(200);
    }
  });
}

// --- Operator Control ---
void opcontrol() { catapultControl(); }

void twovtwoNormalAuton() {
  // Move to lower center goal
  matchloader.move_absolute(1400, 200);
  drive_for_inches(80, 24.4);
  pros::delay(500);

  left_motor_group.move_velocity(50);
  right_motor_group.move_velocity(-50);
  pros::delay(450);
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  intake.move_velocity(-200);
  discore.move_absolute(850, 200);
  drive_for_inches(90, 8);
  pros::delay(1100);

  drive_backward_for_inches(50, 13.8);

  discore.move_absolute(0, 200);
  pros::delay(500);

  // shoot
  intake.move_velocity(0);
  catapult_arm.move_absolute(-600, 200);
  pros::delay(1500);
  catapult_arm.move_absolute(0, 200);

  discore.move_absolute(850, 70);
  pros::delay(500);
  intake.move_velocity(-200);

  // Second matchload
  drive_for_inches(50, 14);
  discore.move_absolute(0, 200);
  pros::delay(1200);

  catapult_arm.move_absolute(-600, 200);
  pros::delay(1000);
  catapult_arm.move_absolute(0, 200);

  // intake.move_velocity(200);
  // pros::delay(500);
  // intake.move_velocity(0);

  discore.move_absolute(850, 200);
  pros::delay(5000);

  pros::delay(500);

  drive_backward_for_inches(50, 13.5);

  discore.move_absolute(0, 200);
  pros::delay(500);

  intake.move_velocity(0);
  catapult_arm.move_absolute(-600, 70);
  intake.move_velocity(-200);
  pros::delay(1500);
  catapult_arm.move_absolute(0, 200);
  intake.move_velocity(0);

  pros::delay(500);

  left_motor_group.move_velocity(50);
  pros::delay(500);
  left_motor_group.move_velocity(0);

  pros::delay(500);

  drive_for_inches(80, 3);

  left_motor_group.move_velocity(-50);
  pros::delay(500);
  left_motor_group.move_velocity(0);

  drive_backward_for_inches(40, 20);
}

void skillsV2() {
  // Reset the catapult
  catapult_start(200, true); // reset

  // Intake two balls
  intake.move_velocity(-200);
  discore.move_absolute(850, 200);
  drive_for_inches(80, 31.5);

  pros::delay(2000);

  drive_backward_for_inches(80, 1.5);
  pros::delay(400);

  turn_left_deg(60, 50, 500);

  // Go to the other side
  drive_for_inches(80, 36);
  pros::delay(500);

  // Throw away the red balls
  matchloader.move_absolute(1400, 200);
  pros::delay(350);
  drive_for_inches(80, 3);
  pros::delay(1200);
  intake.move_velocity(0);
  matchloader.move_absolute(0, 200);
  pros::delay(200);

  // Turn to wall
  turn_right_deg(150, 100, 500);
  pros::delay(250);
  drive_backward_for_inches(80, 2);
  pros::delay(500);
  turn_right_deg(55, 50, 250);

  // Wall reset
  wall_reset();

  // Drive backwards for turn
  drive_backward_for_inches(40, 7.4);
  pros::delay(500);

  // // // Turn to face the long goal
  turn_left_deg(120, 30, 500);

  // // Drive backward to long goal
  drive_backward_for_inches(60, 3);
  pros::delay(350);
  drive_backward_for_inches_async_nonblocking(100, 5.5);
  pros::delay(500);

  // Shoot
  discore.move_absolute(0, 200);
  catapult_arm.move_absolute(-300, 200);
  catapult_arm.move_absolute(-600, 40);
  pros::delay(1500);
  catapult_arm.move_absolute(600, 200);
  matchloader.move_absolute(1400, 200);

  // Align the robot to the long goal
  pros::delay(500); // 1st alignment
  drive_for_inches(50, 2);
  pros::delay(350);
  drive_backward_for_inches_async_nonblocking(80, 4);
  pros::delay(500);

  // Gather matchload
  drive_for_inches(40, 9.8);
  drive_backward_for_inches(40, 0.5);
  drive_for_inches(40, 0.5);
  pros::delay(350);

  // Intake balls
  discore.move_absolute(800, 200);
  intake.move_velocity(-200);
  drive_backward_for_inches(40, 0.5);
  drive_for_inches(40, 0.5);
  drive_backward_for_inches(40, 0.5);
  drive_for_inches(40, 0.5);
  drive_backward_for_inches(40, 0.5);
  drive_for_inches(40, 0.5);
  drive_backward_for_inches(40, 0.5);
  drive_for_inches(40, 0.5);
  pros::delay(2500);

  // Shoot the gathered balls
  pros::delay(500);
  drive_backward_for_inches(50, 13);
  pros::delay(500);
  drive_backward_for_inches_async_nonblocking(80, 6);
  pros::delay(1000);

  discore.move_absolute(0, 200);
  catapult_arm.move_absolute(-600, 100);
  pros::delay(1500);
  catapult_arm.move_absolute(600, 100);
  pros::delay(250);
  catapult_arm.move_absolute(-600, 100);
  pros::delay(1500);
  catapult_arm.move_absolute(600, 100);
  intake.move_velocity(0);

  // 2nd part
  pros::delay(500);
  drive_for_inches(40, 3);
  discore.move_absolute(700, 200);
  pros::delay(250);
  turn_left_deg(129, 30, 500);
  pros::delay(500);
  drive_for_inches(80, 48.5);

  // Turn to face the matchload
  turn_right_deg(125, 30, 500);
  pros::delay(500);
  drive_backward_for_inches_async_nonblocking(80, 8);
  drive_for_inches(80, 5);
  drive_backward_for_inches_async_nonblocking(100, 8);

  pros::delay(250);

  // Gather matchload
  drive_for_inches(40, 9);
  wall_reset(4000);
  drive_backward_for_inches(40, 0.5);
  drive_for_inches(40, 0.5);
  pros::delay(350);

  // Intake balls
  discore.move_absolute(800, 200);
  intake.move_velocity(-200);
  drive_backward_for_inches(40, 0.5);
  drive_for_inches(40, 0.5);
  drive_backward_for_inches(40, 0.5);
  drive_for_inches(40, 0.5);
  drive_backward_for_inches(40, 0.5);
  drive_for_inches(40, 0.5);
  pros::delay(2500);

  // Shoot the gathered balls
  pros::delay(500);
  drive_backward_for_inches(40, 13);
  pros::delay(500);
  drive_backward_for_inches_async_nonblocking(80, 6);
  pros::delay(1000);

  discore.move_absolute(0, 200);
  catapult_arm.move_absolute(-600, 100);
  pros::delay(1500);
  catapult_arm.move_absolute(600, 100);
  pros::delay(250);
  catapult_arm.move_absolute(-600, 100);
  pros::delay(1500);
  catapult_arm.move_absolute(600, 100);
  intake.move_velocity(0);
}

void skills() {
  // Intake two balls
  intake.move_velocity(-200);
  discore.move_absolute(850, 200);
  drive_for_inches(80, 31.5);

  pros::delay(2000);

  drive_backward_for_inches(80, 1.5);
  pros::delay(400);

  // Turn left
  left_motor_group.move_velocity(-50);
  right_motor_group.move_velocity(50);
  pros::delay(420);
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  pros::delay(500);

  drive_for_inches(80, 35);
  pros::delay(500);
  matchloader.move_absolute(1400, 200);
  drive_for_inches_async_nonblocking(80, 5);

  intake.move_velocity(0);
  pros::delay(500);
  matchloader.move_absolute(0, 200);

  // Turn right to reset
  left_motor_group.move_velocity(100);
  right_motor_group.move_velocity(-100);
  pros::delay(500);
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  pros::delay(500);

  drive_backward_for_inches(80, 1);

  pros::delay(500);

  // Turn right to reset
  left_motor_group.move_velocity(50);
  right_motor_group.move_velocity(-50);
  pros::delay(510);
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  pros::delay(500);

  drive_for_inches_async_nonblocking(80, 5);

  // Turn left
  left_motor_group.move_velocity(-50);
  right_motor_group.move_velocity(50);
  pros::delay(450);
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  pros::delay(500);

  drive_backward_for_inches(60, 3);

  pros::delay(350);

  drive_backward_for_inches_async_nonblocking(100, 5);

  pros::delay(500);

  // shoot
  discore.move_absolute(0, 200);
  catapult_arm.move_absolute(-300, 200);
  catapult_arm.move_absolute(-600, 40);
  pros::delay(1500);
  catapult_arm.move_absolute(300, 200);
  matchloader.move_absolute(1400, 200);

  pros::delay(500);

  drive_for_inches(50, 3);

  pros::delay(350);

  drive_backward_for_inches_async_nonblocking(80, 7);

  pros::delay(500);

  drive_for_inches(50, 3);

  pros::delay(350);

  drive_backward_for_inches_async_nonblocking(80, 7);

  pros::delay(500);

  drive_for_inches(40, 9);
  pros::delay(350);
  drive_backward_for_inches(40, 2);
  drive_for_inches_async_nonblocking(40, 3);

  intake.move_velocity(-200);
  pros::delay(2500);

  pros::delay(200);

  drive_backward_for_inches(50, 14);

  pros::delay(500);

  // shoot
  discore.move_absolute(0, 200);
  catapult_arm.move_absolute(-300, 200);
  catapult_arm.move_absolute(-600, 40);
  pros::delay(1500);
  intake.move_velocity(0);
  catapult_arm.move_absolute(300, 200);

  pros::delay(500);

  drive_for_inches(60, 2);

  pros::delay(500);

  // Turn left
  left_motor_group.move_velocity(-50);
  right_motor_group.move_velocity(50);
  pros::delay(460);
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  pros::delay(500);

  drive_for_inches(80, 50);
  pros::delay(350);
  drive_for_inches_async_nonblocking(100, 10);

  pros::delay(500);

  drive_backward_for_inches(80, 8);

  // Turn right
  left_motor_group.move_velocity(50);
  right_motor_group.move_velocity(-50);
  pros::delay(460);
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  drive_for_inches(60, 3);
  drive_for_inches_async_nonblocking(100, 10);

  intake.move_velocity(-200);
  pros::delay(2500);
  intake.move_velocity(0);

  pros::delay(350);

  drive_backward_for_inches(80, 6);

  pros::delay(500);

  // shoot
  discore.move_absolute(0, 200);
  catapult_arm.move_absolute(-300, 200);
  catapult_arm.move_absolute(-600, 40);
  pros::delay(1500);
  matchloader.move_absolute(1400, 200);
  catapult_arm.move_absolute(300, 200);

  pros::delay(500);

  drive_for_inches(80, 3);
  matchloader.move_absolute(0, 200);

  pros::delay(500);

  // Turn left
  left_motor_group.move_velocity(-50);
  right_motor_group.move_velocity(50);
  pros::delay(700);
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  pros::delay(500);

  drive_for_inches(80, 45);

  pros::delay(500);

  // Turn left
  left_motor_group.move_velocity(-50);
  right_motor_group.move_velocity(50);
  pros::delay(500);
  left_motor_group.move_velocity(0);
  right_motor_group.move_velocity(0);

  pros::delay(500);

  drive_for_inches(80, 12);
  pros::delay(500);
  drive_for_inches(100, 15);
}

void test() {
  left_motor_group.move(80);
  pros::delay(1800);
  left_motor_group.move(0);
}

void park() {
  pros::delay(3000);
  intake.move_velocity(-200);
  matchloader.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  drive_backward_for_inches(80, 10);
  drive_for_inches(120, 25);
  matchloader.move_absolute(0, 100);
  pros::delay(7000);
}
// --- Autonomous ---
void autonomous() { skillsV2(); }
