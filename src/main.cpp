#include "main.h"
#include "lemlib/api.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>


ASSET(path_jerryio_txt);
// --- Robot state ---
double x = 0.0, y = 0.0, theta = 0.0, heading = 0.0;

const double ARM1_MIN = -402;
const double ARM1_MAX = 0;
const double ARM1_LEVEL1_ANGLE = -10;   // degrees
const double ARM1_LEVEL2_ANGLE = -1350; // degrees  
const double ARM1_LEVEL3_ANGLE = -2200; // degrees

const double ARM2_MIN = 0.0;
const double ARM2_MAX = 220.0;
const double ARM2_LEVEL1_ANGLE = 0;    // degrees
const double ARM2_LEVEL2_ANGLE = 1300; // degrees 
const double ARM2_LEVEL3_ANGLE = 1500; // degrees


// --- Constants ---
const double wheelDiameter = 3.25; // inches
const double trackWidth = 12.0;    // distance between wheels
const double ticksPerRev = 360.0;  // depends on encoder resolution

const float PI = 3.14159;

// PID constants
float kP = 1.0;
float kI = 0.0;
float kD = 0.5;

// --- Debug motor limits (degrees and velocity) ---
const int ARM_MAX_VEL = 600;       // maximum motor velocity for debug motor
const double ARM_MIN_ANGLE = 0.0;  // minimum allowed angle (degrees)
const double ARM_MAX_ANGLE = 90.0; // maximum allowed angle (degrees)
const double ARM_SAFETY_MARGIN =
    15.0;                      // degrees within limit to start scaling down
const int ARM_ACCEL_STEP = 40; // maximum change in velocity per loop iteration

// left motor group
// left side: 7 and 6
pros::MotorGroup left_motor_group({-8, -2}, pros::MotorGears::green);
pros::MotorGroup right_motor_group({10, 9}, pros::MotorGears::green);

pros::Motor conveyor(3, pros::MotorGears::green);
pros::MotorGroup outtake({4, -5}, pros::MotorGears::blue);

pros::Motor arm1(13);
pros::Motor arm2(12);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group,          // left motor group
                              &right_motor_group,         // right motor group
                              10,                         // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              400,                        // drivetrain rpm
                              2                           // horizontal drift
);

// imu
pros::Imu imu(10);
// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(20);
// vertical tracking wheel encoder
pros::adi::Encoder vertical_encoder('C', 'D', true);

// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder,
                                                lemlib::Omniwheel::NEW_325,
                                                -5.75);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder,
                                              lemlib::Omniwheel::NEW_325, -2.5);

// odometry sensors
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical wheel
                            nullptr,                  // second vertical (none)
                            &horizontal_tracking_wheel, // horizontal wheel
                            nullptr, // second horizontal (none)
                            &imu     // imu
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, 0, 3, 3, 1, 100, 3, 500, 20);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, 0, 10, 0, 0, 0, 0, 0, 0);

lemlib::ExpoDriveCurve
    throttle_curve(3,    // joystick deadband out of 127
                   10,   // minimum output where drivetrain will move out of 127
                   1.019 // expo curve gain
    );

// input curve for steer input during driver control
lemlib::ExpoDriveCurve
    steer_curve(3,    // joystick deadband out of 127
                10,   // minimum output where drivetrain will move out of 127
                1.019 // expo curve gain
    );

// create the chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors, &throttle_curve, &steer_curve);

double prevLeft = 0.0;
double prevRight = 0.0;

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
  // Reset encoders
  left_motor_group.tare_position();
  right_motor_group.tare_position();
  prevLeft = 0;
  prevRight = 0;

  while (true) {
    updateOdometry();
    pros::delay(10); // update every 10ms
  }
}

void moveArm1ToAngle(double angle, int velocity = 100) {
  arm1.move_absolute(angle, velocity);
}

void moveArm2ToAngle(double angle, int velocity = 100) {
  arm2.move_absolute(angle, velocity);
}

float getHeadingCorrection(float desiredHeading) {
  float error = desiredHeading - heading;
  float derivative = error;
  return kP * error + kD * derivative;
}


void driveToPoint(float targetX, float targetY, float baseSpeed, int timeout = 5000) {
  int startTime = pros::millis();

  while (pros::millis() - startTime < timeout) {
    updateOdometry();

    float dx = targetX - x;
    float dy = targetY - y;
    float distance = sqrt(dx * dx + dy * dy);

    float targetHeading = atan2(dy, dx) * (180.0 / PI);
    float headingError = targetHeading - heading;
    while (headingError > 180) headingError -= 360;
    while (headingError < -180) headingError += 360;

    // Tunable gains
    float kP_drive = 5.0;
    float kP_turn = 2.0;

    float driveSpeed = std::clamp(distance * kP_drive, 30.0f, baseSpeed);
    float correction = kP_turn * headingError;

    left_motor_group.move(driveSpeed - correction);
    right_motor_group.move(driveSpeed + correction);

    if (distance < 1.0 && fabs(headingError) < 5.0) break;
    pros::delay(20);
  }

  left_motor_group.move(0);
  right_motor_group.move(0);
}


void turnToAngle(float targetHeading, float baseSpeed = 80, int timeout = 3000) {
  int start = pros::millis();

  while (pros::millis() - start < timeout) {
    updateOdometry();

    float headingError = targetHeading - heading;
    while (headingError > 180) headingError -= 360;
    while (headingError < -180) headingError += 360;

    if (fabs(headingError) < 2.0) break; // within tolerance

    float kP_turn = 2.0;
    float turnSpeed = std::clamp(headingError * kP_turn, -baseSpeed, baseSpeed);

    left_motor_group.move(-turnSpeed);
    right_motor_group.move(turnSpeed);

    pros::delay(20);
  }

  left_motor_group.move(0);
  right_motor_group.move(0);
}

void driveArc(float radius, float angle, float baseSpeed = 80, bool leftArc = true) {
  // arc length = radius * angle (in radians)
  float arcLength = radius * (angle * (PI / 180.0));

  // wheelbase width (distance between wheels) — set for your robot
  const float wheelbase = 25.0; // cm or whatever unit you use

  float innerArc = arcLength * (radius - (wheelbase / 2)) / radius;
  float outerArc = arcLength * (radius + (wheelbase / 2)) / radius;

  float ratio = innerArc / outerArc;

  if (leftArc) {
    left_motor_group.move(baseSpeed * ratio);
    right_motor_group.move(baseSpeed);
  } else {
    left_motor_group.move(baseSpeed);
    right_motor_group.move(baseSpeed * ratio);
  }

  // run for estimated duration (naive version)
  int duration = (arcLength / baseSpeed) * 1000; 
  pros::delay(duration);

  left_motor_group.move(0);
  right_motor_group.move(0);
}

void driveToPointBackward(float targetX, float targetY, float baseSpeed, int timeout = 5000) {
  int startTime = pros::millis();

  while (pros::millis() - startTime < timeout) {
    updateOdometry();

    float dx = targetX - x;
    float dy = targetY - y;
    float distance = sqrt(dx * dx + dy * dy);

    float targetHeading = atan2(dy, dx) * (180.0 / PI);
    float headingError = targetHeading - heading;
    while (headingError > 180) headingError -= 360;
    while (headingError < -180) headingError += 360;

    // flip direction for reverse driving
    headingError += 180;
    while (headingError > 180) headingError -= 360;
    while (headingError < -180) headingError += 360;

    float kP_drive = 5.0;
    float kP_turn = 2.0;

    float driveSpeed = -std::clamp(distance * kP_drive, 30.0f, baseSpeed); // negative speed
    float correction = kP_turn * headingError;

    left_motor_group.move(driveSpeed - correction);
    right_motor_group.move(driveSpeed + correction);

    if (distance < 1.0 && fabs(headingError) < 5.0) break;
    pros::delay(20);
  }

  left_motor_group.move(0);
  right_motor_group.move(0);
}

void driveForward(float distance, float maxSpeed) {
  // Compute target point based on current heading
  float targetX = x + distance * cos(heading * PI / 180.0);
  float targetY = y + distance * sin(heading * PI / 180.0);

  int startTime = pros::millis();
  int timeout = 5000; // safety timeout

  while (pros::millis() - startTime < timeout) {
    updateOdometry();

    float dx = targetX - x;
    float dy = targetY - y;
    float remaining = sqrt(dx * dx + dy * dy); // inches left

    if (remaining < 0.5) break; // stop when close

    // 🔹 Scale speed down as it nears target
    float kP = 20.0;                       // tune this gain
    float driveSpeed = std::min(remaining * kP, maxSpeed);
    driveSpeed = std::max(driveSpeed, 30.0f); // don’t go too slow

    left_motor_group.move(driveSpeed);
    right_motor_group.move(driveSpeed);

    pros::delay(20);
  }

  left_motor_group.move(0);
  right_motor_group.move(0);
}

void driveBackward(float distance, float maxSpeed) {
  // Compute target point behind the robot
  float targetX = x - distance * cos(heading * PI / 180.0);
  float targetY = y - distance * sin(heading * PI / 180.0);

  int startTime = pros::millis();
  int timeout = 5000; // safety timeout in ms

  while (pros::millis() - startTime < timeout) {
    updateOdometry();

    float dx = targetX - x;
    float dy = targetY - y;
    float remaining = sqrt(dx * dx + dy * dy); // inches left

    if (remaining < 0.5) break; // stop when close

    // 🔹 Scale speed down as it nears target
    float kP = 20.0;                       
    float driveSpeed = std::min(remaining * kP, maxSpeed);
    driveSpeed = std::max(driveSpeed, 30.0f); // minimum speed

    // Negative because we are driving backward
    left_motor_group.move(-driveSpeed);
    right_motor_group.move(-driveSpeed);

    pros::delay(20);
  }

  left_motor_group.move(0);
  right_motor_group.move(0);
}



void initialize() {
  pros::lcd::initialize();
  arm1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  arm2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  chassis.calibrate();

  // Make tasks static so they persist after initialize() returns.
  static pros::Task odoTask(odometryTask);

  static pros::Task screen_task([]() {
    while (true) {
      pros::lcd::print(0, "X: %.2f", x);
      pros::lcd::print(1, "Y: %.2f", y);
      pros::lcd::print(2, "Theta: %.2f", theta);
      pros::delay(20);
    }
  });
}

void opcontrol() {
  // previous button states for edge detection (persist across loops)
  static bool prevLevel1Arm = false;
  static bool prevLevel2Arm = false;
  static bool prevLevel3Arm = false;
  static bool prevLevel1Arm2 = false;
  static bool prevLevel2Arm2 = false;
  static bool prevLevel3Arm2 = false;

  while (true) {
    // conveyor buttons
    double arm1PrevSpeed;
    double arm2PrevSpeed;

    double Arm1Position = arm1.get_position();
    double Arm2Position = arm2.get_position();

    double turnScale = 0.6;
    bool conveyorForward =
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    bool conveyorReverse =
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    bool outtakeForward =
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool outtakeReverse =
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

    bool level1Arm = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
    bool level2Arm = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    bool level3Arm = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    bool autoa = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);


    bool level1Arm2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
    bool level2Arm2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
    bool level3Arm2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);

    // joystick values
    int move = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    int turn =
        controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) * turnScale;

    // combine forward/back + turning
    int leftMotorSpeed = move + turn;
    int rightMotorSpeed = move - turn;

    // clamp values so they don’t exceed -127 to 127
    leftMotorSpeed = std::clamp(leftMotorSpeed, -127, 127);
    rightMotorSpeed = std::clamp(rightMotorSpeed, -127, 127);

    // drive motors
    left_motor_group.move(leftMotorSpeed);
    right_motor_group.move(rightMotorSpeed);

    if(autoa) {
      autonomous();
    }

    // helper to clamp velocity
    auto clampVel = [](int vel) { return std::clamp(vel, -100, 100); };

    // --- ARM1 PRESET POSITION CONTROL (rising-edge taps) ---
    if (level1Arm && !prevLevel1Arm) {
      double target = ARM1_LEVEL1_ANGLE; // level1 -> min
      //target = std::clamp(target, ARM1_MIN, ARM1_MAX);
      moveArm1ToAngle(target, 150);
    }
    if (level2Arm && !prevLevel2Arm) {
      double target = ARM1_LEVEL2_ANGLE; // level2 -> mid
      //target = std::clamp(target, ARM1_MIN, ARM1_MAX);
      moveArm1ToAngle(target, 150);
    }
    if (level3Arm && !prevLevel3Arm) {
      double target = ARM1_LEVEL3_ANGLE; // level3 -> max
      //target = std::clamp(target, ARM1_MIN, ARM1_MAX);
      moveArm1ToAngle(target, 150);
    }

    // --- ARM2 PRESET POSITION CONTROL (rising-edge taps) ---
    if (level1Arm2 && !prevLevel1Arm2) {
      double target = ARM2_LEVEL1_ANGLE; // level1 -> min
      moveArm2ToAngle(target, 150);
    }
    if (level2Arm2 && !prevLevel2Arm2) {
      double target = ARM2_LEVEL2_ANGLE; // level2 -> mid
      moveArm2ToAngle(target, 150);
    }
    if (level3Arm2 && !prevLevel3Arm2) {
      double target = ARM2_LEVEL3_ANGLE; // level3 -> max
      moveArm2ToAngle(target, 150);
    }

    // update previous button states for next loop
    prevLevel1Arm = level1Arm;
    prevLevel2Arm = level2Arm;
    prevLevel3Arm = level3Arm;
    prevLevel1Arm2 = level1Arm2;
    prevLevel2Arm2 = level2Arm2;
    prevLevel3Arm2 = level3Arm2;

    if (outtakeForward && !outtakeReverse) {
      outtake.move_velocity(600);
    } else if (outtakeReverse && !outtakeForward) {
      outtake.move_velocity(-600);
    } else {
      outtake.move_velocity(0);
    }

    if (conveyorForward && !conveyorReverse) {
      conveyor.move_velocity(200);
    } else if (conveyorReverse && !conveyorForward) {
      conveyor.move_velocity(-200);
    } else {
      conveyor.move_velocity(0);
    }

    // conveyor motors
    pros::delay(20); // loop delay
  }
}

void autonomous() {
  // // Start facing 0°, at (0,0)
  // // 1. Drive forward to (50,30)
  // driveToPoint(50, 30, 100);
  // // 2. Turn right to face 90°
  // turnToAngle(90, 80);
  // // 3. Drive an arc left (like a smooth turn)
  // driveArc(40, 90, 70, true);
  // // 4. Back into target zone
  // driveToPointBackward(20, 20, 80);

  driveForward(15, 100);
  turnToAngle(45);
  driveForward(2, 100);
  outtake.move_velocity(600);
  pros::delay(2000);
  outtake.move_velocity(0);
  driveArc(40, 90, 70, true);
  outtake.move_velocity(-600);
  pros::delay(2000);
  outtake.move_velocity(0);
  turnToAngle(360);
  driveForward(12, 100);
  turnToAngle(45);
  driveForward(5, 100);
  outtake.move_velocity(600);
  pros::delay(3000);
  outtake.move_velocity(0);
  turnToAngle(360);
  driveForward(12, 100);
  moveArm1ToAngle(ARM1_LEVEL3_ANGLE, 150);
  outtake.move_velocity(-600);
  conveyor.move_velocity(-200);
  pros::delay(3000);
  
}

  
