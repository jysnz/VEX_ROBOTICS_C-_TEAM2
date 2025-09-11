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
const double ARM1_LEVEL2_ANGLE = -1450; // degrees  
const double ARM1_LEVEL3_ANGLE = -2400; // degrees

const double ARM2_MIN = 0.0;
const double ARM2_MAX = 220.0;
const double ARM2_LEVEL1_ANGLE = 0;    // degrees
const double ARM2_LEVEL2_ANGLE = 1300; // degrees 
const double ARM2_LEVEL3_ANGLE = 1500; // degrees


// --- Constants ---
const double wheelDiameter = 4; // inches
const double trackWidth = 12.0;    // distance between wheels
const double ticksPerRev = 360.0;  // depends on encoder resolution
const float PI = 3.14159;

// PID constants
static double kP = 0.45;    // distance P
static double kI = 0.0;     // distance I (often 0 at first)
static double kD = 2.0;     // distance D
static double kH = 0.12;    // heading/straightness P (encoders-only)

// geometry
static constexpr double WHEEL_DIAM_IN = 4;
static constexpr double TPI = 360.0 / (M_PI * WHEEL_DIAM_IN); // ticks per inch

static inline int clamp127(double v) { return (int)std::max(-127.0, std::min(127.0, v)); }

// --- Debug motor limits (degrees and velocity) ---
const int ARM_MAX_VEL = 600;       // maximum motor velocity for debug motor
const double ARM_MIN_ANGLE = 0.0;  // minimum allowed angle (degrees)
const double ARM_MAX_ANGLE = 90.0; // maximum allowed angle (degrees)
const double ARM_SAFETY_MARGIN =
    15.0;                      // degrees within limit to start scaling down
const int ARM_ACCEL_STEP = 40; // maximum change in velocity per loop iteration

// left motor group
// left side: 7 and 6
pros::MotorGroup left_motor_group({-9, -18}, pros::MotorGears::green);
pros::MotorGroup right_motor_group({4, 13}, pros::MotorGears::green);

pros::Motor conveyor(20);
pros::Motor conveyor1(1);
pros::Motor conveyor2(2);


pros::Motor arm1(13);
pros::Motor arm2(12);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group,          // left motor group
                              &right_motor_group,         // right motor group
                              10,                         // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              333,                        // drivetrain rpm
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
lemlib::OdomSensors sensors(nullptr, // vertical wheel
                            nullptr,                  // second vertical (none)
                            nullptr, // horizontal wheel
                            nullptr, // second horizontal (none)
                            nullptr     // imu
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


void driveToPoint(float targetX, float targetY, float baseSpeed) {
  while (true) {
    updateOdometry();

    float dx = targetX - x;
    float dy = targetY - y;
    float distance = sqrt(dx * dx + dy * dy);
    float targetHeading = atan2(dy, dx) * (180.0 / PI);

    float correction = getHeadingCorrection(targetHeading);

    left_motor_group.move(baseSpeed - correction);
    right_motor_group.move(baseSpeed + correction);

    if (distance < 1.0) break; // stop when close
    pros::delay(20);
  }

  left_motor_group.move(0);
  right_motor_group.move(0);
}

void turnByAngle(float angle, float turnSpeed = 30) {
  float targetHeading = heading + angle;

  // Normalize target heading to [0, 360)
  if (targetHeading >= 360) targetHeading -= 360;
  if (targetHeading < 0) targetHeading += 360;

  while (true) {
    updateOdometry();
    float error = targetHeading - heading;

    // Normalize error to shortest turn direction
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    if (fabs(error) < 2.0) break;

    float turnDirection = (error > 0) ? 1 : -1;
    left_motor_group.move(-turnSpeed * turnDirection);
    right_motor_group.move(turnSpeed * turnDirection);

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
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    bool conveyorReverse =
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    bool conveyorForward1 =
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    bool conveyorReverse1 =
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);

    bool auton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    bool level1Arm = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    bool level2Arm = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    bool level3Arm = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

    bool level1Arm2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
    bool level2Arm2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
    bool level3Arm2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);

    // joystick values
    int move = -controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    int turn =
        -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) * turnScale;

    // combine forward/back + turning
    int leftMotorSpeed = move + turn;
    int rightMotorSpeed = move - turn;

    // clamp values so they don’t exceed -127 to 127
    leftMotorSpeed = std::clamp(leftMotorSpeed, -127, 127);
    rightMotorSpeed = std::clamp(rightMotorSpeed, -127, 127);

    // drive motors
    left_motor_group.move(leftMotorSpeed);
    right_motor_group.move(rightMotorSpeed);

    if(auton) {
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

    if (conveyorForward1 && !conveyorReverse1) {
      conveyor1.move_velocity(200);
      conveyor2.move_velocity(-200);
    } else if (conveyorReverse1 && !conveyorForward1) {
      conveyor1.move_velocity(-200);
      conveyor2.move_velocity(200);
    } else {
      conveyor1.move_velocity(0);
      conveyor2.move_velocity(0);
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


void drive_forward(double inches, int maxSpeed = 110, int accel = 10, int timeout_ms = 4000) {
    const double targetTicks = inches * TPI;

    left_motor_group.tare_position();
    right_motor_group.tare_position();

    uint32_t start = pros::millis();
    const int dt_ms = 10;

    double errInt = 0.0;
    double prevErr = 0.0;

    double speed = 0; // current commanded speed

    while (true) {
        // get encoder avg
        double l = left_motor_group.get_position();
        double r = right_motor_group.get_position();
        double pos = (l + r) * 0.5;

        // error in ticks
        double err = targetTicks - pos;

        if (fabs(err) < 10.0) break; // within ~0.1–0.2 in
        if ((int)(pros::millis() - start) > timeout_ms) break;

        // integrate & derivative
        errInt += err * (dt_ms / 1000.0);
        errInt = std::clamp(errInt, -5000.0, 5000.0);

        double dErr = (err - prevErr) / (dt_ms / 1000.0);

        // ----------------------
        // Trapezoid motion profile
        // ----------------------
        if (fabs(speed) < maxSpeed) {
            speed += accel;
            if (speed > maxSpeed) speed = maxSpeed;
        }

        double decelDistance = (speed * speed) / (2.0 * accel);
        if (fabs(err) <= decelDistance) {
            speed -= accel;
            if (speed < 30) speed = 30; // don't drop below min rolling power
        }

        // ----------------------
        // PID correction (small trim)
        // ----------------------
        double pid = kP * err + kI * errInt + kD * dErr;

        // combine: profile sets base speed, PID trims it
        double forward = ((err > 0) ? speed : -speed) + pid;

        // clamp final command
        forward = std::clamp(forward, -(double)maxSpeed, (double)maxSpeed);

        // heading correction (basic drift fix)
        double headingErr = (l - r);
        double turn = kH * headingErr;

        int leftCmd  = clamp127(forward - turn);
        int rightCmd = clamp127(forward + turn);

        left_motor_group.move(leftCmd);
        right_motor_group.move(rightCmd);

        prevErr = err;
        pros::delay(dt_ms);
    }

    left_motor_group.move(0);
    right_motor_group.move(0);
}

void drive_curve(double inches, double curveRatio, int maxSpeed = 110, int accel = 10, int timeout_ms = 4000) {
    // curveRatio > 1.0 → turn left (right motors slower)
    // curveRatio < 1.0 → turn right (left motors slower)
    // curveRatio = 1.0 → straight

    const double targetTicks = inches * TPI;
    left_motor_group.tare_position();
    right_motor_group.tare_position();

    uint32_t start = pros::millis();
    const int dt_ms = 10;
    double pos = 0.0;

    while (true) {
        double l = left_motor_group.get_position();
        double r = right_motor_group.get_position();
        pos = (l + r) * 0.5;

        if (fabs(targetTicks - pos) < 10.0) break;
        if ((int)(pros::millis() - start) > timeout_ms) break;

        // basic speed ramp
        double speed = maxSpeed;
        if (targetTicks - pos < 500) speed = 50; // slow near end

        // apply curve ratio
        int leftCmd  = clamp127(speed);
        int rightCmd = clamp127(speed * curveRatio);

        left_motor_group.move(leftCmd);
        right_motor_group.move(rightCmd);

        pros::delay(dt_ms);
    }

    left_motor_group.move(0);
    right_motor_group.move(0);
}

void turnRight(int speed, int time) {
  left_motor_group.move(speed);
  right_motor_group.move(-speed);
  pros::delay(time);
  left_motor_group.move(0);
  right_motor_group.move(0);
}

void turnLeft(int speed, int time) {
  left_motor_group.move(-speed);
  right_motor_group.move(speed);
  pros::delay(time);
  left_motor_group.move(0);
  right_motor_group.move(0);
}

void backward(int speed, int time) {
  left_motor_group.move(-speed);
  right_motor_group.move(-speed);
  pros::delay(time);
  left_motor_group.move(0);
  right_motor_group.move(0);
}

void autonomous() {
  // First path
  drive_forward(24, 110, 5, 500);
  turnRight(100, 120);
  //Intake
  drive_curve(10, 1.3);
  //Outtake
  drive_forward(-24, 50, 5, 300);
  turnRight(100, 120);
  drive_forward(-24, 50, 5, 600);
  //Intake
  drive_forward(24, 50, 5, 600);
  //Outtake
}
