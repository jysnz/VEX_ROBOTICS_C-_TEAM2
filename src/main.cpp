#include "main.h"
#include "lemlib/api.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>


ASSET(path_jerryio_txt);
// --- Robot state ---
double x = 0.0, y = 0.0, theta = 0.0;

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
const double wheelDiameter = 3.25; // inches
const double trackWidth = 12.0;    // distance between wheels
const double ticksPerRev = 360.0;  // depends on encoder resolution

// --- Debug motor limits (degrees and velocity) ---
const int ARM_MAX_VEL = 600;       // maximum motor velocity for debug motor
const double ARM_MIN_ANGLE = 0.0;  // minimum allowed angle (degrees)
const double ARM_MAX_ANGLE = 90.0; // maximum allowed angle (degrees)
const double ARM_SAFETY_MARGIN =
    15.0;                      // degrees within limit to start scaling down
const int ARM_ACCEL_STEP = 40; // maximum change in velocity per loop iteration

// left motor group
// left side: 7 and 6
pros::MotorGroup left_motor_group({-8, -7}, pros::MotorGears::green);
pros::MotorGroup right_motor_group({10, 9}, pros::MotorGears::green);

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

void updateOdometry() {
  // Get current encoder positions (degrees)
  double leftDeg = left_motor_group.get_position();
  double rightDeg = right_motor_group.get_position();

  // Convert to "ticks"
  double leftTicks = (leftDeg / 360.0) * ticksPerRev;
  double rightTicks = (rightDeg / 360.0) * ticksPerRev;

  // Delta ticks
  double deltaLeftTicks = leftTicks - prevLeft;
  double deltaRightTicks = rightTicks - prevRight;

  prevLeft = leftTicks;
  prevRight = rightTicks;

  // Convert ticks → distance
  double distPerTick = (M_PI * wheelDiameter) / ticksPerRev;
  double dL = deltaLeftTicks * distPerTick;
  double dR = deltaRightTicks * distPerTick;

  // Kinematics
  double dTheta = (dR - dL) / trackWidth;
  double dS = (dR + dL) / 2.0;

  // Update global pose
  x += dS * cos(theta + dTheta / 2.0);
  y += dS * sin(theta + dTheta / 2.0);
  theta += dTheta;
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

static double normalizeDeg(double ang) {
  while (ang > 180.0) ang -= 360.0;
  while (ang < -180.0) ang += 360.0;
  return ang;
}
static double normalizeRad(double ang) {
  while (ang > M_PI) ang -= 2.0 * M_PI;
  while (ang < -M_PI) ang += 2.0 * M_PI;
  return ang;
}

void driveToWaypoint(double distanceInInches, int maxSpeed /*=100*/) {
  // target is distanceInInches forward from current (odometry) pose
  double startX = x;
  double startY = y;
  double startTheta = theta; // radians

  double targetX = startX + distanceInInches * cos(startTheta);
  double targetY = startY + distanceInInches * sin(startTheta);

  // PID-ish gains (tweak if needed)
  const double kP_dist = 5.0;     // maps inches -> motor speed unit
  const double kD_dist = 0.6;
  const double kP_heading = 30.0; // maps radians -> motor speed unit

  double prevErr = 0.0;
  double prevPosSum = 0.0;
  int recoveryAttempts = 0;

  while (true) {
    double dx = targetX - x;
    double dy = targetY - y;
    double distErr = sqrt(dx * dx + dy * dy);

    if (distErr < 0.5) break; // reached within 0.5 inch

    double pathAngle = atan2(dy, dx);
    double headingErr = normalizeRad(pathAngle - theta); // radians

    double speed = kP_dist * distErr + kD_dist * (distErr - prevErr);
    double correction = kP_heading * headingErr; // positive -> turn left

    // clamp base speed
    if (speed > maxSpeed) speed = maxSpeed;
    if (speed < -maxSpeed) speed = -maxSpeed;

    double leftSpeed = speed - correction;
    double rightSpeed = speed + correction;

    // clamp final speeds
    leftSpeed = std::clamp(static_cast<int>(leftSpeed), -maxSpeed, maxSpeed);
    rightSpeed = std::clamp(static_cast<int>(rightSpeed), -maxSpeed, maxSpeed);

    left_motor_group.move(static_cast<int>(leftSpeed));
    right_motor_group.move(static_cast<int>(rightSpeed));

    // simple stuck detection based on odometry progress
    double posSum = x + y;
    double delta = fabs(posSum - prevPosSum);
    prevPosSum = posSum;
    if (distErr > 3.0 && delta < 0.01) {
      recoveryAttempts++;
      if (recoveryAttempts > 4) {
        // back off briefly
        left_motor_group.move(-30);
        right_motor_group.move(-30);
        pros::delay(300);
        left_motor_group.move(0);
        right_motor_group.move(0);
        recoveryAttempts = 0;
      }
    } else {
      recoveryAttempts = 0;
    }

    prevErr = distErr;
    pros::delay(20);
  }

  left_motor_group.move(0);
  right_motor_group.move(0);
}

void driveArc(double distanceInInches, double radiusInInches, int maxSpeed /*=100*/) {
  // If radius is extremely large, fall back to straight drive
  if (std::fabs(radiusInInches) >= 1e6) {
    driveToWaypoint(distanceInInches, maxSpeed);
    return;
  }

  // arc angle (radians) = arc length / radius
  double s = distanceInInches;
  double R = radiusInInches;
  double deltaTheta = s / R; // radians; sign follows R

  // compute center of rotation and target pose
  // robot current pose (x,y,theta)
  double cx = x + R * sin(theta);   // center x (R to the left is + when R>0 considered right-turn, see below)
  double cy = y - R * cos(theta);   // center y
  // target theta
  double targetTheta = theta + deltaTheta;
  // target position computed by rotating start vector around center by deltaTheta
  double startRelX = x - cx;
  double startRelY = y - cy;
  double cosd = cos(deltaTheta);
  double sind = sin(deltaTheta);
  double targetRelX = startRelX * cosd - startRelY * sind;
  double targetRelY = startRelX * sind + startRelY * cosd;
  double targetX = cx + targetRelX;
  double targetY = cy + targetRelY;

  // simple proportional controllers for progress
  const double kP_pos = 5.0;
  const double kD_pos = 0.4;
  const double kP_heading = 25.0;

  double prevPosErr = 0.0;
  int recoveryAttempts = 0;
  double prevSum = x + y;

  while (true) {
    double dx = targetX - x;
    double dy = targetY - y;
    double posErr = sqrt(dx * dx + dy * dy);
    if (posErr < 0.6) break;

    double pathAngle = atan2(dy, dx);
    double headingErr = normalizeRad(pathAngle - theta);

    double baseSpeed = kP_pos * posErr + kD_pos * (posErr - prevPosErr);
    if (baseSpeed > maxSpeed) baseSpeed = maxSpeed;
    if (baseSpeed < -maxSpeed) baseSpeed = -maxSpeed;

    double correction = kP_heading * headingErr;

    double leftSpeed = baseSpeed - correction;
    double rightSpeed = baseSpeed + correction;

    leftSpeed = std::clamp(static_cast<int>(leftSpeed), -maxSpeed, maxSpeed);
    rightSpeed = std::clamp(static_cast<int>(rightSpeed), -maxSpeed, maxSpeed);

    left_motor_group.move(static_cast<int>(leftSpeed));
    right_motor_group.move(static_cast<int>(rightSpeed));

    // stuck detection via odometry
    double sum = x + y;
    double delta = fabs(sum - prevSum);
    prevSum = sum;
    if (posErr > 3.0 && delta < 0.01) {
      recoveryAttempts++;
      if (recoveryAttempts > 4) {
        left_motor_group.move(-30);
        right_motor_group.move(-30);
        pros::delay(350);
        left_motor_group.move(0);
        right_motor_group.move(0);
        recoveryAttempts = 0;
      }
    } else {
      recoveryAttempts = 0;
    }

    prevPosErr = posErr;
    pros::delay(20);
  }

  left_motor_group.move(0);
  right_motor_group.move(0);
}

void turnToAngle(double targetDegrees, int maxSpeed /*=80*/) {
  // Use odometry theta (radians -> degrees)
  auto toDeg = [](double r) { return r * 180.0 / M_PI; };
  auto toRad = [](double d) { return d * M_PI / 180.0; };

  double prevErr = 0.0;
  int recoveryAttempts = 0;
  double prevThetaDeg = toDeg(theta);

  // PID-ish gains
  const double kP = 1.6;   // maps degrees -> motor speed
  const double kD = 0.12;

  while (true) {
    double currentDeg = toDeg(theta);
    double error = normalizeDeg(targetDegrees - currentDeg);

    if (fabs(error) < 1.5) break;

    double speed = kP * error + kD * (error - prevErr);
    if (speed > maxSpeed) speed = maxSpeed;
    if (speed < -maxSpeed) speed = -maxSpeed;

    // positive speed -> turn right (left negative, right positive)
    left_motor_group.move(static_cast<int>(-speed));
    right_motor_group.move(static_cast<int>(speed));

    // simple progress detection using odometry theta change
    if (fabs(error) > 8.0) {
      double deltaThetaDeg = fabs(currentDeg - prevThetaDeg);
      if (deltaThetaDeg < 0.2) {
        recoveryAttempts++;
        if (recoveryAttempts > 4) {
          // brief reverse attempt
          left_motor_group.move(30 * (error > 0 ? -1 : 1));
          right_motor_group.move(30 * (error > 0 ? 1 : -1));
          pros::delay(200);
          recoveryAttempts = 0;
        }
      } else {
        recoveryAttempts = 0;
      }
      prevThetaDeg = currentDeg;
    } else {
      recoveryAttempts = 0;
    }

    prevErr = error;
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

void autonomous() {
  // First path
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(23.148, 24.132, 2000);
  conveyor.move_velocity(200);
  pros::delay(1000);
  conveyor.move_velocity(0);

  // Second path
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(6.247, 43.175, 2000);
}
