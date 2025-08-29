#include "main.h"
#include "lemlib/api.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>

ASSET(path_jerryio_txt);
// --- Robot state ---
double x = 0.0, y = 0.0, theta = 0.0;

const double ARM1_MIN = 0.0, ARM1_MAX = 90.0;   // arm1 limits
const double ARM2_MIN = 0.0, ARM2_MAX = 120.0;  // arm2 limits

inline double clampd(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

// persistent arm targets (in motor degrees)
static double arm1TargetDeg = 0.0;
static double arm2TargetDeg = 0.0;

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
pros::MotorGroup left_motor_group({-19, -20}, pros::MotorGears::green);
pros::MotorGroup right_motor_group({11, 12}, pros::MotorGears::green);

pros::Motor conveyor(16);
pros::Motor conveyor1(15);

pros::Motor arm1(18);
pros::Motor arm2(17);

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

void initialize() {
  pros::lcd::initialize();
  arm1.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  arm2.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
  arm1.tare_position();
  arm2.tare_position();
  arm1.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  arm2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  arm1TargetDeg = 0;
  arm2TargetDeg = 0;
  arm1.move_absolute(arm1TargetDeg, ARM_MAX_VEL);
  arm2.move_absolute(arm2TargetDeg, ARM_MAX_VEL);
  chassis.calibrate();
  pros::Task odoTask(odometryTask);
  pros::Task screen_task([&]() {
    while (true) {
      pros::lcd::print(0, "X: %.2f", x);
      pros::lcd::print(1, "Y: %.2f", y);
      pros::lcd::print(2, "Theta: %.2f", theta);
      pros::delay(20);
    }
  });
}

void opcontrol() {
  while (true) {
    // conveyor buttons
    double arm1Angle = 0;
    double arm2Angle = 0;

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

    bool level1Arm = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    bool level2Arm = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    bool level3Arm = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

    bool level1Arm2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    bool level2Arm2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
    bool level3Arm2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);

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

    // --- ARM PRESETS (button edges not required; holding is fine) ---
    if (level1Arm)      arm1TargetDeg = 0;
    else if (level2Arm) arm1TargetDeg = 45;
    else if (level3Arm) arm1TargetDeg = 90;

    if (level1Arm2)      arm2TargetDeg = 0;
    else if (level2Arm2) arm2TargetDeg = 90;
    else if (level3Arm2) arm2TargetDeg = 120;

    // clamp to safe ranges
    arm1TargetDeg = clampd(arm1TargetDeg, ARM1_MIN, ARM1_MAX);
    arm2TargetDeg = clampd(arm2TargetDeg, ARM2_MIN, ARM2_MAX);

    // command the motors each loop (OK to reissue the same target)
    arm1.move_absolute(arm1TargetDeg, ARM_MAX_VEL);
    arm2.move_absolute(arm2TargetDeg, ARM_MAX_VEL);

    if (conveyorForward && !conveyorReverse) {
      conveyor.move_velocity(200);
    } else if (conveyorReverse && !conveyorForward) {
      conveyor.move_velocity(-200);
    } else {
      conveyor.move_velocity(0);
    }

    if (conveyorForward1 && !conveyorReverse1) {
      conveyor1.move_velocity(200);
    } else if (conveyorReverse && !conveyorForward) {
      conveyor1.move_velocity(-200);
    } else {
      conveyor1.move_velocity(0);
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
