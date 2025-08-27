#include "main.h"
#include "lemlib/api.hpp"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include <cmath>

ASSET(path_jerryio_txt);
// --- Robot state ---
double x = 0.0, y = 0.0, theta = 0.0;

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
pros::MotorGroup left_motor_group({-9, -10}, pros::MotorGears::green);
pros::MotorGroup right_motor_group({1, 2}, pros::MotorGears::green);

pros::MotorGroup conveyor({11, 12}, pros::MotorGears::green);

pros::Motor arm(16);

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

void initialize() {
  pros::lcd::initialize();
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
    double turnScale = 0.6;
    bool conveyorForward =
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    bool conveyorReverse =
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    bool clutch = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    // bool armUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    // bool armDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    bool level1Arm = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
    bool level2Arm = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
    bool level3Arm = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);

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

    if (clutch) {
      if (leftMotorSpeed < 0 && rightMotorSpeed < 0) {
        left_motor_group.move_velocity(leftMotorSpeed / 2);
        right_motor_group.move_velocity(rightMotorSpeed / 2);
      } else {
        left_motor_group.move_velocity(leftMotorSpeed / 1.5);
        right_motor_group.move_velocity(rightMotorSpeed / 1.5);
      }
    } else {
      left_motor_group.move_velocity(leftMotorSpeed);
      right_motor_group.move_velocity(rightMotorSpeed);
    }

    // Debug arm position control: level buttons map to preset angles (deg).
    // Pressing level1Arm -> 0°, level2Arm -> 45°, level3Arm -> 90°.
    {
      // Position targets
      double targetAngle = 0.0;
      bool haveTarget = false;
      if (level1Arm) {
        targetAngle = 0.0;
        haveTarget = true;
      } else if (level2Arm) {
        targetAngle = 45.0;
        haveTarget = true;
      } else if (level3Arm) {
        targetAngle = 90.0;
        haveTarget = true;
      }

      // Read current angle from the arm motor (degrees)
      double angleDeg = arm.get_position();

      // Compute raw velocity target using a simple P-controller when a target
      // is set
      int rawTarget = 0;
      if (haveTarget) {
        const double POS_KP = 8.0; // proportional gain (tune this)
        double error = targetAngle - angleDeg;
        double vel = POS_KP * error;
        // clamp to max vel
        if (vel > ARM_MAX_VEL)
          vel = ARM_MAX_VEL;
        if (vel < -ARM_MAX_VEL)
          vel = -ARM_MAX_VEL;
        rawTarget = static_cast<int>(vel);
      } else {
        rawTarget = 0; // no target -> hold
      }

      // Determine allowed target after angle checks (hard limits + safety
      // margin)
      double allowedTarget = rawTarget;
      if (rawTarget > 0 && angleDeg >= ARM_MAX_ANGLE) {
        allowedTarget = 0; // at/over positive limit
      } else if (rawTarget < 0 && angleDeg <= ARM_MIN_ANGLE) {
        allowedTarget = 0; // at/under negative limit
      } else if (rawTarget > 0 &&
                 angleDeg > (ARM_MAX_ANGLE - ARM_SAFETY_MARGIN)) {
        double factor =
            std::max(0.0, (ARM_MAX_ANGLE - angleDeg) / ARM_SAFETY_MARGIN);
        allowedTarget = static_cast<int>(rawTarget * factor);
      } else if (rawTarget < 0 &&
                 angleDeg < (ARM_MIN_ANGLE + ARM_SAFETY_MARGIN)) {
        double factor =
            std::max(0.0, (angleDeg - ARM_MIN_ANGLE) / ARM_SAFETY_MARGIN);
        allowedTarget = static_cast<int>(rawTarget * factor);
      }

      // Ramp (limit acceleration)
      static int currentDebugVel = 0; // persists across loop iterations
      int delta = static_cast<int>(allowedTarget) - currentDebugVel;
      if (delta > ARM_ACCEL_STEP)
        delta = ARM_ACCEL_STEP;
      if (delta < -ARM_ACCEL_STEP)
        delta = -ARM_ACCEL_STEP;
      currentDebugVel += delta;

      // Apply to arm motor
      arm.move_velocity(currentDebugVel);
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
