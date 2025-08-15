#include "main.h"
#include "lemlib/api.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

// left motor group
pros::MotorGroup left_motor_group({-11, 12}, pros::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({13, -14}, pros::MotorGears::green);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group,        // left motor group
                              &right_motor_group,       // right motor group
                              10,                       // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              360,                      // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(10);
// horizontal tracking wheel encoder
pros::Rotation horizontal_encoder(20);
// vertical tracking wheel encoder
pros::adi::Encoder vertical_encoder('C', 'D', true);
// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder,
                                                lemlib::Omniwheel::NEW_275,
                                                -5.75);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder,
                                              lemlib::Omniwheel::NEW_275, -2.5);

// odometry settings
lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    &horizontal_tracking_wheel, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a
             // second one
    &imu     // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(10,  // proportional gain (kP)
                       0,   // integral gain (kI)
                       3,   // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in inches
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in inches
                       500, // large error range timeout, in milliseconds
                       20   // maximum acceleration (slew)
    );

// angular PID controller
lemlib::ControllerSettings
    angular_controller(2,  // proportional gain (kP)
                       0,  // integral gain (kI)
                       10, // derivative gain (kD)
                       0,  // anti windup
                       0,  // small error range, in inches
                       0,  // small error range timeout, in milliseconds
                       0,  // large error range, in inches
                       0,  // large error range timeout, in milliseconds
                       0   // maximum acceleration (slew)
    );

// create the chassis
lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors             // odometry sensors
);

// initialize function. Runs on program startup
void initialize() {
  pros::lcd::initialize(); // initialize brain screen
  chassis.calibrate();     // calibrate sensors
  // print position to brain screen
  pros::Task screen_task([&]() {
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
      // delay to save resources
      pros::delay(20);
    }
  });
}

void opcontrol() {
  int speed = 0;       // forward/backward speed
  int rampCounter = 0; // how long button is held

  while (true) {
    bool r1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool r2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);

    // Get turning value from X-axis (left stick horizontal)
    int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

    // If both pressed → stop
    if (r1 && r2) {
      speed = 0;
      rampCounter = 0;
    }
    // Forward acceleration
    else if (r1) {
      rampCounter++;
      speed += (1 + rampCounter / 10);
    }
    // Reverse acceleration
    else if (r2) {
      rampCounter++;
      speed -= (1 + rampCounter / 10);
    }
    // No button pressed → slow down
    else {
      rampCounter = 0;
      if (speed > 0)
        speed -= 1;
      else if (speed < 0)
        speed += 1;
    }

    // Clamp speed
    if (speed > 127)
      speed = 127;
    if (speed < -127)
      speed = -127;

    // Tank drive with turning
    int leftMotorSpeed = speed + turn;
    int rightMotorSpeed = speed - turn;

    // Clamp again to avoid exceeding limits
    if (leftMotorSpeed > 127)
      leftMotorSpeed = 127;
    if (leftMotorSpeed < -127)
      leftMotorSpeed = -127;
    if (rightMotorSpeed > 127)
      rightMotorSpeed = 127;
    if (rightMotorSpeed < -127)
      rightMotorSpeed = -127;

    chassis.tank(leftMotorSpeed, rightMotorSpeed);

    pros::delay(20);
  }
}

void autonomous() {
  // set position to x:0, y:0, heading:0
  chassis.setPose(0, 0, 0);
  // turn to face heading 90 with a very long timeout
  chassis.turnToHeading(90, 100000);
}

// Single stick arcade drive
//  void opcontrol() {
//      // loop forever
//      while (true) {
//          // get left y and right x positions
//          int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
//          int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

//         // move the robot
//         chassis.arcade(leftY, leftX);

//         // delay to save resources
//         pros::delay(25);
//     }
// }

// Double stick arcade drive
//  void opcontrol() {
//      // loop forever
//      while (true) {
//          // get left y and right x positions
//          int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
//          int rightX =
//          controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

//         // move the robot
//         chassis.arcade(leftY, rightX);

//         // delay to save resources
//         pros::delay(25);
//     }
// }

// Throttle steer priority
//  void opcontrol() {
//      // loop forever
//      while (true) {
//          // get left y and right x positions
//          int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
//          int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

//         // move the robot
//         // prioritize steering slightly
//         chassis.arcade(leftY, leftX, false, 0.75);

//         // delay to save resources
//         pros::delay(25);
//     }
// }

// Curvature Drive

// void opcontrol() {
//     // loop forever
//     while (true) {
//         // get left y and right x positions
//         int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
//         int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

//         // move the robot
//         chassis.curvature(leftY, leftX);

//         // delay to save resources
//         pros::delay(25);
//     }
// }

// Double stick curvature
//  void opcontrol() {
//      // loop forever
//      while (true) {
//          // get left y and right x positions
//          int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
//          int rightX =
//          controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

//         // move the robot
//         chassis.curvature(leftY, rightX);

//         // delay to save resources
//         pros::delay(25);
//     }
// }
