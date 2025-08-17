#include "main.h"
#include "lemlib/api.hpp"
#include "pros/misc.h"

// left motor group
// left side: 7 and 6
pros::MotorGroup left_motor_group({-18, -17}, pros::MotorGears::green);
pros::MotorGroup right_motor_group({13, 14}, pros::MotorGears::green);

// conveyor motors
pros::MotorGroup conveyor({1, -2}, pros::MotorGears::red);
pros::Motor debug(11);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group,        // left motor group
                              &right_motor_group,       // right motor group
                              10,                       // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              360,                      // drivetrain rpm
                              2                         // horizontal drift
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

// chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors);

void initialize() {
  pros::lcd::initialize();
  chassis.calibrate();
  pros::Task screen_task([&]() {
    while (true) {
      pros::lcd::print(0, "X: %f", chassis.getPose().x);
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
      pros::delay(20);
    }
  });
}

void opcontrol() {
  int speed = 0;
  int rampCounter = 0;

  while (true) {
    // buttons for forward/backward
    bool r1 =
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1); // drive forward
    bool r2 =
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2); // drive reverse

    // conveyor buttons
    bool convForward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    bool convReverse = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    bool debugM = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    // analog stick for turning
    int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

    // --- Forward/Reverse speed logic ---
    if (r1 && r2) {
      speed = 0;
      rampCounter = 0;
    } else if (r1) {
      rampCounter++;
      speed += (1 + rampCounter / 10);
    } else if (r2) {
      rampCounter++;
      speed -= (1 + rampCounter / 10);
    } else {
      rampCounter = 0;
      if (speed > 0)
        speed -= 1;
      else if (speed < 0)
        speed += 1;
    }

    // limit speed range
    if (speed > 127)
      speed = 127;
    if (speed < -127)
      speed = -127;

    // --- Combine forward/backward with turning ---
    int leftMotorSpeed = speed + turn;  // turn adds differential
    int rightMotorSpeed = speed - turn; // turn subtracts differential

    // clip values
    if (leftMotorSpeed > 127)
      leftMotorSpeed = 127;
    if (leftMotorSpeed < -127)
      leftMotorSpeed = -127;
    if (rightMotorSpeed > 127)
      rightMotorSpeed = 127;
    if (rightMotorSpeed < -127)
      rightMotorSpeed = -127;

    // drive motors
    left_motor_group.move(leftMotorSpeed);
    right_motor_group.move(rightMotorSpeed);

    if (turn) {
      // turn motors
      left_motor_group.move(leftMotorSpeed);
      right_motor_group.move(rightMotorSpeed);
    } else {
      // straight motors    
    }

    // conveyor motors
    if (convForward) {
      conveyor.move(127);
    } else if (convReverse) {
      conveyor.move(-127);
    } else {
      conveyor.move(0);
    }

    if (debugM) {
      debug.move(127);
    } else {
      debug.move(0);
    }

    pros::delay(20);
  }
}

void autonomous() {
  chassis.setPose(0, 0, 0);
  chassis.turnToHeading(90, 2000); // 2 second timeout
}
