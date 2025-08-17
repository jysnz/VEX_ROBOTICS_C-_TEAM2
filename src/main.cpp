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
lemlib::Drivetrain drivetrain(&left_motor_group,          // left motor group
                              &right_motor_group,         // right motor group
                              10,                         // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              180,                        // drivetrain rpm
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
  while (true) {
    // conveyor buttons
    bool convForward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    bool convReverse = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

    // joystick values
    int move = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // combine forward/back + turning
    int leftMotorSpeed = move + turn;
    int rightMotorSpeed = move - turn;

    // clamp values so they don’t exceed -127 to 127
    leftMotorSpeed = std::clamp(leftMotorSpeed, -127, 127);
    rightMotorSpeed = std::clamp(rightMotorSpeed, -127, 127);

    // drive motors
    left_motor_group.move(leftMotorSpeed);
    right_motor_group.move(rightMotorSpeed);

    // conveyor motors
    if (convForward) {
      conveyor.move(127);
    } else if (convReverse) {
      conveyor.move(-127);
    } else {
      conveyor.move(0);
    }

    pros::delay(20); // loop delay
  }
}

void autonomous() {
  chassis.setPose(0, 0, 0);
  chassis.turnToHeading(90, 2000); // 2 second timeout
}
