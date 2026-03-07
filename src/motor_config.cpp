#include "motor_config.h"
#include "constants.h"

// ============== DRIVETRAIN MOTORS ==============
pros::MotorGroup left_motor_group({-4, -3, -1}, pros::MotorGears::green);
pros::MotorGroup right_motor_group({10, 8, 9}, pros::MotorGears::green);

// ============== MECHANISM MOTORS ==============
pros::Motor catapult_arm(13, pros::MotorGears::red);
pros::Motor intake(11, pros::MotorGears::green);
pros::Motor intake2(12, pros::MotorGears::green);
pros::Motor intake3(7, pros::MotorGears::green);
pros::Motor matchloader(14, pros::MotorGears::red);
pros::Motor discore(17, pros::MotorGears::green);
pros::Motor switchScore(19, pros::MotorGears::green);

// ============== SENSORS ==============
pros::Imu imu(19);
pros::Rotation horizontal_encoder(20);
pros::adi::Encoder vertical_encoder('C', 'D', true);

// ============== DRIVER CONTROLLER ==============
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ============== LEMLIB CHASSIS COMPONENTS ==============
lemlib::Drivetrain drivetrain(&left_motor_group,               // left motor group
                              &right_motor_group,              // right motor group
                              10,                              // 10 inch track width
                              lemlib::Omniwheel::NEW_325,       // using new 4" omnis
                              400,                             // drivetrain rpm
                              2                                // horizontal drift
);

// Tracking wheels
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder,
                                                lemlib::Omniwheel::NEW_325,
                                                -5.75);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder,
                                              lemlib::Omniwheel::NEW_325, -2.5);

// Odometry sensors
lemlib::OdomSensors sensors(&vertical_tracking_wheel,     // vertical wheel
                            nullptr,                      // second vertical (none)
                            &horizontal_tracking_wheel,   // horizontal wheel
                            nullptr,                      // second horizontal (none)
                            &imu                          // imu
);

// Lateral & Angular PID controllers
lemlib::ControllerSettings lateral_controller(10, 0, 3, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angular_controller(2, 0, 10, 0, 0, 0, 0, 0, 0);

// Expo drive curves
lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.019);
lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);

// Main chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors, &throttle_curve, &steer_curve);

// ============== INITIALIZATION ==============
double Drivetrain::TURN_CALIBRATION = 1.80;

void initialize_motors() {
    // Motor brake modes will be set in initialize() function
}
