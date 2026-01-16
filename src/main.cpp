#include "lemlib/chassis/chassis.hpp"
#include "robot_config.hpp"
#include "helpers.hpp"
#include "tasks.hpp"
#include "autons.hpp"
#include "odometry.hpp"

// --- Initialize ---
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();

    // 2. Start the math engine
    pros::Task odom_task(odom_task_fn, NULL, "Odometry");
}

// --- Operator Control ---
void opcontrol() {
    catapultControls();       
   
}

// --- Autonomous ---
void autonomous() {

    driveForwardPID(48, 60, 3000);

    // drive_for_inches_consistent(80, 12);

    // pros::delay(1000);

    // turn_time_consistent(80, 679, true);

    // pros::delay(1000);

    // //Forward
    // matchloader.move_absolute(0, 100);
    // drive_for_inches_consistent(80, 27.2);

    // pros::delay(500);

    // turn_time_consistent(80, 679, true);

    // pros::delay(500);

    // //Move to matchload
    // intake.move_velocity(-200);
    // discore.move_absolute(-500, 200);
    // drive_for_inches_consistent(80, 8.5);
    // pros::delay(500);
    // pros::delay(2000);


    // drive_backward_consistent(80, 24.3);

    // pros::delay(1000);
    // discore.move_absolute(0, 200);
    // catapult_arm.move_absolute(-400, 100);
    // pros::delay(500);
    // catapult_arm.move_absolute(0, 100);
    // pros::delay(500);
    // catapult_arm.move_absolute(-400, 100);
    // pros::delay(500);
    // catapult_arm.move_absolute(0, 100);
    // intake.move_velocity(0);


    // drive_for_inches_consistent(80, 23);
    // pros::delay(500);

    // discore.move_absolute(-500, 200);
    // intake.move_velocity(-200);
    // pros::delay(2000);

    // drive_backward_consistent(150, 25);

    // pros::delay(1000);
    // discore.move_absolute(0, 200);
    // catapult_arm.move_absolute(-400, 100);
    // pros::delay(500);
    // catapult_arm.move_absolute(0, 400);
    // pros::delay(500);
    // catapult_arm.move_absolute(-400, 100);
    // pros::delay(500);
    // catapult_arm.move_absolute(0, 400);
    // intake.move_velocity(0);

    
    // // drive_for_inches_consistent(80, 7);

    // // drive_arc_consistent(80, 25, 30, false, false);
    // // drive_for_inches_consistent(120, 20);
    
}






