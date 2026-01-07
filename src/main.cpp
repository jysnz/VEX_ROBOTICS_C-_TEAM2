#include "lemlib/chassis/chassis.hpp"
#include "robot_config.hpp"
#include "helpers.hpp"
#include "tasks.hpp"
#include "autons.hpp"

// --- Initialize ---
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    matchloader.move_absolute(-1700, 100);
    pros::Task odo(odometryTask);
}

// --- Operator Control ---
void opcontrol() {
    startTuningUI();
    const int MAX_SPEED = 127;
    static bool controlsReversed = false;

    while (true) {

        // --- 1. TOGGLE MODE (B Button) ---
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            currentMode = (currentMode == LATERAL) ? ANGULAR : LATERAL;
            controller.rumble(currentMode == LATERAL ? "." : "-");
        }

        // --- 2. SELECT TEST (L1 + Up/Down) ---
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) test_index = (test_index + 1) % 5;
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) test_index = (test_index - 1 + 5) % 5;
        } 
        else {
            // --- 3. TUNE CONSTANTS ---
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))    tune_kp += 0.5;
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))  tune_kp -= 0.5;
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) tune_kd += 1.0;
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))  tune_kd -= 1.0;
        }

        // Tune Start I (R1/R2)
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) tune_start_i += 0.5;
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) tune_start_i -= 0.5;

        // --- 4. TRIGGER TEST (Button X) ---
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            chassis.setPose(0, 0, 0);
            
            if (currentMode == LATERAL) {
                // --- LATERAL SEQUENCE: 2 Steps Forward, then Back to Start ---
                // Step 1: Forward 12 inches
                chassis.moveToPoint(0, 12, 1500, {.forwards = true});
                // Step 3: Back to 0
                chassis.moveToPoint(0, 0, 2000, {.forwards = false});
            } 
            else {
                // --- ANGULAR SEQUENCE: Rotate 360, then back to 0 ---
                // Step 1: Full rotation to 360 degrees
                chassis.turnToHeading(360, 3000, {}); // Added empty params {} to fix red line
                chassis.waitUntilDone();
                pros::delay(200);

                // Step 2: Rotate back to the original heading (0 degrees)
                chassis.turnToHeading(0, 3000, {});
            }
        }
        pros::delay(20);

        

        
        // bool intakeForward = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
        // bool intakeReverse = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        // bool intakePause = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        // bool auton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);

        // bool catapultArm = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        // bool discoreDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        // bool discoreUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);

        // bool matchLoadUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B); 
        // bool matchLoadDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

        // bool reverseControlTap = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN);

        // int move = -controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        // int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // if (reverseControlTap) controlsReversed = !controlsReversed;
        // if(auton) autonomous();

        // if (controlsReversed) move = -move;

        // int leftMotorSpeed = std::clamp(move + turn, -MAX_SPEED, MAX_SPEED);
        // int rightMotorSpeed = std::clamp(move - turn, -MAX_SPEED, MAX_SPEED);

        // left_motor_group.move(leftMotorSpeed);
        // right_motor_group.move(rightMotorSpeed);

        // if (catapultArm){
        //     catapult_arm.move_absolute(-600, 400);
        //     discore.move_absolute(0, 200); 
        // } 
        // else catapult_arm.move_absolute(0, 400);

        // if (discoreDown) {
        //     discore.move_absolute(0, 200);
        // }
        // else if (discoreUp) discore.move_absolute(-500, 200);

        // if (matchLoadUp && !matchLoadDown)     matchloader.move_absolute(0, 100);
        // else if (matchLoadDown && !matchLoadUp) matchloader.move_absolute(-1700, 100);

        // if (intakeForward && !intakeReverse) intake.move_velocity(200);
        // else if (intakeReverse && !intakeForward) intake.move_velocity(-200);
        // if (intakePause) intake.move_velocity(0);

        // pros::delay(20);
    }
}

// --- Autonomous ---
void autonomous() {
    hardCoded2v2Auton();

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






