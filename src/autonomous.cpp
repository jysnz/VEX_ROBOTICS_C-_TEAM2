#include "autonomous.h"
#include "driving.h"
#include "mechanisms.h"
#include "motor_config.h"

// ============== TWO VS TWO ROUTINE ==============
void twoVtwo() {
    catapult_arm.tare_position();
    matchloader.tare_position();
    discore.tare_position();

    // Go to matchload
    matchloader.move_absolute(1400, 200);
    intake.move_velocity(-200);
    discore.move_absolute(850, 200);
    drive_for_inches(80, 25);

    pros::delay(500);

    // Turn to matchload
    turn_right_deg(120, 30, 500);

    // Long goal reset and shoot
    wall_reset_v2(7000, 200, -1);
    pros::delay(500);
    discore.move_absolute(0, 200);
    catapultShootForAuto(100);

    // Long goal reset
    drive_for_inches(80, 3);
    discore.move_absolute(850, 200);
    wall_reset_v2(10000, 200, -1);
    drive_for_inches(80, 3);
    wall_reset_v2(10000, 200, -1);
    pros::delay(500);

    // Gather matchload
    drive_for_inches(40, 10);
    wall_reset_v2(5000, 250);
    pros::delay(500);
    drive_backward_for_inches(80, .5);
    wall_reset_v2(4000, 100);

    // Drive back to long goal
    drive_backward_for_inches(40, 10);
    wall_reset_v2(8000, 200, -1);
    pros::delay(500);
    matchloader.move_absolute(0, 200);

    // Shoot
    discore.move_absolute(0, 200);
    pros::delay(500);
    catapultShootForAuto(100);
}

// ============== TWO VS TWO WITH MATCHLOAD ==============
void twovtwoWithMatchload() {
    catapult_arm.tare_position();
    matchloader.tare_position();
    discore.tare_position();

    // Go to matchload
    matchloader.move_absolute(1400, 200);
    discore.move_absolute(850, 200);
    drive_for_inches(80, 24.5);

    pros::delay(500);

    // Turn to matchload
    turn_right_deg(120, 30, 500);

    // Long goal reset
    drive_for_inches(80, 3);
    wall_reset_v2(10000, 200, 1);
    pros::delay(500);

    // Gather matchload
    intake.move_velocity(-200);
    pros::delay(400);
    drive_backward_for_inches(80, .5);
    wall_reset_v2(4000);

    // Drive back to long goal
    drive_backward_for_inches(40, 10);
    turn_left_deg(10, 50, 250);
    wall_reset_v2(8000, 200, -1);
    pros::delay(500);

    // Shoot
    discore.move_absolute(0, 200);
    pros::delay(500);
    catapultShootForAuto(100);
    pros::delay(250);

    // Long goal reset
    drive_for_inches(80, 2);
    wall_reset_v2(8000, 200, -1);

    // Gather matchload
    intake.move_velocity(-200);
    drive_for_inches(50, 16);
    pros::delay(300);
    drive_backward_for_inches(80, .5);
    wall_reset_v2(4000, 0);

    drive_backward_for_inches(80, 2);
    turn_right_deg(30, 70, 500);
    catapultShootForAuto(100);
    pros::delay(500);
    turn_left_deg(50, 70, 500);
    drive_for_inches(80, 2);

    wall_reset_v2(4000, 100, 1);
    pros::delay(4000);

    // Drive back to long goal
    drive_backward_for_inches(40, 10);
    turn_left_deg(10, 50, 250);
    wall_reset_v2(8000, 200, -1);
    pros::delay(500);

    // Shoot
    catapultShootForAuto(100);
    matchloader.move_absolute(0, 200);
    pros::delay(250);
}

// ============== SKILLS V3 ==============
void skillsV3() {
    catapult_arm.tare_position();
    matchloader.tare_position();
    discore.tare_position();

    intake.move_velocity(-200);
    discore.move_absolute(850, 200);
    drive_for_inches(80, 29);
    wall_reset_v2(7000);
    drive_backward_for_inches(80, 6);
    turn_right_deg(120, 30, 500);
    pros::delay(500);

    // Long goal reset
    wall_reset_v2(7000, 200, -1);
    pros::delay(500);

    // Shoot
    discore.move_absolute(0, 200);
    catapultShootForAuto(200);
    pros::delay(500);

    discore.move_absolute(850, 200);
    matchloader.move_absolute(1400, 200);

    // Long goal reset
    wall_reset_v2(7000, 200, -1);
    pros::delay(500);

    // Long goal reset
    drive_for_inches(80, 3);
    wall_reset_v2(10000, 200, -1);
    drive_for_inches(80, 3);
    wall_reset_v2(10000, 200, -1);
    pros::delay(500);

    // Gather matchload
    drive_for_inches(50, 15);
    pros::delay(350);
    drive_backward_for_inches(80, .5);
    wall_reset_v2(4000);
    drive_backward_for_inches(80, .5);
    wall_reset_v2(4000);
    drive_backward_for_inches(80, .5);
    wall_reset_v2(4000);
    drive_backward_for_inches(80, .5);
    wall_reset_v2(4000);
    drive_backward_for_inches(80, .5);
    wall_reset_v2(4000);
    drive_backward_for_inches(80, .5);
    wall_reset_v2(4000, 200);

    // Drive back to long goal
    drive_backward_for_inches(40, 10);
    turn_left_deg(10, 50, 250);
    wall_reset_v2(8000, 200, -1);
    pros::delay(500);

    // Shoot
    discore.move_absolute(0, 200);
    pros::delay(500);
    catapultShootForAuto(200);
    matchloader.move_absolute(0, 200);
    pros::delay(250);

    // Long goal reset
    drive_for_inches(80, 2);
    wall_reset_v2(8000, 200, -1);
    drive_for_inches(80, 2);
    wall_reset_v2(8000, 200, -1);

    pros::delay(500);

    // Park
    discore.move_absolute(850, 200);
    drive_for_inches(80, 9);
    turn_right_deg(55, 40, 500);
    drive_for_inches(150, 30);
}

// ============== SKILLS V2 ==============
void skillsV2() {
    intake.move_velocity(-200);
    discore.move_absolute(850, 200);
    drive_for_inches(120, 29);
    wall_reset_v2(7000);
    drive_backward_for_inches(80, 1.5);

    turn_left_deg(60, 50, 250);

    drive_for_inches(150, 42);
    pros::delay(250);

    // Throw away the red balls
    pros::delay(350);
    drive_for_inches(80, 2);

    // Turn to wall
    turn_right_deg(90, 90, 200);
    drive_backward_for_inches(80, 1.5);
    pros::delay(500);
    turn_right_deg(50, 50, 250);

    // Wall reset
    wall_reset_v2();
    pros::delay(250);

    // Drive backwards for turn
    drive_backward_for_inches(40, 7.4);
    pros::delay(500);

    // Turn to face the long goal
    turn_left_deg(120, 30, 250);

    // Drive backward to long goal
    wall_reset_v2(6000, 200, -1);
    pros::delay(500);

    // Shoot
    discore.move_absolute(0, 200);
    catapult_arm.move_absolute(-300, 200);
    catapult_arm.move_absolute(-600, 40);
    pros::delay(500);
    catapult_arm.move_absolute(600, 200);
    pros::delay(500);
    catapult_arm.move_absolute(-300, 200);
    catapult_arm.move_absolute(-600, 40);
    pros::delay(500);
    catapult_arm.move_absolute(600, 200);
    pros::delay(500);
    matchloader.move_absolute(1400, 200);

    pros::delay(250);

    // Align the robot to the long goal
    drive_for_inches(50, 2);
    wall_reset_v2(8000, 200, -1);
    pros::delay(500);

    // Gather matchload
    drive_for_inches(40, 9.8);
    pros::delay(350);

    // Intake balls
    discore.move_absolute(800, 200);
    drive_backward_for_inches(40, 0.3);
    wall_reset_v2(6000, 1000);
    drive_backward_for_inches(40, 0.3);
    wall_reset_v2(6000, 1000);
    drive_backward_for_inches(40, 0.3);
    wall_reset_v2(6000, 1000);
    drive_backward_for_inches(40, 0.3);
    wall_reset_v2(6000, 1000);
    drive_backward_for_inches(40, 0.3);
    wall_reset_v2(6000, 1000);

    // Shoot the gathered balls
    pros::delay(500);
    drive_backward_for_inches(50, 13);
    wall_reset_v2(8000, 200, -1);
    pros::delay(1000);

    discore.move_absolute(0, 200);
    pros::delay(500);
    catapult_arm.move_absolute(-600, 100);
    pros::delay(250);
    catapult_arm.move_absolute(600, 100);
    pros::delay(250);
    catapult_arm.move_absolute(-600, 100);
    pros::delay(250);
    catapult_arm.move_absolute(600, 100);
    pros::delay(250);
    intake.move_velocity(0);

    pros::delay(500);

    // 2nd half
    drive_for_inches(100, 3);
    turn_right_deg(90, 40, 250);
    wall_reset_v2();

    drive_backward_for_inches(80, 53);

    // Turn to face the matchload
    turn_left_deg(125, 30, 250);
    wall_reset_v2(8000, 200, -1);
    drive_for_inches(80, 5);
    wall_reset_v2(8000, 200, -1);

    pros::delay(250);

    // Gather matchload
    drive_for_inches(40, 9.8);
    pros::delay(350);

    // Intake balls
    discore.move_absolute(800, 200);
    intake.move_velocity(-200);
    drive_backward_for_inches(40, 0.3);
    wall_reset_v2(6000, 100);
    drive_backward_for_inches(40, 0.3);
    wall_reset_v2(6000, 100);
    drive_backward_for_inches(40, 0.3);
    wall_reset_v2(6000, 100);
    drive_backward_for_inches(40, 0.3);
    wall_reset_v2(6000, 100);
    drive_backward_for_inches(40, 0.3);
    wall_reset_v2(6000, 1000);

    // Shoot the gathered balls
    pros::delay(500);
    drive_backward_for_inches(50, 13);
    wall_reset_v2(8000, 200, -1);
    pros::delay(1000);

    discore.move_absolute(0, 200);
    pros::delay(500);
    catapult_arm.move_absolute(-600, 100);
    pros::delay(750);
    catapult_arm.move_absolute(600, 100);
    pros::delay(750);
    catapult_arm.move_absolute(-600, 100);
    pros::delay(750);
    catapult_arm.move_absolute(600, 100);
    pros::delay(250);
    intake.move_velocity(0);

    // Park
    drive_for_inches(100, 3);

    matchloader.move_absolute(-1400, 200);

    turn_left_deg(90, 80, 250);

    drive_for_inches(120, 38);
    turn_left_deg(60, 70, 250);

    drive_for_inches(50, 10);
    drive_for_inches(120, 15);
}

// ============== SKILLS ==============
void skills() {
    intake.move_velocity(-200);
    discore.move_absolute(850, 200);
    drive_for_inches(80, 31.5);

    pros::delay(2000);

    drive_backward_for_inches(80, 1.5);
    pros::delay(400);

    // Turn left
    left_motor_group.move_velocity(-50);
    right_motor_group.move_velocity(50);
    pros::delay(420);
    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);

    pros::delay(500);

    drive_for_inches(80, 35);
    pros::delay(500);
    matchloader.move_absolute(1400, 200);
    drive_for_inches_async_nonblocking(80, 5);

    intake.move_velocity(0);
    pros::delay(500);
    matchloader.move_absolute(0, 200);

    // Turn right to reset
    left_motor_group.move_velocity(100);
    right_motor_group.move_velocity(-100);
    pros::delay(500);
    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);

    pros::delay(500);

    drive_backward_for_inches(80, 1);

    pros::delay(500);

    // Turn right to reset
    left_motor_group.move_velocity(50);
    right_motor_group.move_velocity(-50);
    pros::delay(510);
    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);

    pros::delay(500);

    drive_for_inches_async_nonblocking(80, 5);

    // Turn left
    left_motor_group.move_velocity(-50);
    right_motor_group.move_velocity(50);
    pros::delay(450);
    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);

    pros::delay(500);

    drive_backward_for_inches(60, 3);

    pros::delay(350);

    drive_backward_for_inches_async_nonblocking(100, 5);

    pros::delay(500);

    // shoot
    discore.move_absolute(0, 200);
    catapult_arm.move_absolute(-300, 200);
    catapult_arm.move_absolute(-600, 40);
    pros::delay(1500);
    catapult_arm.move_absolute(300, 200);
    matchloader.move_absolute(1400, 200);

    pros::delay(500);

    drive_for_inches(50, 3);

    pros::delay(350);

    drive_backward_for_inches_async_nonblocking(80, 7);

    pros::delay(500);

    drive_for_inches(50, 3);

    pros::delay(350);

    drive_backward_for_inches_async_nonblocking(80, 7);

    pros::delay(500);

    drive_for_inches(40, 9);
    pros::delay(350);
    drive_backward_for_inches(40, 2);
    drive_for_inches_async_nonblocking(40, 3);

    intake.move_velocity(-200);
    pros::delay(2500);

    pros::delay(200);

    drive_backward_for_inches(50, 14);

    pros::delay(500);

    // shoot
    discore.move_absolute(0, 200);
    catapult_arm.move_absolute(-300, 200);
    catapult_arm.move_absolute(-600, 40);
    pros::delay(1500);
    intake.move_velocity(0);
    catapult_arm.move_absolute(300, 200);

    pros::delay(500);

    drive_for_inches(60, 2);

    pros::delay(500);

    // Turn left
    left_motor_group.move_velocity(-50);
    right_motor_group.move_velocity(50);
    pros::delay(460);
    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);

    pros::delay(500);

    drive_for_inches(80, 50);
    pros::delay(350);
    drive_for_inches_async_nonblocking(100, 10);

    pros::delay(500);

    drive_backward_for_inches(80, 8);

    // Turn right
    left_motor_group.move_velocity(50);
    right_motor_group.move_velocity(-50);
    pros::delay(460);
    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);

    drive_for_inches(60, 3);
    drive_for_inches_async_nonblocking(100, 10);

    intake.move_velocity(-200);
    pros::delay(2500);
    intake.move_velocity(0);

    pros::delay(350);

    drive_backward_for_inches(80, 6);

    pros::delay(500);

    // shoot
    discore.move_absolute(0, 200);
    catapult_arm.move_absolute(-300, 200);
    catapult_arm.move_absolute(-600, 40);
    pros::delay(1500);
    matchloader.move_absolute(1400, 200);
    catapult_arm.move_absolute(300, 200);

    pros::delay(500);

    drive_for_inches(80, 3);
    matchloader.move_absolute(0, 200);

    pros::delay(500);

    // Turn left
    left_motor_group.move_velocity(-50);
    right_motor_group.move_velocity(50);
    pros::delay(700);
    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);

    pros::delay(500);

    drive_for_inches(80, 45);

    pros::delay(500);

    // Turn left
    left_motor_group.move_velocity(-50);
    right_motor_group.move_velocity(50);
    pros::delay(500);
    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);

    pros::delay(500);

    drive_for_inches(80, 12);
    pros::delay(500);
    drive_for_inches(100, 15);
}

// ============== DEBUG ==============
void debug() {
    // Throw away the red balls
    matchloader.move_absolute(1400, 200);
    pros::delay(350);
    drive_for_inches(80, 2);
    intake.move_velocity(0);

    // Turn to wall
    turn_right_deg(150, 90, 0);
    drive_backward_for_inches(80, 3);
    matchloader.move_absolute(-1400, 200);
    turn_right_deg(40, 50, 250);

    // Wall reset
    wall_reset_v2();
}

// ============== PARK ==============
void park() {
    pros::delay(3000);
    intake.move_velocity(-200);
    matchloader.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    drive_backward_for_inches(80, 10);
    drive_for_inches(120, 25);
    matchloader.move_absolute(0, 100);
    pros::delay(7000);
}

// ============== TEST ==============
void test() {
    left_motor_group.move(80);
    pros::delay(1800);
    left_motor_group.move(0);
}
