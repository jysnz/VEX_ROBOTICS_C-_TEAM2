#include "lemlib/chassis/chassis.hpp"
#include "odometry.hpp"
#include "robot_config.hpp"
#include "helpers.hpp"

void AS5600_autonSkills(){
    driveForwardPID(12, 100, 1000);
    pros::delay(500);
    driveForwardPID(24, 100, 1000);

    // driveForwardPID(18, 100, 3000);
    // pros::delay(500);
    // driveReversePID(18, 50, 4000);
}

void hardCoded2v2Auton(){
    //Move to lower center goal
    matchloader.move_absolute(-1700, 100);
    drive_for_inches(80, 21.5); 

    //Turn left to lower center goal
    right_motor_group.move(100);
    pros::delay(270);
    right_motor_group.move(0);

    //Score to lower center goal the payload
    intake.move_velocity(100);
    pros::delay(2000);
    intake.move_velocity(0);

    //Move to matchload
    drive_backward_for_inches(80, 16.5);
    matchloader.move_absolute(0, 100);

    //Turn left to face the matchload
    left_motor_group.move(-100);
    pros::delay(730);
    left_motor_group.move(0);

    //Get the matchload
    drive_for_inches(80, 8.8);
    intake.move_velocity(-200);
    discore.move_absolute(-650, 200);
    pros::delay(7000);
    intake.move_velocity(0);

    //Score to long goal
    drive_backward_for_inches(60, 13);
    pros::delay(1000);

    fire_catapult_safe(2);
    discore.move_velocity(0);
    pros::delay(1000);
    catapult_arm.move_absolute(0, 400);
}

void park2v2V2(){
    chassis.turnToHeading(274.76, 5000);
    chassis.moveToPoint(-61.63, -33.48, 5000);
    chassis.turnToHeading(358.61, 5000);
    chassis.moveToPoint(-62.43, -0.25, 5000, {.minSpeed = 127});
}

void skills_autonomous_left(){
    chassis.setPose(-62.636, 17.13, 0);
    
    //Score to upper center goal
    matchloader.move_absolute(-1700, 100);
    chassis.moveToPoint(-17.10, 17.13, 5000);
    chassis.moveToPoint(-13.29, 13.07, 5000);
    catapult_arm.move_absolute(-600, 100);
    pros::delay(500);
    catapult_arm.move_absolute(0, 100);

    //Get the alternative balls
    intake.move_velocity(-200);
    discore.move_absolute(-500, 200);
    chassis.moveToPoint(-0.65, 22.13, 5000);
    chassis.moveToPoint(-0.57, 38.12, 5000);
    intake.move_velocity(0);

    //Score to lower center goal
    chassis.moveToPoint(22.68, 21.18, 5000);
    chassis.moveToPoint(13.36, 12.12, 5000);
    intake.move_velocity(200);
    pros::delay(4000);
    intake.move_velocity(0);
    discore.move_velocity(0);
    
    //Go to other side matchload
    chassis.moveToPoint(42.26, 45.97, 5000);
    matchloader.move_absolute(0, 100);
    chassis.moveToPoint(59.67, 46.45, 5000);
    intake.move_velocity(-200);
    discore.move_absolute(-500, 200);
    pros::delay(2000);
    intake.move_velocity(0);

    //Go to other side to score
    chassis.moveToPoint(-36.63, 27.59, 5000);
    chassis.moveToPoint(-47.58, 47.31, 5000);
    chassis.moveToPoint(-30.18, 46.84, 5000);
    discore.move_velocity(0);
    catapult_arm.move_absolute(-600, 100);
    pros::delay(500);
    catapult_arm.move_absolute(0, 100);
    pros::delay(500);
    catapult_arm.move_absolute(-600, 100);
    pros::delay(500);
    catapult_arm.move_absolute(0, 100);

    //Get matchload
    chassis.moveToPoint(-60.69, 46.60, 5000);
    intake.move_velocity(-200);
    discore.move_absolute(-500, 200);
    pros::delay(2000);
    intake.move_velocity(0);    
    
    //Score to long goal
    chassis.moveToPoint(-30.18, 46.84, 5000);
    discore.move_velocity(0);
    catapult_arm.move_absolute(-600, 100);
    pros::delay(500);
    catapult_arm.move_absolute(0, 100);
    pros::delay(500);
    catapult_arm.move_absolute(-600, 100);
    pros::delay(500);
    catapult_arm.move_absolute(0, 100);

    //Park
    chassis.moveToPoint(-59.98, 29.43, 5000);
    chassis.moveToPoint(-62.63, 17.13, 5000);
    chassis.moveToPoint(-62.606, 2.74, 5000);

};

void twoVtwo_autonomous_right(){
    chassis.setPose(-62.90, -17.10, 0);
    //Score to lower center goal
    matchloader.move_absolute(-1700, 100);
    chassis.moveToPoint(-17.03, -17.10, 5000);
    chassis.turnToHeading(43.15, 5000);
    chassis.moveToPoint(-12.91, -12.71, 5000);
    spit_ball(1000, 200);

    //Get the matchload
    chassis.moveToPoint(-40.42, -47.26, 5000, {.forwards = false});
    matchloader.move_absolute(0, 100);
    chassis.turnToHeading(270, 5000);
    chassis.moveToPoint(-61.87, -47, 5000, {.maxSpeed = 60});
    get_matchload(3000, 200);

    //Score to long goal
    chassis.moveToPoint(-28.68, -47, 5000, {.forwards = false});
    score_long_goal(300, 100);
    intake.move_velocity(0);

    //Use discore to gain control zone
    chassis.moveToPoint(-40.42, -47.26, 5000);
    chassis.turnToHeading(38.08, 5000);
    chassis.moveToPoint(-33.02, -37.81, 5000);
    chassis.turnToHeading(90, 5000);
    chassis.moveToPoint(-14.13, -37.81, 5000, {.maxSpeed = 60});

    //Position to matchload
    chassis.moveToPoint(-33.02, -37.81, 5000, {.forwards = false});
    chassis.turnToHeading(39.95, 5000);
    chassis.moveToPoint(-40.42, -47, 5000);
    chassis.turnToHeading(270, 5000);
    chassis.moveToPoint(-61.87, -47, 5000, {.maxSpeed = 60});
};

void twoVtwo_autonomous_right_v2_blue(){
    chassis.setPose(-62.90, -17.10, 0);
    //Score to lower center goal
    matchloader.move_absolute(-1700, 100);
    chassis.moveToPoint(-17.03, -17.10, 5000);
    chassis.turnToHeading(43.15, 5000);
    chassis.moveToPoint(-12.91, -12.71, 5000);
    spit_ball(1000, 200);

    //Get the two balls near matchload
    chassis.moveToPoint(-40.42, -47.26, 5000, {.forwards = false});
    intake.move_velocity(-200);
    discore.move_absolute(-500, 200);
    chassis.turnToHeading(0, 5000);
    chassis.moveToPoint(-46.89, -63.22, 5000);
    intake.move_velocity(0);

    //Get the matchload
    chassis.turnToHeading(333.81, 5000);
    matchloader.move_absolute(0, 100);
    chassis.moveToPoint(-54.93, -46.88, 5000);
    chassis.turnToHeading(270, 5000);
    chassis.moveToPoint(-61.87, -47, 5000, {.maxSpeed = 60});   
    get_matchload(3000, 200, true);

    //Score to long goal
    chassis.moveToPoint(-28.68, -47, 5000, {.forwards = false});
    score_long_goal(300, 100);
    intake.move_velocity(0);

    //Use discore to gain control zone
    chassis.moveToPoint(-40.42, -47.26, 5000);
    chassis.turnToHeading(38.08, 5000);
    chassis.moveToPoint(-33.02, -37.81, 5000);
    chassis.turnToHeading(90, 5000);
    chassis.moveToPoint(-14.13, -37.81, 5000, {.maxSpeed = 60});

    park2v2V2();
}

void skills_autonomous_right();
void twoVtwo_autonomous_left();