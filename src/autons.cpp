#include "robot_config.hpp"

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

    //Score to lower center goal
    chassis.setPose(-62.87, -16.93, 0);
    matchloader.move_absolute(-1700, 100);
    chassis.moveToPoint(-17.08, -16.93, 5000);
    chassis.moveToPoint(-12.97, -12.82, 5000);
    intake.move_velocity(200);
    pros::delay(1000);
    intake.move_velocity(0);

    //Get the matchload
    chassis.moveToPoint(-46.24, -46.98, 5000);
    matchloader.move_absolute(0, 100);
    chassis.moveToPoint(-60.37, -46.98, 5000);
    discore.move_absolute(-500, 200);
    intake.move_velocity(-200);
    pros::delay(3000);

    //Score to long goal
    chassis.moveToPoint(-28.17, -46.98, 5000);
    discore.move_velocity(0);
    catapult_arm.move_absolute(-600, 100);
    pros::delay(500);
    catapult_arm.move_absolute(0, 100);
    pros::delay(500);
    catapult_arm.move_absolute(-600, 100);
    pros::delay(500);
    catapult_arm.move_absolute(0, 100);

    //Use discore to gain control zone
    chassis.moveToPoint(-44.98, -37.19, 5000);
    chassis.moveToPoint(-14.22, -37.19, 5000);
    chassis.moveToPoint(-44.98, -37.19, 5000);

    //Position to matchload
    chassis.moveToPoint(-53.03, -46.98, 5000);
    chassis.moveToPoint(-60.19, -46.98, 5000);
};

void skills_autonomous_right();
void twoVtwo_autonomous_left();