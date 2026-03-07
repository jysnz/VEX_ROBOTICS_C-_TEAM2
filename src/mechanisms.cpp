#include "mechanisms.h"
#include "constants.h"
#include "motor_config.h"
#include <cmath>

// ============== GLOBAL STATE TRACKING ==============
CatapultState catState = CAT_IDLE;
int catAttempts = 0;
bool shotSuccess = false;
int stalledTime = 0;

// ============== CATAPULT CONTROL ==============
void startCatapultShoot() {
    if (catState != CAT_IDLE)
        return;

    catAttempts = 0;
    stalledTime = 0;
    shotSuccess = false;

    catState = CAT_FIRING;
    catapult_arm.move_absolute(Catapult::FIRE_POS, Catapult::SPEED);
}

void catapultTask(void *) {
    while (true) {
        double pos = catapult_arm.get_position();
        double vel = std::abs(catapult_arm.get_actual_velocity());

        switch (catState) {

        case CAT_IDLE:
            // Do nothing
            break;

        case CAT_FIRING:
            // Shot completed
            if (pos <= Catapult::FIRE_POS + 25) {
                shotSuccess = true;
                catState = CAT_RELOADING;
                catapult_arm.move_absolute(Catapult::LOAD_POS, Catapult::SPEED);
                break;
            }

            // Stall detection
            if (vel < 5)
                stalledTime += Catapult::CHECK_DELAY;
            else
                stalledTime = 0;

            // Jam detected
            if (stalledTime >= Catapult::STALL_TIME) {
                catState = CAT_RELOADING;
                catapult_arm.move_absolute(Catapult::LOAD_POS, Catapult::SPEED);
            }
            break;

        case CAT_RELOADING:
            // Finished reload
            if (std::abs(pos - Catapult::LOAD_POS) < Mechanisms::POSITION_TOLERANCE) {

                if (shotSuccess || ++catAttempts >= Catapult::MAX_ATTEMPTS) {
                    // Done
                    catState = CAT_IDLE;
                    catAttempts = 0;
                    shotSuccess = false;
                } else {
                    // Retry fire
                    catState = CAT_FIRING;
                    stalledTime = 0;
                    catapult_arm.move_absolute(Catapult::FIRE_POS, Catapult::SPEED);
                }
            }
            break;
        }

        pros::delay(Catapult::CHECK_DELAY);
    }
}

void shoot(double velocity) {
    catapult_arm.move_absolute(-550, velocity);
    discore.move_absolute(0, 200);
    pros::delay(300);
    catapult_arm.move_absolute(0, velocity);
}

void catapultShoot() {
    for (int attempt = 0; attempt < Catapult::MAX_ATTEMPTS; attempt++) {

        bool shotSuccess = false;

        // ---- FIRE ----
        catapult_arm.move_absolute(Catapult::FIRE_POS, Catapult::SPEED);

        int stalledTime = 0;

        while (true) {
            pros::delay(Catapult::CHECK_DELAY);

            double pos = catapult_arm.get_position();
            double vel = std::abs(catapult_arm.get_actual_velocity());

            // Successful fire
            if (pos <= Catapult::FIRE_POS + 25) {
                shotSuccess = true;
                break;
            }

            // Jam detection
            if (vel < 5)
                stalledTime += Catapult::CHECK_DELAY;
            else
                stalledTime = 0;

            if (stalledTime >= Catapult::STALL_TIME)
                break;
        }

        // ---- RESET / RELOAD ----
        catapult_arm.move_absolute(Catapult::LOAD_POS, Catapult::SPEED);

        while (std::abs(catapult_arm.get_position() - Catapult::LOAD_POS) > Mechanisms::POSITION_TOLERANCE) {
            pros::delay(10);
        }

        // If shot worked, we are done
        if (shotSuccess)
            return;

        // Otherwise: loop repeats → retry fire
    }

    // Failsafe
    catapult_arm.move_velocity(0);
}

void catapultShootForAuto(double SPEED) {
    for (int attempt = 0; attempt < Catapult::MAX_ATTEMPTS; attempt++) {

        bool shotSuccess = false;

        // ---- FIRE ----
        catapult_arm.move_absolute(Catapult::FIRE_POS, SPEED);

        int stalledTime = 0;

        while (true) {
            pros::delay(Catapult::CHECK_DELAY);

            double pos = catapult_arm.get_position();
            double vel = std::abs(catapult_arm.get_actual_velocity());

            // Successful fire
            if (pos <= Catapult::FIRE_POS + 25) {
                shotSuccess = true;
                break;
            }

            // Jam detection
            if (vel < 5)
                stalledTime += Catapult::CHECK_DELAY;
            else
                stalledTime = 0;

            if (stalledTime >= Catapult::STALL_TIME)
                break;
        }

        // ---- RESET / RELOAD ----
        catapult_arm.move_absolute(Catapult::LOAD_POS, SPEED);

        while (std::abs(catapult_arm.get_position() - Catapult::LOAD_POS) > Mechanisms::POSITION_TOLERANCE) {
            pros::delay(10);
        }

        // If shot worked, we are done
        if (shotSuccess)
            return;

        // Otherwise: loop repeats → retry fire
    }

    // Failsafe
    catapult_arm.move_velocity(0);
}

// ============== CATAPULT HOMING/RESET ==============
void catapult_reset_smooth(int maxVoltage,
                           int minVoltage,
                           int rampStep,
                           int settleTime,
                           int timeout) {
    int stalledTime = 0;
    int elapsed = 0;
    int currentVoltage = minVoltage;

    // Initial gentle start
    catapult_arm.move_voltage(currentVoltage);

    while (stalledTime < settleTime && elapsed < timeout) {

        // Smooth ramp up
        currentVoltage += rampStep;
        if (currentVoltage > maxVoltage)
            currentVoltage = maxVoltage;

        catapult_arm.move_voltage(currentVoltage);

        double vel = std::abs(catapult_arm.get_actual_velocity());

        // Stall detect
        if (vel < Mechanisms::STALL_VELOCITY_THRESHOLD) {
            stalledTime += 10;
        } else {
            stalledTime = 0;
        }

        pros::delay(10);
        elapsed += 10;
    }

    // Smooth ramp down
    while (currentVoltage > 0) {
        currentVoltage -= 300;
        if (currentVoltage < 0)
            currentVoltage = 0;

        catapult_arm.move_voltage(currentVoltage);
        pros::delay(10);
    }

    catapult_arm.move_voltage(0);
    pros::delay(50);

    // Define this as home
    catapult_arm.tare_position();
}

void discore_reset_smooth(int maxVoltage,
                          int minVoltage,
                          int rampStep,
                          int settleTime,
                          int timeout) {
    int stalledTime = 0;
    int elapsed = 0;
    int currentVoltage = minVoltage;

    // Initial gentle start
    discore.move_voltage(-currentVoltage);

    while (stalledTime < settleTime && elapsed < timeout) {

        // Smooth ramp up
        currentVoltage += rampStep;
        if (currentVoltage > maxVoltage)
            currentVoltage = maxVoltage;

        discore.move_voltage(-currentVoltage);

        double vel = std::abs(catapult_arm.get_actual_velocity());

        // Stall detect
        if (vel < Mechanisms::STALL_VELOCITY_THRESHOLD) {
            stalledTime += 10;
        } else {
            stalledTime = 0;
        }

        pros::delay(10);
        elapsed += 10;
    }

    // Smooth ramp down
    while (currentVoltage > 0) {
        currentVoltage -= 300;
        if (currentVoltage < 0)
            currentVoltage = 0;

        discore.move_voltage(-currentVoltage);
        pros::delay(10);
    }

    discore.move_voltage(0);
    pros::delay(50);

    // Define this as home
    discore.tare_position();
}

// ============== INTAKE CONTROL ==============
void intake() {
    intake.move_velocity(200);
    intake2.move_velocity(-200);
    intake3.move_velocity(200);
}

void outtake() {
    intake.move_velocity(-200);
    intake2.move_velocity(200);
    intake3.move_velocity(-200);
}
