#include "driving.h"
#include "utils.h"
#include "constants.h"
#include "motor_config.h"
#include <cmath>

// ============== BASIC TURNING ==============
void turn_left_deg(double degrees, int speed, double delay) {
    int timeMs = turn_deg_to_ms(degrees);

    left_motor_group.move_velocity(-speed);
    right_motor_group.move_velocity(speed);

    pros::delay(timeMs);

    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);

    pros::delay(delay);
}

void turn_right_deg(double degrees, int speed, double delay) {
    int timeMs = turn_deg_to_ms(degrees);

    left_motor_group.move_velocity(speed);
    right_motor_group.move_velocity(-speed);

    pros::delay(timeMs);

    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);

    pros::delay(delay);
}

void turn_left_deg_vol(double degrees, int voltage, double delay) {
    int timeMs = turn_deg_to_ms(degrees);

    left_motor_group.move_voltage(-voltage);
    right_motor_group.move_voltage(voltage);

    pros::delay(timeMs);

    left_motor_group.move_voltage(0);
    right_motor_group.move_voltage(0);

    pros::delay(delay);
}

void turn_right_deg_vol(double degrees, int voltage, double delay) {
    int timeMs = turn_deg_to_ms(degrees);

    left_motor_group.move_voltage(voltage);
    right_motor_group.move_voltage(-voltage);

    pros::delay(timeMs);

    left_motor_group.move_voltage(0);
    right_motor_group.move_voltage(0);

    pros::delay(delay);
}

// ============== WALL HIT DETECTION ==============
bool drive_hit_wall() {
    static int stoppedTime = 0;

    double lv = std::abs(left_motor_group.get_actual_velocity());
    double rv = std::abs(right_motor_group.get_actual_velocity());

    if (lv < Drivetrain::Motion::VEL_THRESH && rv < Drivetrain::Motion::VEL_THRESH) {
        stoppedTime += 10;
        if (stoppedTime >= Drivetrain::Motion::STOP_TIME_MS)
            return true;
    } else {
        stoppedTime = 0;
    }

    return false;
}

// ============== FORWARD DRIVING ==============
void drive_for_inches(double maxSpeed, double inches) {
    double targetDegrees = inchesToDegrees(inches);

    left_motor_group.tare_position();
    right_motor_group.tare_position();

    double currentSpeed = 0;
    double decelPoint = targetDegrees * Drivetrain::Motion::DECEL_START;

    while (true) {
        double leftPos = std::abs(left_motor_group.get_position());
        double rightPos = std::abs(right_motor_group.get_position());
        double avgPos = (leftPos + rightPos) / 2.0;

        // ACCELERATION
        if (avgPos < decelPoint) {
            currentSpeed += Drivetrain::Motion::ACCEL_RATE;
            if (currentSpeed > maxSpeed)
                currentSpeed = maxSpeed;
        }
        // DECELERATION
        else {
            double remaining = targetDegrees - avgPos;
            currentSpeed = maxSpeed * (remaining / (targetDegrees - decelPoint));
            if (currentSpeed < Drivetrain::Motion::MIN_FINAL_SPEED)
                currentSpeed = Drivetrain::Motion::MIN_FINAL_SPEED;
        }

        // END CONDITION
        if (avgPos >= targetDegrees - 2)
            break;

        left_motor_group.move_velocity(currentSpeed);
        right_motor_group.move_velocity(currentSpeed);

        pros::delay(10);
    }

    // ----- SMOOTH FINAL STOP -----
    double lastSpeed = std::max(currentSpeed, Drivetrain::Motion::MIN_FINAL_SPEED);
    while (lastSpeed > 0) {
        left_motor_group.move_velocity(lastSpeed);
        right_motor_group.move_velocity(lastSpeed);
        lastSpeed -= 2;
        if (lastSpeed < 0)
            lastSpeed = 0;
        pros::delay(10);
    }

    // Hard stop
    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);
}

// ============== FORWARD DRIVING - ASYNC TASKS ==============
bool drive_forward_nb_running = false;

void drive_forward_nb_task(void *param) {
    drive_forward_nb_running = true;

    double *args = static_cast<double *>(param);
    double maxSpeed = args[0];
    double inches = args[1];

    double targetDegrees = inchesToDegrees(inches);

    left_motor_group.tare_position();
    right_motor_group.tare_position();

    double currentSpeed = 0;

    while (true) {
        double leftPos = std::abs(left_motor_group.get_position());
        double rightPos = std::abs(right_motor_group.get_position());
        double avgPos = (leftPos + rightPos) / 2.0;

        if (drive_hit_wall())
            break;

        if (avgPos >= targetDegrees - 2)
            break;

        currentSpeed += Drivetrain::Motion::ACCEL_RATE;
        if (currentSpeed > maxSpeed)
            currentSpeed = maxSpeed;

        left_motor_group.move_velocity(currentSpeed);
        right_motor_group.move_velocity(currentSpeed);

        pros::delay(10);
    }

    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);

    delete[] args;
    drive_forward_nb_running = false;
}

void drive_for_inches_async_nonblocking(double maxSpeed, double inches) {
    if (drive_forward_nb_running)
        return;

    double *args = new double[2]{maxSpeed, inches};
    pros::Task(drive_forward_nb_task, args, "Drive Forward NB");
}

// ============== FORWARD DRIVING - ASYNC WITH DELAY ==============
bool drive_task_running = false;

void drive_task_fn(void *param) {
    drive_task_running = true;

    double *args = static_cast<double *>(param);
    double maxSpeed = args[0];
    double inches = args[1];
    int delayMs = static_cast<int>(args[2]);

    drive_for_inches(maxSpeed, inches);

    pros::delay(delayMs);

    delete[] args;
    drive_task_running = false;
}

void drive_for_inches_async(double maxSpeed, double inches, int delayMs) {
    if (drive_task_running)
        return;

    double *args = new double[3];
    args[0] = maxSpeed;
    args[1] = inches;
    args[2] = static_cast<double>(delayMs);

    pros::Task driveTask(drive_task_fn, args, "Drive Task");
}

// ============== FORWARD DRIVING - VOLTAGE BASED ==============
void drive_for_inches_voltage(double maxVoltage, double inches) {
    double targetDegrees = inchesToDegrees(std::abs(inches));
    double direction = sign(inches);

    left_motor_group.tare_position();
    right_motor_group.tare_position();

    double currentVoltage = 0;
    double decelPoint = targetDegrees * Drivetrain::Motion::DECEL_START;

    while (true) {
        double leftPos = std::abs(left_motor_group.get_position());
        double rightPos = std::abs(right_motor_group.get_position());
        double avgPos = (leftPos + rightPos) / 2.0;

        if (avgPos < decelPoint) {
            currentVoltage += Drivetrain::Motion::ACCEL_STEP;
            if (currentVoltage > maxVoltage)
                currentVoltage = maxVoltage;
        }
        else {
            double remaining = targetDegrees - avgPos;
            currentVoltage = maxVoltage * (remaining / (targetDegrees - decelPoint));
            if (currentVoltage < Drivetrain::Motion::MIN_VOLTAGE)
                currentVoltage = Drivetrain::Motion::MIN_VOLTAGE;
        }

        if (avgPos >= targetDegrees - 2)
            break;

        left_motor_group.move_voltage(currentVoltage * direction);
        right_motor_group.move_voltage(currentVoltage * direction);

        pros::delay(10);
    }

    double lastVoltage = std::max(currentVoltage, Drivetrain::Motion::MIN_VOLTAGE);
    while (lastVoltage > 0) {
        left_motor_group.move_voltage(lastVoltage * direction);
        right_motor_group.move_voltage(lastVoltage * direction);
        lastVoltage -= Drivetrain::Motion::RAMP_DECREMENT;
        if (lastVoltage < 0)
            lastVoltage = 0;
        pros::delay(10);
    }

    left_motor_group.move_voltage(0);
    right_motor_group.move_voltage(0);
}

void drive_for_inches_voltage_simple(double maxVoltage, double inches) {
    double targetDegrees = inchesToDegrees(std::abs(inches));
    double direction = sign(inches);

    left_motor_group.tare_position();
    right_motor_group.tare_position();

    double currentVoltage = 0;

    while (true) {
        double leftPos = std::abs(left_motor_group.get_position());
        double rightPos = std::abs(right_motor_group.get_position());
        double avgPos = (leftPos + rightPos) / 2.0;

        if (avgPos >= targetDegrees - 2)
            break;

        currentVoltage += 400;
        if (currentVoltage > maxVoltage)
            currentVoltage = maxVoltage;

        left_motor_group.move_voltage(currentVoltage * direction);
        right_motor_group.move_voltage(currentVoltage * direction);

        pros::delay(10);
    }

    double lastVoltage = std::max(currentVoltage, Drivetrain::Motion::MIN_VOLTAGE);
    while (lastVoltage > 0) {
        left_motor_group.move_voltage(lastVoltage * direction);
        right_motor_group.move_voltage(lastVoltage * direction);
        lastVoltage -= 200;
        if (lastVoltage < 0)
            lastVoltage = 0;
        pros::delay(10);
    }

    left_motor_group.move_voltage(0);
    right_motor_group.move_voltage(0);
}

// ============== BACKWARD DRIVING ==============
void drive_backward_for_inches(double maxSpeed, double inches) {
    double targetDegrees = inchesToDegrees(inches);

    left_motor_group.tare_position();
    right_motor_group.tare_position();

    double currentSpeed = 0;
    double decelPoint = targetDegrees * Drivetrain::Motion::DECEL_START;
    double backwardSpeedCommand = 0.0;

    while (true) {
        double leftPos = std::abs(left_motor_group.get_position());
        double rightPos = std::abs(right_motor_group.get_position());
        double avgPos = (leftPos + rightPos) / 2.0;

        if (avgPos < decelPoint) {
            currentSpeed += Drivetrain::Motion::ACCEL_RATE;
            if (currentSpeed > maxSpeed)
                currentSpeed = maxSpeed;
        }
        else {
            double remaining = targetDegrees - avgPos;
            currentSpeed = maxSpeed * (remaining / (targetDegrees - decelPoint));
            if (currentSpeed < Drivetrain::Motion::MIN_FINAL_SPEED)
                currentSpeed = Drivetrain::Motion::MIN_FINAL_SPEED;
        }

        backwardSpeedCommand = -currentSpeed;

        if (avgPos >= targetDegrees - 2)
            break;

        left_motor_group.move_velocity(backwardSpeedCommand);
        right_motor_group.move_velocity(backwardSpeedCommand);

        pros::delay(10);
    }

    double lastSpeedCommand = std::min(backwardSpeedCommand, -Drivetrain::Motion::MIN_FINAL_SPEED);
    while (lastSpeedCommand < 0) {
        left_motor_group.move_velocity(lastSpeedCommand);
        right_motor_group.move_velocity(lastSpeedCommand);
        lastSpeedCommand += 2;
        if (lastSpeedCommand > 0)
            lastSpeedCommand = 0;
        pros::delay(10);
    }

    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);
}

// ============== BACKWARD DRIVING - ASYNC TASKS ==============
bool drive_backward_nb_running = false;

void drive_backward_nb_task(void *param) {
    drive_backward_nb_running = true;

    double *args = static_cast<double *>(param);
    double maxSpeed = args[0];
    double inches = args[1];

    double targetDegrees = inchesToDegrees(inches);

    left_motor_group.tare_position();
    right_motor_group.tare_position();

    double currentSpeed = 0;

    while (true) {
        double leftPos = std::abs(left_motor_group.get_position());
        double rightPos = std::abs(right_motor_group.get_position());
        double avgPos = (leftPos + rightPos) / 2.0;

        if (drive_hit_wall())
            break;

        if (avgPos >= targetDegrees - 2)
            break;

        currentSpeed += Drivetrain::Motion::ACCEL_RATE;
        if (currentSpeed > maxSpeed)
            currentSpeed = maxSpeed;

        left_motor_group.move_velocity(-currentSpeed);
        right_motor_group.move_velocity(-currentSpeed);

        pros::delay(10);
    }

    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);

    delete[] args;
    drive_backward_nb_running = false;
}

void drive_backward_for_inches_async_nonblocking(double maxSpeed, double inches) {
    if (drive_backward_nb_running)
        return;

    double *args = new double[2]{maxSpeed, inches};
    pros::Task(drive_backward_nb_task, args, "Drive Backward NB");
}

// ============== BACKWARD DRIVING - ASYNC WITH DELAY ==============
bool drive_backward_task_running = false;

void drive_backward_task_fn(void *param) {
    drive_backward_task_running = true;

    double *args = static_cast<double *>(param);
    double maxSpeed = args[0];
    double inches = args[1];
    int delayMs = static_cast<int>(args[2]);

    drive_backward_for_inches(maxSpeed, inches);

    pros::delay(delayMs);

    delete[] args;
    drive_backward_task_running = false;
}

void drive_backward_inches_async(double maxSpeed, double inches, int delayMs) {
    if (drive_backward_task_running)
        return;

    double *args = new double[3];
    args[0] = maxSpeed;
    args[1] = inches;
    args[2] = static_cast<double>(delayMs);

    pros::Task driveBackwardTask(drive_backward_task_fn, args, "Drive Backward Task");
}

// ============== WALL RESET ==============
void wall_reset(int voltage, int settleTime) {
    left_motor_group.move_voltage(voltage);
    right_motor_group.move_voltage(voltage);

    int stalledTime = 0;

    while (stalledTime < settleTime) {
        double lv = std::abs(left_motor_group.get_actual_velocity());
        double rv = std::abs(right_motor_group.get_actual_velocity());

        if (lv < Mechanisms::STALL_VELOCITY_THRESHOLD && 
            rv < Mechanisms::STALL_VELOCITY_THRESHOLD) {
            stalledTime += 10;
        } else {
            stalledTime = 0;
        }

        pros::delay(10);
    }

    left_motor_group.move_voltage(0);
    right_motor_group.move_voltage(0);

    pros::delay(50);
}

void wall_reset_v2(int voltage, int settleTime, int direction, int timeout) {
    voltage = std::abs(voltage) * direction;

    left_motor_group.move_voltage(voltage);
    right_motor_group.move_voltage(voltage);

    int stalledTime = 0;
    int elapsed = 0;

    while (stalledTime < settleTime && elapsed < timeout) {
        double lv = std::abs(left_motor_group.get_actual_velocity());
        double rv = std::abs(right_motor_group.get_actual_velocity());

        if (lv < Mechanisms::STALL_VELOCITY_THRESHOLD && 
            rv < Mechanisms::STALL_VELOCITY_THRESHOLD) {
            stalledTime += 10;
        } else {
            stalledTime = 0;
        }

        pros::delay(10);
        elapsed += 10;
    }

    left_motor_group.move_voltage(0);
    right_motor_group.move_voltage(0);

    pros::delay(50);
}
