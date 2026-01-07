#include "helpers.hpp"
#include <cstdio>

const double wheelDiameter = 3.25;
const double trackWidth = 14.5;
const double ticksPerRev = 360.0;
const int MAX_VOLTAGE = 11000;
const float PI = 3.14159;

// --- Robot state ---
double x = 0.0, y = 0.0, theta = 0.0, heading = 0.0;

// PID constants
float kP = 1.0;
float kI = 0.0;
float kD = 0.5;

double prevLeft = 0.0;
double prevRight = 0.0;
const double TURN_MULTIPLIER = 2.85;

// --- Helper functions ---
float ticksToInches(float ticks) {
  return (ticks / ticksPerRev) * PI * wheelDiameter;
}

// SETTINGS
// Cap voltage at 11000mV (approx 85% power). 
// This ensures performance is the same at 100% batt and 60% batt.
// Helper function to convert RPM/Velocity (0-200) to Voltage (0-12000)
// 200 RPM ~= 12000 mV
double velocityToVoltage(double velocity) {
    return (velocity / 200.0) * 12000.0;
}

// --- New drive_for_inches function ---
double inchesToDegrees(double inches) {
    double wheelCircumference = PI * wheelDiameter;
    double rotations = inches / wheelCircumference;
    return rotations * 360.0; // degrees
}

void drive_to_object(double maxSpeed, double targetInches, int timeoutMs) {
    // 1. Reset start time
    int startTime = pros::millis();

    // 2. Start moving
    left_motor_group.move_velocity(maxSpeed);
    right_motor_group.move_velocity(maxSpeed);

    while (true) {
        // --- READ SENSOR ---
        // PROS ADI Ultrasonic returns Millimeters. 
        // 25.4 mm = 1 inch.
        double currentDistInches = ultrasonic.get_value() / 25.4;

        // If using V5 Distance Sensor instead, use this line:
        // double currentDistInches = distance_sensor.get() / 25.4;

        // --- CHECKS ---
        
        // 1. Check if we are close enough
        // We check > 0 because sometimes sensors return -1 if they see nothing
        if (currentDistInches < targetInches && currentDistInches > 0) {
            break; 
        }

        // 2. Check for timeout (Safety)
        if (pros::millis() - startTime > timeoutMs) {
            break;
        }

        pros::delay(10);
    }

    // 3. Stop
    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);
}

void reverse_to_object_smooth(double maxSpeed, double targetInches, int timeoutMs) {
    int startTime = pros::millis();
    
    // 1. Initial speed and ramping constants
    double currentSpeed = 0;
    const double accelRate = 4.0; // How fast it speeds up per 10ms
    const double minSpeed = 15.0; // Minimum speed to keep the robot moving
    
    // We define a "deceleration zone" in inches. 
    // Example: Start slowing down when within 10 inches of the target.
    const double decelZone = 10.0; 

    while (true) {
        // --- READ SENSOR ---
        double currentDist = ultrasonic.get_value() / 25.4; // Convert mm to inches
        
        // --- 1. HANDLE SPEED (Ramping) ---
        // Calculate how far we are from our stopping point
        double error = currentDist - targetInches;

        if (error > decelZone) {
            // ACCELERATION PHASE
            currentSpeed += accelRate;
            if (currentSpeed > maxSpeed) currentSpeed = maxSpeed;
        } 
        else {
            // DECELERATION PHASE
            // Scale speed based on how close we are to the targetInches
            currentSpeed = maxSpeed * (error / decelZone);
            if (currentSpeed < minSpeed) currentSpeed = minSpeed;
        }

        // --- 2. MOVE MOTORS ---
        // Using negative currentSpeed because we are reversing
        left_motor_group.move_velocity(-currentSpeed);
        right_motor_group.move_velocity(-currentSpeed);

        // --- 3. END CONDITIONS ---
        // Stop if we reach the target distance (with a 0.5 inch tolerance)
        if (currentDist <= targetInches + 0.5 && currentDist > 0) break;

        // Safety Timeout
        if (pros::millis() - startTime > timeoutMs) break;

        pros::delay(10);
    }

    // --- 4. FINAL HARD STOP ---
    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);
}

void drive_for_inches_consistent(double maxSpeedVelocity, double inches) {
    double targetDegrees = inchesToDegrees(inches);
    
    // Convert the user's velocity input (0-200) to Voltage (0-11000)
    // We cap it at MAX_VOLTAGE so it behaves the same on low battery.
    double maxVolts = velocityToVoltage(maxSpeedVelocity);
    if (maxVolts > MAX_VOLTAGE) maxVolts = MAX_VOLTAGE;

    left_motor_group.tare_position();
    right_motor_group.tare_position();

    // SLEW SETTINGS (Acceleration control)
    double currentVolts = 0;
    const double slewStep = 500; // How much voltage to add per loop (Adjust for acceleration)
    
    // DECEL SETTINGS
    const double decelStartRatio = 0.70; // Start slowing down at 70% of distance
    double decelPoint = targetDegrees * decelStartRatio;
    
    while (true) {
        double currentPos = std::abs(left_motor_group.get_position()); // Simplified for brevity

        // 1. CALCULATE DESIRED SPEED
        double targetVolts = maxVolts;

        // Deceleration Logic
        if (currentPos > decelPoint) {
            double remaining = targetDegrees - currentPos;
            // Proportional slow down
            targetVolts = maxVolts * (remaining / (targetDegrees - decelPoint));
            // Minimum voltage to keep moving (friction threshold)
            if (targetVolts < 2000) targetVolts = 2000; 
        }

        // 2. APPLY SLEW RATE (The Soft Start)
        // If we want to go faster than we are currently going, only add a little bit
        if (currentVolts < targetVolts) {
            currentVolts += slewStep;
            if (currentVolts > targetVolts) currentVolts = targetVolts;
        } 
        // If we need to slow down, we can drop voltage instantly (or slew down too)
        else {
            currentVolts = targetVolts;
        }

        // 3. MOVE WITH VOLTAGE
        // Using move_voltage is more "raw" and honest than move_velocity
        left_motor_group.move_voltage(currentVolts);
        right_motor_group.move_voltage(currentVolts);

        // Break condition
        if (currentPos >= targetDegrees - 5) break;
        
        pros::delay(10);
    }

    // Stop
    left_motor_group.move_voltage(0);
    right_motor_group.move_voltage(0);
}

void drive_backward_consistent(double maxSpeedVelocity, double inches) {
    double targetDegrees = -inchesToDegrees(inches); // Negative for reverse
    double maxVolts = velocityToVoltage(maxSpeedVelocity);
    if (maxVolts > MAX_VOLTAGE) maxVolts = MAX_VOLTAGE;

    left_motor_group.tare_position();
    right_motor_group.tare_position();

    double currentVolts = 0;
    const double slewStep = 500;
    const double decelStartRatio = 0.70;
    double decelPoint = targetDegrees * decelStartRatio; // This will be a negative number

    while (true) {
        double currentPos = left_motor_group.get_position(); // Don't use abs() here to track negative

        double targetVolts = -maxVolts; // Target is negative voltage

        // Deceleration (logic inverted for negative numbers)
        if (currentPos < decelPoint) {
            double remaining = targetDegrees - currentPos;
            targetVolts = -maxVolts * (remaining / (targetDegrees - decelPoint));
            if (targetVolts > -2000) targetVolts = -2000; 
        }

        // Slew Rate (Descending towards negative)
        if (currentVolts > targetVolts) {
            currentVolts -= slewStep;
            if (currentVolts < targetVolts) currentVolts = targetVolts;
        } else {
            currentVolts = targetVolts;
        }

        left_motor_group.move_voltage(currentVolts);
        right_motor_group.move_voltage(currentVolts);

        if (currentPos <= targetDegrees + 5) break;
        pros::delay(10);
    }
    left_motor_group.move_voltage(0);
    right_motor_group.move_voltage(0);
}

void turn_time_consistent(double maxSpeedVelocity, int durationMs, bool turnLeft) {
    // 1. CONVERT VELOCITY TO VOLTAGE 
    // We cap it at a value the battery can always reach (e.g., 10000mV or 10V)
    double targetVolts = velocityToVoltage(maxSpeedVelocity);
    if (targetVolts > 10000) targetVolts = 10000; 

    // 2. SLEW SETTINGS (For a smooth start)
    double currentVolts = 0;
    const double slewStep = 500; // Voltage added every 10ms
    
    // 3. TIMER SETUP
    uint32_t startTime = pros::millis();

    while (pros::millis() - startTime < durationMs) {
        
        // APPLY SLEW RATE (Soft Start)
        // This prevents the wheels from slipping/skidding at the start of the turn
        if (currentVolts < targetVolts) {
            currentVolts += slewStep;
            if (currentVolts > targetVolts) currentVolts = targetVolts;
        }

        // DECEL LOGIC (Optional: Slow down for the last 150ms for a precise stop)
        uint32_t elapsed = pros::millis() - startTime;
        if (elapsed > (durationMs - 150)) {
            currentVolts *= 0.8; // Simple decay
            if (currentVolts < 2500) currentVolts = 2500; 
        }

        // MOVE
        if (turnLeft) {
            left_motor_group.move_voltage(-currentVolts);
            right_motor_group.move_voltage(currentVolts);
        } else {
            left_motor_group.move_voltage(currentVolts);
            right_motor_group.move_voltage(-currentVolts);
        }

        pros::delay(10);
    }

    // 4. STOP & BRAKE
    left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    left_motor_group.move_voltage(0);
    right_motor_group.move_voltage(0);
}

void drive_arc_consistent(double maxSpeedVelocity, double inches, double ratio, bool turnLeft, bool forward) {
    double targetDegrees = inchesToDegrees(inches);
    int dir = forward ? 1 : -1; // Direction multiplier
    
    double maxVolts = velocityToVoltage(maxSpeedVelocity);
    if (maxVolts > 10000) maxVolts = 10000;

    left_motor_group.tare_position();
    right_motor_group.tare_position();

    double currentVolts = 0;
    const double slewStep = 500;
    const double decelStartRatio = 0.70;
    double decelPoint = targetDegrees * decelStartRatio;
    
    while (true) {
        // Track the outside wheel (absolute value to handle reverse)
        double currentPos = turnLeft ? 
            std::abs(right_motor_group.get_position()) : 
            std::abs(left_motor_group.get_position());

        // 1. CALCULATE DESIRED SPEED
        double targetVolts = maxVolts;

        if (currentPos > decelPoint) {
            double remaining = targetDegrees - currentPos;
            targetVolts = maxVolts * (remaining / (targetDegrees - decelPoint));
            if (targetVolts < 2000) targetVolts = 2000; 
        }

        // 2. APPLY SLEW RATE
        if (currentVolts < targetVolts) {
            currentVolts += slewStep;
            if (currentVolts > targetVolts) currentVolts = targetVolts;
        } else {
            currentVolts = targetVolts;
        }

        // 3. APPLY DIRECTION AND RATIO
        double outerVolts = currentVolts * dir;
        double innerVolts = (currentVolts * ratio) * dir;

        // 4. MOVE
        if (turnLeft) {
            left_motor_group.move_voltage(innerVolts);
            right_motor_group.move_voltage(outerVolts);
        } else {
            left_motor_group.move_voltage(outerVolts);
            right_motor_group.move_voltage(innerVolts);
        }

        if (currentPos >= targetDegrees - 5) break;
        
        pros::delay(10);
    }

    left_motor_group.move_voltage(0);
    right_motor_group.move_voltage(0);
}

void drive_back_and_forth(double times, double speed, double seconds){
    for(int i = 0; i < times; i++){
        left_motor_group.move(-speed);
        right_motor_group.move(-speed);
        pros::delay(seconds);
        left_motor_group.move(speed);
        right_motor_group.move(speed);
        pros::delay(seconds);
    }
}

void eat_ball(double milliseconds, double velocity){
    intake.move_velocity(-velocity);
    pros::delay(milliseconds);
    intake.move_velocity(0);
}

void spit_ball(double milliseconds, double velocity){
    intake.move_velocity(velocity);
    pros::delay(milliseconds);
    intake.move_velocity(0);
}

void get_matchload(double milliseconds, double velocity, bool twoVtwoV2){
    if(twoVtwoV2 == true){
        discore.move_absolute(-500, 200);
    }
    intake.move_velocity(-velocity);
    pros::delay(milliseconds);
}

void score_long_goal(double angle, double velocity) {
    discore.move_velocity(0);
    catapult_arm.move_absolute(-angle, velocity);
    pros::delay(500);
    catapult_arm.move_absolute(0, velocity);
    pros::delay(500);
    catapult_arm.move_absolute(-angle, velocity);
    pros::delay(500);
    catapult_arm.move_absolute(0, velocity);
}

void detect_wall_to_score(double targetInches) {
    int distance = ultrasonic.get_value() / 25.4; // Convert mm to inches
    printf("Distance: %d mm\n", distance);

    if (distance < 100 && distance != -1) {
        left_motor_group.move_velocity(0);
        right_motor_group.move_velocity(0);
    }

    pros::delay(20); // Small delay to prevent CPU hogging
}

void fire_catapult_safe(double targetInches) {
    // 1. Get the current distance
    double currentDist = ultrasonic.get_value() / 25.4;

    // 2. PRINT TO LAPTOP TERMINAL
    // \n is a "newline" so each reading starts on a new line
    // \r is a "carriage return" to prevent messy indenting
    printf("Distance: %.2f in | Target: %.1f in | Status: ", currentDist, targetInches);

    // 3. Logic and Status Printing
    if (currentDist > 1.0 && currentDist < targetInches) {
        printf("FIRING\n"); // Laptop output
        pros::lcd::print(5, "STATE: FIRING"); // Brain output
        
        catapult_arm.move_absolute(-600, 400);
        discore.move_velocity(0); 
    } 
    else if (currentDist <= 1.0 && currentDist > 0) {
        printf("BLINDED (Too Close)\n");
        pros::lcd::print(5, "STATE: BLINDED");
        catapult_arm.move_absolute(0, 400);
    }
    else {
        printf("EMPTY/FAR\n");
        pros::lcd::print(5, "STATE: EMPTY");
        catapult_arm.move_absolute(0, 400);
    }
}

void drive_for_inches(double maxSpeed, double inches) {
    double targetDegrees = inchesToDegrees(inches);

    left_motor_group.tare_position();
    right_motor_group.tare_position();

    const double accelRate = 2.0;
    const double decelStart = 0.6;

    double currentSpeed = 0;
    double decelPoint = targetDegrees * decelStart;

    while (true) {
        double leftPos  = std::abs(left_motor_group.get_position());
        double rightPos = std::abs(right_motor_group.get_position());
        double avgPos   = (leftPos + rightPos) / 2.0;

        // ACCELERATION
        if (avgPos < decelPoint) {
            currentSpeed += accelRate;
            if (currentSpeed > maxSpeed)
                currentSpeed = maxSpeed;
        }
        // DECELERATION
        else {
            double remaining = targetDegrees - avgPos;
            currentSpeed = maxSpeed * (remaining / (targetDegrees - decelPoint));
            if (currentSpeed < 10) currentSpeed = 10; // lower min for smooth stop
        }

        // END CONDITION
        if (avgPos >= targetDegrees - 2) break;

        left_motor_group.move_velocity(currentSpeed);
        right_motor_group.move_velocity(currentSpeed);

        pros::delay(10);
    }

    // ----- SMOOTH FINAL STOP -----
    double lastSpeed = std::max(currentSpeed, 10.0); // start ramp-down from current speed
    while (lastSpeed > 0) {
        left_motor_group.move_velocity(lastSpeed);
        right_motor_group.move_velocity(lastSpeed);
        lastSpeed -= 2;           // small decrement for smooth stop
        if (lastSpeed < 0) lastSpeed = 0;
        pros::delay(10);
    }

    // Hard stop
    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);
}


void drive_backward_for_inches(double maxSpeed, double inches) {
    // targetDegrees is the magnitude of the rotation needed (always positive)
    double targetDegrees = inchesToDegrees(inches);

    left_motor_group.tare_position();
    right_motor_group.tare_position();

    // Constant parameters
    const double accelRate = 2.0;
    const double decelStart = 0.6; // Start decelerating at 60% of the distance

    double currentSpeed = 0;
    double decelPoint = targetDegrees * decelStart;
    
    // We will use a negative speed command to move backward
    double backwardSpeedCommand = 0.0;

    while (true) {
        // Use the absolute value for position tracking, as in the original function.
        // This keeps the acceleration/deceleration logic simple and positive-based.
        double leftPos  = std::abs(left_motor_group.get_position());
        double rightPos = std::abs(right_motor_group.get_position());
        double avgPos   = (leftPos + rightPos) / 2.0;

        // ACCELERATION (same logic as forward)
        if (avgPos < decelPoint) {
            currentSpeed += accelRate;
            if (currentSpeed > maxSpeed)
                currentSpeed = maxSpeed;
        }
        // DECELERATION (same logic as forward)
        else {
            double remaining = targetDegrees - avgPos;
            currentSpeed = maxSpeed * (remaining / (targetDegrees - decelPoint));
            if (currentSpeed < 10) currentSpeed = 10; // lower min for smooth stop
        }

        // Set the final speed command to be negative for backward movement
        backwardSpeedCommand = -currentSpeed;

        // END CONDITION
        if (avgPos >= targetDegrees - 2) break; // Stop a little early

        left_motor_group.move_velocity(backwardSpeedCommand);
        right_motor_group.move_velocity(backwardSpeedCommand);

        pros::delay(10);
    }

    // ----- SMOOTH FINAL STOP (Ramp down to 0) -----
    // We ramp down the NEGATIVE speed towards 0
    double lastSpeedCommand = std::min(backwardSpeedCommand, -10.0); // start ramp-down from current speed
    while (lastSpeedCommand < 0) { // loop while the command is negative
        left_motor_group.move_velocity(lastSpeedCommand);
        right_motor_group.move_velocity(lastSpeedCommand);
        lastSpeedCommand += 2;      // small POSITIVE increment to approach 0
        if (lastSpeedCommand > 0) lastSpeedCommand = 0;
        pros::delay(10);
    }

    // Hard stop
    left_motor_group.move_velocity(0);
    right_motor_group.move_velocity(0);
}