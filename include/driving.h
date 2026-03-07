#ifndef DRIVING_H
#define DRIVING_H

// ============== BASIC TURNING ==============
/**
 * Turn left by specified degrees at given speed
 * @param degrees Degrees to turn
 * @param speed Velocity (rpm)
 * @param delay Delay after turn completes (ms)
 */
void turn_left_deg(double degrees, int speed, double delay);

/**
 * Turn right by specified degrees at given speed
 * @param degrees Degrees to turn
 * @param speed Velocity (rpm)
 * @param delay Delay after turn completes (ms)
 */
void turn_right_deg(double degrees, int speed, double delay);

/**
 * Turn left using voltage control
 * @param degrees Degrees to turn
 * @param voltage Voltage to apply
 * @param delay Delay after turn completes (ms)
 */
void turn_left_deg_vol(double degrees, int voltage, double delay);

/**
 * Turn right using voltage control
 * @param degrees Degrees to turn
 * @param voltage Voltage to apply
 * @param delay Delay after turn completes (ms)
 */
void turn_right_deg_vol(double degrees, int voltage, double delay);

// ============== FORWARD DRIVING ==============
/**
 * Drive forward for specified distance with smooth acceleration/deceleration
 * @param maxSpeed Maximum velocity (rpm)
 * @param inches Distance to travel
 */
void drive_for_inches(double maxSpeed, double inches);

/**
 * Drive forward asynchronously (non-blocking)
 * @param maxSpeed Maximum velocity
 * @param inches Distance to travel
 * @param delayMs Delay before movement is allowed again (ms)
 */
void drive_for_inches_async(double maxSpeed, double inches, int delayMs);

/**
 * Drive forward with non-blocking acceleration only (no deceleration)
 * @param maxSpeed Maximum velocity
 * @param inches Distance to travel
 */
void drive_for_inches_async_nonblocking(double maxSpeed, double inches);

/**
 * Drive forward using voltage control
 * @param maxVoltage Maximum voltage to apply
 * @param inches Distance to travel
 */
void drive_for_inches_voltage(double maxVoltage, double inches);

/**
 * Drive forward using voltage with simple acceleration
 * @param maxVoltage Maximum voltage
 * @param inches Distance to travel
 */
void drive_for_inches_voltage_simple(double maxVoltage, double inches);

// ============== BACKWARD DRIVING ==============
/**
 * Drive backward for specified distance with smooth acceleration/deceleration
 * @param maxSpeed Maximum velocity (rpm)
 * @param inches Distance to travel
 */
void drive_backward_for_inches(double maxSpeed, double inches);

/**
 * Drive backward asynchronously (non-blocking)
 * @param maxSpeed Maximum velocity
 * @param inches Distance to travel
 * @param delayMs Delay before movement is allowed again (ms)
 */
void drive_backward_inches_async(double maxSpeed, double inches, int delayMs);

/**
 * Drive backward with non-blocking acceleration only
 * @param maxSpeed Maximum velocity
 * @param inches Distance to travel
 */
void drive_backward_for_inches_async_nonblocking(double maxSpeed, double inches);

// ============== WALL RESET / ALIGNMENT ==============
/**
 * Push robot against wall to reset position
 * @param voltage Voltage to apply (default 8000)
 * @param settleTime Time to wait for stall detection (default 200ms)
 */
void wall_reset(int voltage = 8000, int settleTime = 200);

/**
 * Advanced wall reset with directional control
 * @param voltage Voltage to apply
 * @param settleTime Time for stall detection
 * @param direction 1 for forward, -1 for backward
 * @param timeout Maximum time to run (ms)
 */
void wall_reset_v2(int voltage = 8000, int settleTime = 200, int direction = 1, 
                   int timeout = 1000);

// ============== WALL HIT DETECTION ==============
/**
 * Check if robot has hit a wall (stopped moving)
 * @return true if robot appears to have hit a wall
 */
bool drive_hit_wall();

#endif // DRIVING_H
