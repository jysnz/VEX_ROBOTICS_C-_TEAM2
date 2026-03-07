#ifndef MECHANISMS_H
#define MECHANISMS_H

// ============== CATAPULT CONTROL ==============
/**
 * Enum for catapult firing states
 */
enum CatapultState { CAT_IDLE, CAT_FIRING, CAT_RELOADING };

/**
 * Start the catapult shooting sequence
 * Uses state machine for automatic firing and reloading
 */
void startCatapultShoot();

/**
 * Main catapult task - manages firing state machine
 * Should run as a background task
 */
void catapultTask(void *);

/**
 * Shoot using catapult (blocking - waits for completion)
 */
void shoot(double velocity = 200);

/**
 * Shoot with automatic retry and jam detection (for autonomous)
 * @param SPEED Catapult arm speed
 */
void catapultShootForAuto(double SPEED);

/**
 * Shoot catapult with automatic retry (blocking version)
 */
void catapultShoot();

// ============== CATAPULT HOMING/RESET ==============
/**
 * Reset catapult to home position with smooth voltage ramp
 * @param maxVoltage Maximum voltage to apply
 * @param minVoltage Minimum starting voltage
 * @param rampStep Voltage increment per step
 * @param settleTime Time to wait for stall (ms)
 * @param timeout Maximum time to run (ms)
 */
void catapult_reset_smooth(int maxVoltage = 7000,
                           int minVoltage = 1500,
                           int rampStep = 200,
                           int settleTime = 200,
                           int timeout = 2000);

/**
 * Reset discore mechanism with smooth voltage ramp
 * @param maxVoltage Maximum voltage to apply
 * @param minVoltage Minimum starting voltage
 * @param rampStep Voltage increment per step
 * @param settleTime Time to wait for stall (ms)
 * @param timeout Maximum time to run (ms)
 */
void discore_reset_smooth(int maxVoltage = 7000,
                          int minVoltage = 1500,
                          int rampStep = 200,
                          int settleTime = 200,
                          int timeout = 2000);

// ============== INTAKE CONTROL ==============
/**
 * Run intake motors to collect game pieces
 */
void intake();

/**
 * Run intake in reverse to eject game pieces
 */
void outtake();

// ============== MATCH LOADER CONTROL ==============
// Match loader control is done directly via motor commands
// Example: matchloader.move_absolute(1400, 200);

// ============== DISCORE CONTROL ==============
// Discore control is done directly via motor commands
// Example: discore.move_absolute(0, 200);

#endif // MECHANISMS_H
