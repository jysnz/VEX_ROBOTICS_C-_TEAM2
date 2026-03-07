#ifndef UTILS_H
#define UTILS_H

#include <cmath>

// ============== MATH UTILITIES ==============
/**
 * Get the sign of a number
 * @param x The number
 * @return 1 if x > 0, -1 if x < 0, 0 if x == 0
 */
double sign(double x);

// ============== UNIT CONVERSION ==============
/**
 * Convert motor ticks to inches traveled
 * @param ticks Motor position in degrees
 * @return Distance in inches
 */
float ticksToInches(float ticks);

/**
 * Convert distance in inches to motor degrees needed
 * @param inches Distance to travel
 * @return Motor degrees (rotations * 360)
 */
double inchesToDegrees(double inches);

/**
 * Convert degrees to milliseconds for turning
 * @param degrees Degrees to turn
 * @return Time in milliseconds
 */
int turn_deg_to_ms(double degrees);

// ============== SENSOR UTILITIES ==============
/**
 * Get distance from ultrasonic sensor in centimeters
 * @return Distance in cm, or -1 if out of range
 */
double get_distance_cm();

/**
 * Read ultrasonic sensor continuously (debug/telemetry)
 */
void ultrasonicSense();

#endif // UTILS_H
