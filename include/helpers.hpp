#include "robot_config.hpp"

float ticksToInches(float ticks);
double velocityToVoltage(double velocity);
double inchesToDegrees(double inches);

void drive_to_object(double maxSpeed, double targetInches, int timeoutMs);
void reverse_to_object_smooth(double maxSpeed, double targetInches, int timeoutMs);
void drive_for_inches_consistent(double maxSpeedVelocity, double inches);
void drive_backward_consistent(double maxSpeedVelocity, double inches);
void turn_time_consistent(double maxSpeedVelocity, int durationMs, bool turnLeft);
void drive_arc_consistent(double maxSpeedVelocity, double inches, double ratio, bool turnLeft, bool forward);