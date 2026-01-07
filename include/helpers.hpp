#include "robot_config.hpp"

// Add these EXTERN declarations so other files can see them without re-creating them
extern double x, y, theta, heading;
extern float kP, kI, kD;
extern const double trackWidth;
extern double prevLeft, prevRight;
extern const float PI;
extern float tune_kp, tune_ki, tune_kd, tune_start_i;

float ticksToInches(float ticks);
double velocityToVoltage(double velocity);
double inchesToDegrees(double inches);

void drive_to_object(double maxSpeed, double targetInches, int timeoutMs);
void reverse_to_object_smooth(double maxSpeed, double targetInches, int timeoutMs);
void drive_for_inches_consistent(double maxSpeedVelocity, double inches);
void drive_backward_consistent(double maxSpeedVelocity, double inches);
void turn_time_consistent(double maxSpeedVelocity, int durationMs, bool turnLeft);
void drive_arc_consistent(double maxSpeedVelocity, double inches, double ratio, bool turnLeft, bool forward);
void drive_back_and_forth(double times, double speed, double seconds);
void eat_ball(double milliseconds, double velocity);
void spit_ball(double milliseconds, double velocity);
void get_matchload(double milliseconds, double velocity, bool twoVtwoV2=false);
void score_long_goal(double angle, double velocity);
void detect_wall_to_score(double targetInches);
void drive_for_inches(double maxSpeed, double inches);
void drive_backward_for_inches(double maxSpeed, double inches);
void fire_catapult_safe(double targetInches);

