#ifndef TASKS_HPP
#define TASKS_HPP

#include "main.h"

// PID Tuning Variables
extern float tune_kp, tune_ki, tune_kd, tune_start_i;

// Mode and Test Selectors
enum TuningMode { LATERAL, ANGULAR };
extern TuningMode currentMode; 
extern int test_index;
extern const char* test_names[];

void updateOdometry();
void odometryTask();
void startScreenTask();
void startTuningUI(); 
void sensorDirectionTest();
void motorDirectionTest();
void catapultControls();

#endif