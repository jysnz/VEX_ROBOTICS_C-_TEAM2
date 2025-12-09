#include "vex.h"
using namespace vex;



brain Brain;

// ------------------- MOTORS & DRIVE -------------------
motor right_motor_f  = motor(PORT1,  ratio18_1, true);
motor right_motor_b  = motor(PORT5,  ratio18_1, true);
motor_group right_drive_smart(right_motor_f, right_motor_b);

motor left_motor_f = motor(PORT3, ratio18_1, false);
motor left_motor_b = motor(PORT4, ratio18_1, false);
motor_group left_drive_smart(left_motor_f, left_motor_b);

drivetrain driveTrain(left_drive_smart, right_drive_smart,
    319.19, 317.5, 300, mm, 0.2857142857142857);

    // Top long goal
motor motor6 = motor(PORT6, ratio18_1, true);
// Inside middle
motor motor7 = motor(PORT7, ratio18_1, true);
// Middle goal
motor motor8 = motor(PORT8, ratio18_1, true);
// Basket
motor motor20 = motor(PORT20, ratio18_1, true);
// Arm
motor motor19 = motor(PORT19, ratio18_1, true);

bool arm = false;

// ------------------- CONFIGURATION -------------------
const int LEFT_SPEED  = 90;
const int RIGHT_SPEED = 90;   
const double SETTLE_TIME = 0.15; 

const double TICKS_PER_CM = 15;     
const double TICKS_PER_DEG_TURN = 5; 

// ------------------- MOVEMENT FUNCTIONS -------------------
void drive_cm(double distance_cm, double left_speed, double right_speed, bool intake_bool, double intake_wait) {
    double ticks = distance_cm * TICKS_PER_CM;

    left_motor_f.setVelocity(left_speed, pct);
    left_motor_b.setVelocity(left_speed, pct);
    right_motor_f.setVelocity(right_speed, pct);
    right_motor_b.setVelocity(right_speed, pct);

    left_drive_smart.spinFor(fwd, ticks, deg, false);
    right_drive_smart.spinFor(fwd, ticks, deg, true);

    if (intake_bool){
       motor7.spin(fwd, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(reverse, 100, pct);
            
            wait(intake_wait,sec);
            motor8.stop();
            motor7.stop();
            motor20.stop();
    }
    wait(SETTLE_TIME, sec);
}

void turn_deg(double degrees, double left_speed, double right_speed) {
    double ticks = fabs(degrees) * TICKS_PER_DEG_TURN;
    left_speed /= 2;
    right_speed /= 2;

    left_motor_f.setVelocity(left_speed, pct);
    left_motor_b.setVelocity(left_speed, pct);
    right_motor_f.setVelocity(right_speed, pct);
    right_motor_b.setVelocity(right_speed, pct);

    if(degrees > 0) { 
        left_drive_smart.spinFor(fwd, ticks, deg, false);
        right_drive_smart.spinFor(reverse, ticks, deg, true);
    } else { 
        left_drive_smart.spinFor(reverse, ticks, deg, false);
        right_drive_smart.spinFor(fwd, ticks, deg, true);
    }

    wait(SETTLE_TIME, sec);
}

void warmup_motors(double duration_sec = 0.1) {
    left_drive_smart.spinFor(fwd, 10, deg, false);
    right_drive_smart.spinFor(fwd, 10, deg, true);
    wait(duration_sec, sec);
}

void drive_for(double seconds, int speed, bool intake_bool,double intake_wait) {
    int direction = (seconds >= 0) ? 1 : -1;

    left_drive_smart.setVelocity(speed, pct);
    right_drive_smart.setVelocity(speed, pct);

    left_drive_smart.spin(direction == 1 ? fwd : reverse);
    right_drive_smart.spin(direction == 1 ? fwd : reverse);
    
    intake_wait /= 3;
    double intake_stop= 0.5;
    if (intake_bool){
       motor7.spin(fwd, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(reverse, 100, pct);
            
            wait(intake_wait,sec);
            motor8.stop();
            motor7.stop();
            motor20.stop();

            motor7.spin(fwd, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(reverse, 100, pct);

            wait(intake_stop,sec);
            motor7.spin(fwd, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(reverse, 100, pct);

            motor7.spin(fwd, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(reverse, 100, pct);

            wait(intake_wait,sec);
             motor8.stop();
            motor7.stop();
            motor20.stop();

            motor7.spin(fwd, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(reverse, 100, pct);

        wait(intake_wait,sec);
             motor8.stop();
            motor7.stop();
            motor20.stop();

    } 
    
    wait(fabs(seconds), sec);
    
    left_drive_smart.stop(brake);
    right_drive_smart.stop(brake);
}

// ------------------- MOTOR TEMP COLOR FUNCTION -------------------
void updateMotorTemperatureColors() {
    double temps[] = {
        left_motor_f.temperature(celsius),
        left_motor_b.temperature(celsius),
        right_motor_f.temperature(celsius),
        right_motor_b.temperature(celsius)
    };

    double maxTemp = temps[0];
    for(int i = 1; i < 4; i++) {
        if(temps[i] > maxTemp) maxTemp = temps[i];
    }

    if(maxTemp < 45) Brain.Screen.setFillColor(green);
    else if(maxTemp < 55) Brain.Screen.setFillColor(orange);
    else Brain.Screen.setFillColor(red);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("Max Motor Temp: %.1f C", maxTemp);
}
void waitUntilStall(motor &m) {
    wait(200, msec);
    while(true) {
        if(fabs(m.velocity(pct)) < 1) {
            m.stop(hold);
            break;
        }
        wait(10, msec);
    }
}

void toggleArm() {
    if(arm) {
        motor19.spin(fwd, 100, pct);
        waitUntilStall(motor19);
        arm = false;
    } else {
        motor19.spin(reverse, 100, pct);
        waitUntilStall(motor19);
        arm = true;
    }
}

void arm_up(double wait_seconds){
    motor19.spin(reverse, 100, pct);
    waitUntilStall(motor19);       
    wait(wait_seconds, sec);        
}

void arm_down(double wait_seconds){
    motor19.spin(fwd, 100, pct);
    waitUntilStall(motor19);        
    wait(wait_seconds, sec);     
}

void long_goal(double wait_time){
           motor6.spin(fwd, 100, pct);
            motor7.spin(reverse, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(fwd, 100, pct);
            wait(wait_time,sec);
            motor6.stop();
            motor7.stop();
            motor8.stop();  
            motor20.stop();
    }

    void upper_middle(double wait_time){
           motor6.spin(reverse, 100, pct);
            motor7.spin(reverse, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(fwd, 100, pct);
            wait(wait_time,sec);
            motor6.stop();
            motor7.stop();
            motor8.stop();
            motor20.stop();
    }
  void outtake(double wait_time){
           motor8.spin(reverse, 100, pct);
            motor7.spin(reverse, 100, pct);
            motor20.spin(fwd, 100, pct);
            wait(wait_time,sec);
            motor8.stop();
            motor7.stop();
            motor20.stop();
            
    }
void intake(double wait_time){
           motor7.spin(fwd, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(reverse, 100, pct);
            wait(wait_time,sec);
            motor8.stop();
            motor7.stop();
            motor20.stop();
    }

// ------------------- AUTONOMOUS ROUTINE -------------------
void autonomous() {
    // Warmup motors
    //warmup_motors();
   /*     for(int i=0; i<20; i++) { // arbitrary loop to simulate movement monitoring
        updateMotorTemperatureColors();
        wait(100, msec);
    }
    // Example routine
    arm_up(0.1);
    drive_cm(97.5);    // move forward 60 cm
      //150 onwards left side get ahead 
    turn_deg(61.5);// in the current tick the 90 angle is over by 50%
    
    
    arm_down(0.1);
    drive_for(.35, 80,true,2.2);

    
    drive_cm(-15);
    turn_deg(-132.8);
    arm_up(0.1);
    intake();
        drive_cm(44.8);
    long_goal(6);

    drive_cm(-10);
    drive_cm(14);
    drive_cm(-10);
    drive_cm(13);
    drive_cm(-10);
    drive_cm(13);
    */
  // /*
    //skills matchload
    arm_down(0.1);
    drive_cm(99,90,90,false,0);
    turn_deg(62.1,60,60);
    
  // */
  // /*
    drive_cm(29.53,75,75,true,1);// adjust time in intake 

  // */
   drive_cm(-15,50,50, false, 0);
   drive_cm(16, 80,80 , true, 3);
  // /*
    //intake(2.2);// .

    drive_cm(-15,50,50, false ,0 ); //. 
    turn_deg(-126.5,75,75); //.
    arm_up(0.1);//. 
/*
   drive_for(.25, 50,true,2.2);//,
   long_goal(7);//.
*/
    drive_cm(45.3,60,60,false,0);
    long_goal(8);
}

// ------------------- MAIN -------------------
int main() {


    // Run autonomous routine
    autonomous();

    // Continue monitoring motor temperature indefinitely
    while(true) {
        updateMotorTemperatureColors();
        wait(100, msec);
    }
}
