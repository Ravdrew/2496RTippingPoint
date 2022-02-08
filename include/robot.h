#include "main.h"

#ifndef ROBOT_H_
#define ROBOT_H_
extern pros::Motor leftFront; 
extern pros::Motor leftMid;
extern pros::Motor leftBack;
extern pros::Motor rightFront; 
extern pros::Motor rightMid;
extern pros::Motor rightBack;
extern pros::Motor chainBar;
extern pros::Motor backLift;

extern pros::Imu imu;
extern pros::Controller controller;
extern pros::ADIDigitalOut chainClaw;
extern pros::ADIDigitalOut backClaw;
extern pros::ADIDigitalOut jSClamp;
/*extern pros::ADIAnalogIn scooperPot;
extern pros::ADIAnalogIn liftPot;
extern pros::ADIDigitalIn minus;
extern pros::ADIDigitalIn plus;
extern pros::ADIDigitalIn scoopLimit;*/

void reset_encoders();

#endif