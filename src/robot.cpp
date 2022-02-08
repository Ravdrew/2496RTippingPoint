#include "main.h"
#include "robot.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor leftFront(19, MOTOR_GEARSET_36, true, MOTOR_ENCODER_DEGREES);
pros::Motor leftMid(17, MOTOR_GEARSET_36, true, MOTOR_ENCODER_DEGREES);
pros::Motor leftBack(18, MOTOR_GEARSET_36, false, MOTOR_ENCODER_DEGREES);
pros::Motor rightFront(10, MOTOR_GEARSET_36, false, MOTOR_ENCODER_DEGREES);
pros::Motor rightMid(9, MOTOR_GEARSET_36, true, MOTOR_ENCODER_DEGREES);
pros::Motor rightBack(8, MOTOR_GEARSET_36, false, MOTOR_ENCODER_DEGREES);
pros::Motor chainBar(5, MOTOR_GEARSET_36, false, MOTOR_ENCODER_DEGREES); //Gerald waz here
pros::Motor backLift(1, MOTOR_GEARSET_36, false, MOTOR_ENCODER_DEGREES);

//pros::ADIAnalogIn scooperPot(4);//I <3 code
//pros::ADIAnalogIn liftPot(6);
//pros::Imu imu(7);
pros::ADIDigitalOut chainClaw(5);
pros::ADIDigitalOut backClaw(8);
pros::ADIDigitalOut jSClamp(7);
//pros::ADIDigitalIn minus(3);
//pros::ADIDigitalIn plus(5);
//pros::ADIDigitalIn scoopLimit(7);


void reset_encoders(){
    leftFront.tare_position();
	leftMid.tare_position();
	leftBack.tare_position();
	rightFront.tare_position();
	rightMid.tare_position();
	rightBack.tare_position();
}