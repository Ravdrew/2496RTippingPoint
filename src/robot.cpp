#include "main.h"
#include "robot.h"

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Motor leftFront(13, MOTOR_GEARSET_36, true, MOTOR_ENCODER_DEGREES);
pros::Motor leftMid(17, MOTOR_GEARSET_36, true, MOTOR_ENCODER_DEGREES); //kinda not good
pros::Motor leftBack(18, MOTOR_GEARSET_36, true, MOTOR_ENCODER_DEGREES);
pros::Motor rightFront(6, MOTOR_GEARSET_36, false, MOTOR_ENCODER_DEGREES);
pros::Motor rightMid(21, MOTOR_GEARSET_36, false, MOTOR_ENCODER_DEGREES);
pros::Motor rightBack(9 , MOTOR_GEARSET_36, false, MOTOR_ENCODER_DEGREES);
pros::Motor chainBar(5, MOTOR_GEARSET_36, true , MOTOR_ENCODER_DEGREES); //Gerald waz here
pros::Motor backLift(1, MOTOR_GEARSET_36, false, MOTOR_ENCODER_DEGREES);

pros::Imu imu(11);
pros::ADIDigitalOut chainClaw(6);
pros::ADIDigitalOut backClaw(7);
pros::ADIDigitalOut jSClamp(2);
pros::ADIDigitalOut stick(5);
pros::ADIDigitalIn chainLimit(8);
pros::ADIDigitalIn backLimit(3);
pros::ADIDigitalIn plus(1);

pros::ADIAnalogIn chainSense({{19, 1}});
pros::ADIAnalogIn backSense(4);

void reset_encoders(){
    leftFront.tare_position();
	leftMid.tare_position();
	leftBack.tare_position();
	rightFront.tare_position();
	rightMid.tare_position();
	rightBack.tare_position();
}