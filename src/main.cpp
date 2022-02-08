#include "main.h"
#include "robot.h"
#include "movement.h"
#include "PID.h"
//#define SCOOPER_GROUND 1150
//#define LIFT_GROUND 2115
//#define MAX_AUTOS 4


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */

void disabled() {
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
/*int selectedAuto = 0;

bool switchPressed = false;
bool resetNeeded = false;
int timer = 0;
void competition_initialize() {
	imu.reset();
	controller.clear();
	while (true) {
		if(minus.get_value() && !switchPressed){
			switchPressed = true;
			resetNeeded = true;
			selectedAuto -= 1;
		}
		else if(plus.get_value() && !switchPressed){
			switchPressed = true;
			resetNeeded = true;
			selectedAuto += 1;
		}
		else if(!plus.get_value() && !minus.get_value()){
			switchPressed = false;
		}

		if(selectedAuto > MAX_AUTOS){
			resetNeeded = true;
			selectedAuto = 0;
		}
		else if(selectedAuto < 0){
			resetNeeded = true;
			selectedAuto = MAX_AUTOS;
		}
		
		if (!(timer % 25)) {
			if(resetNeeded){
				resetNeeded = false;
				controller.clear();
			}
			if(selectedAuto == 0) controller.set_text(1, 0, "AWP");
			else if(selectedAuto == 1) controller.set_text(1, 0, "cP Goal Ally");
			else if(selectedAuto == 2) controller.set_text(1, 0, "cP Goal Hard");
			else if(selectedAuto == 3) controller.set_text(1, 0, "Prog Skills");
		}
		timer++;
		pros::delay(2);
	}
}*/

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
/*
void noAuto(){}

void progSkills(){
	lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	scooper.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	autonOn = true;
	scooperSetLowLimit();
	chas_move(-127, -127);
	pros::delay(250);
	chas_move(0,0);
	liftTurnTo(LIFT_GROUND - 1250, -127);
	scooperTurnTo(SCOOPER_GROUND + 2200, 127);
	absturn(90, true, 3);
	move(500, true, 3);
	absturn(180, true, 3);
	liftTurnTo(LIFT_GROUND + 133, -127);
	move(1100, true, 3);
	liftTurnTo(LIFT_GROUND - 1140, 127);
	lift.move(8);
	move(700, true, 3);
	absturn(90, true, 3);
	move(-900);
	move(300, true, 3);
	absturn(180, true, 3);
	move(1040, true, 3);
	absturn(90, true, 3);
	liftTurnTo(LIFT_GROUND - 700, 127);
	//lift.move(12);
	scooper.move(8);
}

void cpGoalHard(){
	autonOn = true;
	pros::Task liftSetupHard(liftSetLow);
	move(1200, false, 0, 127, 0, true);//90, 300 int target, bool ask_slew = false, int slew_rate = 0, int power_cap = 127, int active_cap = 0, bool cP = false;);
	pros::Task liftRaiseHard(liftSetHigh);
	pros::delay(50);
	chas_move(-127, 127);
	pros::delay(400);
	chas_move(0, 0);
	pros::delay(300);
	absturn(0, true, 3);
	move(-1200, false, 0, 90, 450);
	absturn(45, true, 3);
	move(-400, true, 3);
	absturn(-70, true, 3);
	pros::Task scooperDropHard(scooperSetLow);
	chas_move(-100, -100);
	pros::delay(600);
	chas_move(0, 0);
	pros::delay(50);
	move(600, true, 20);
	autonOn = false;
	lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	scooper.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lift.move(30);
	scooper.move(8);
}

void cPGoalAlly(){
	autonOn = true;
	pros::Task liftSetup(liftSetLow);
	move(1045, false, 0, 127, 0, true); //90, 300 int target, bool ask_slew = false, int slew_rate = 0, int power_cap = 127, int active_cap = 0, bool cP = false;);
	pros::Task liftRaise(liftSetHigh);
	pros::delay(50);
	chas_move(-127, 127);
	pros::delay(400);
	chas_move(0, 0);
	pros::delay(300);
	absturn(0, true, 3);
	move(-960, false, 0, 90, 450);
	absturn(-90, true, 3);
	//autonOn = false;
	pros::Task scooperDrop(scooperSetLow);
	pros::delay(200);
	chas_move(-100, -100);
	pros::delay(450);
	chas_move(0, 0);
	pros::delay(50);
	move(250, true, 20);
	pros::delay(100);
	move(-495, true, 30);
	scooperTurnTo(SCOOPER_GROUND + 300, 127);
	lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	scooper.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	lift.move(12);
	scooper.move(8);
	move(500, true, 5);
}

void AWP(){
	pros::Task scooperDrop(scooperDropRing);
	move(-370, true, 5, 110);
	move(200, true, 5);
	pros::Task scooperReset(scooperReturn);
	move(-426, true, 5, 92);
	move(245, true, 5);
	absturn(130, true, 5);
	move(600, true, 5);
	absturn(90);
	move(1600, true, 5);
	absturn(345);
	move(150, true, 5);
	liftTurnTo(LIFT_GROUND - 450, -127);
	liftTurnTo(LIFT_GROUND - 1250, 127);
}*/

void autonomous() {
	backLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	chainBar.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	backClaw.set_value(false);
	backLift.move(-127);
	chas_move(-127, -127);
	pros::delay(650);
	chas_move(-60, -60);
	backLift.move(0);
	pros::delay(550);
	chas_move(0,0);//
	backClaw.set_value(true);
	pros::delay(100);
	//backLift.move(127);
	chas_move(127, 127);
	pros::delay(600);
	//backLift.move(127);
	pros::delay(400);
	chas_move(0, 0);


	/*if(selectedAuto == 0) AWP();
	else if(selectedAuto == 1) cPGoalAlly();
	else if(selectedAuto == 2) cpGoalHard();
	else if(selectedAuto == 3) progSkills();
	else if (selectedAuto == 4) noAuto();*/

	//backLift.move_absolute(target, speed)
	//chainBar.move_absolute(target, speed)
	
	//Half Awp
	//backLift.move_absolute(target, speed)
	//chainBar.move_absolute(target, speed)
	//chainClaw.set_value(false);
	

	//Neutral
	/*backLift.move(-127);
	pros::delay(200)
	backLift.move(0);*/
	//chainBar.move_absolute(target, speed)
	/*chas_move(127, 127);
	pros::delay(400);
	chas_move(0,0);
	backClaw.set_value(true);
	//backLift.move_absolute(target, speed)
	chas_move(-127, -127);
	pros::delay(300);
	chas_move(0,0);
	backLift.move(30);
	chas_move(-80, 80);
	pros::delay(200);
	chas_move(0,0);*/

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

bool openChain = false;
bool openJS = true;
bool openBack = false;
bool backOnCode = false;
void opcontrol() {
	chainClaw.set_value(false);
	backClaw.set_value(false);
	chainBar.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	backLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	while (true) {
		
//您好我叫甘安卓，大家好我叫陆遥
		if(controller.get_digital(DIGITAL_L1)){
			chainBar.move(127);
		}       
		else if(controller.get_digital(DIGITAL_L2)){
			chainBar.move(-127);
		}    
		else{ 
			chainBar.move_velocity(0);
		}

		if(controller.get_digital(DIGITAL_R1)){
			backLift.move(-127);
			backOnCode = false;
		}       
		else if(controller.get_digital(DIGITAL_R2)){
			backLift.move(127);
			backOnCode = true;
		}    
		else if(backOnCode){ 
			backLift.move(15);
		}
		else{
			backLift.move(0);
		}

		if(controller.get_digital_new_press(DIGITAL_Y)){
			openChain = !openChain;
			if(openChain){
				chainClaw.set_value(true);
			}
			else{
				chainClaw.set_value(false);
			}
		}

		if(controller.get_digital_new_press(DIGITAL_RIGHT)){
			openBack = !openBack;
			if(openBack){
				backClaw.set_value(true);
			}
			else{
				backClaw.set_value(false);
			}
		}
		
		if(controller.get_digital_new_press(DIGITAL_DOWN)){
			openJS = !openJS;
			if(openJS){
				jSClamp.set_value(true);
			}
			else{
				jSClamp.set_value(false);
			}
		}
		
		int lPwr, rPwr; //forwardpower and turnpower
		int rYaxis, lXaxis; //controller axis 
		rYaxis = controller.get_analog(ANALOG_RIGHT_Y);
		lXaxis = controller.get_analog(ANALOG_LEFT_Y);
		rPwr = (abs(rYaxis) > 2) ? rYaxis : 0;
		lPwr = (abs(lXaxis) > 2) ? lXaxis : 0;  
		chas_move(lPwr, rPwr);
//gerald was here
		pros::delay(20);
	}
}
