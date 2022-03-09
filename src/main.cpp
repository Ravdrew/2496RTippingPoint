#include "main.h"
#include "robot.h"
#include "movement.h"
#include "PID.h"

#define MAX_AUTOS 3
#define CHAIN_KP 5.5
#define CHAIN_KI 0.1
#define CHAIN_KD 0.0

#define BACK_KP 7
#define BACK_KI 0.1
#define BACK_KD 0.0

#define INTEGRAL_KICK_IN 50
#define MAX_INTEGRAL 40


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
int selectedAuto = 1;

bool switchPressed = false;
bool resetNeeded = false;

int timer = 0;
void competition_initialize() {
	controller.clear();
	while (true) {

		if(plus.get_value() && !switchPressed){
			switchPressed = true;
			resetNeeded = true;
			selectedAuto += 1;
		}
		else if(plus.get_value() == false){
			switchPressed = false;
		}

		if(selectedAuto > MAX_AUTOS){
			resetNeeded = true;
			selectedAuto = 1;
		}
		
		if (!(timer % 25)) {
			if(resetNeeded){
				resetNeeded = false;
				controller.clear();
			}
			if(selectedAuto == 1) controller.set_text(1, 0, "Testing");
			else if(selectedAuto == 2) controller.set_text(1, 0, "Prog Skills");
			else if(selectedAuto == 3) controller.set_text(1, 0, "No Auton");
		}
		timer++;
		pros::delay(2);
	}
}

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
bool autonRunning = true;
int autoChainTarget = -30;
int autoBackTarget = 0;

void autonChainBr(void * param){
	while(chainLimit.get_value() == 0){
		chainBar.move(-127);
	}
	chainBar.tare_position();
		
	PID chainPID(CHAIN_KP, CHAIN_KI, CHAIN_KD);
	float chain_voltage;

	while(autonRunning){
		chain_voltage = chainPID.calc(autoChainTarget, chainBar.get_position(), INTEGRAL_KICK_IN, MAX_INTEGRAL, 0, false);
		chainBar.move(chain_voltage);

		pros::delay(10);
	}

	chainBar.move(0);
}

void autonBackBr(void * param){
	pros::delay(300);

	while(backLimit.get_value() == 0){
		backLift.move(-127);
	}
	backLift.move(0);
	backLift.tare_position();

	PID backPID(BACK_KP, BACK_KI, BACK_KD);
	float back_voltage;

	while(autonRunning){
		back_voltage = backPID.calc(autoBackTarget, backLift.get_position(), INTEGRAL_KICK_IN, MAX_INTEGRAL, 0, false);
		backLift.move(back_voltage);
		
		pros::delay(10);
	}

	backLift.move(0);
}

void testing(){
	//pros::Task chainAutonT(autonChainBr);
	//pros::Task backAutonT(autonBackBr);
	goalYoink();
	//stickDown();
	//pros::delay(2000);
	//postGoalReset();
	pros::Task chainAutonT(autonChainBr);
	absturn(-33);
	chainClawOpen();
	stickUp();
	moveTillChain(127, 100);
	move(-370);
	pros::Task backAutonT(autonBackBr);
	autoChainTarget = 60;
	absturn(-180);
	moveTillBack(-60, 100);
	//backClawOpen();
	jsClawOpen();
	//pros::delay(300);
	//moveTillBack(-60, 60)

	//absturn(45);
	//move(-50);
}

void rushMidClose(){
	goalYoink();
	pros::Task chainAutonT(autonChainBr);
	autoChainTarget = 100;
	pros::delay(10);
	stickUp();
	//move(-25);
	absturn(90);
	move(200);//can try using a time based movement here
	pros::delay(10);
	chainClawOpen();
	move(-200);
	/*pros::Task backAutonT(autonBackBr);
	absturn(-180);
	autoChainTarget = -30;
	moveTillBack(-100, 100);
	autoBackTarget = 100;*/

}

void soloAWPRush(){}

void noAuton(){}

void progSkills(){}


void autonomous() {
	start_heading = imu.get_rotation();
	backLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	chainBar.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


	if(selectedAuto == 1) rushMidClose(); //testing();
	else if(selectedAuto == 2) progSkills();
	else if(selectedAuto == 3) noAuton();
	
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

bool backOnCode = false;
void noAutoLiftsChain(){
	if(controller.get_digital(DIGITAL_L1)){
		chainBar.move(-127);
	}       
	else if(controller.get_digital(DIGITAL_L2)){
		chainBar.move(127);
	}    
	else{ 
		chainBar.move(0);
	}
}

void noAutoLiftsBack(){
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
}

bool openChain = true;
bool openJS = false;
bool openBack = true;
bool openStick = false;
bool autoLiftOn = false;
int chainIndex = 0;
int backIndex = 0;
//int chainTargets[] = {0, 550, 930}; //0, 1180 ,1700
//int backTargets[] = {0, 350, 480}; //0, 700, 950

void autoLifts(){
	if(controller.get_digital(DIGITAL_L1)){
		chainBar.move(-127);
	}       
	else if(controller.get_digital(DIGITAL_L2)){
		chainBar.move(127);
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
	
}

void opcontrol() {
	autonRunning = false;
	chainBar.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	backLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	//pros::Task backDriver(backDriverPID);
	//pros::Task chainDriver(chainDriverPID);
	
	while (true) {
		if(controller.get_digital_new_press(DIGITAL_LEFT)){
			autoLiftOn = !autoLiftOn;
		}

		if(autoLiftOn == true){
			autoLifts();
		}
		else{
			noAutoLiftsBack();
			noAutoLiftsChain();
		}

		if(controller.get_digital_new_press(DIGITAL_Y)){
			openChain = !openChain;
			chainClaw.set_value(openChain);
		}

		if(controller.get_digital_new_press(DIGITAL_RIGHT)){
			openBack = !openBack;
			backClaw.set_value(openBack);
		}
		
		if(controller.get_digital_new_press(DIGITAL_DOWN)){
			openJS = !openJS;
			jSClamp.set_value(openJS);
		}

		if(controller.get_digital_new_press(DIGITAL_X)){
			openStick = !openStick;
			stick.set_value(openStick);
		}
		
		int lPwr, rPwr;
		int rYaxis, lXaxis;
		rYaxis = controller.get_analog(ANALOG_RIGHT_Y);
		lXaxis = controller.get_analog(ANALOG_LEFT_Y);
		rPwr = (abs(rYaxis) > 2) ? rYaxis : 0;
		lPwr = (abs(lXaxis) > 2) ? lXaxis : 0;  
		chas_move(lPwr, rPwr);

		pros::delay(20);
	}
}
