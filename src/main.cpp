#include "main.h"
#include "robot.h"
#include "movement.h"
#include "PID.h"

#define MAX_AUTOS 7
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
	/*controller.clear();
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
			if(selectedAuto == 1) controller.set_text(1, 0, "AWP"); //will be awp
			else if(selectedAuto == 2) controller.set_text(1, 0, "Plat Down");
			else if(selectedAuto == 3) controller.set_text(1, 0, "Line Side");
			else if(selectedAuto == 4) controller.set_text(1, 0, "Elims");
			else if(selectedAuto == 5) controller.set_text(1, 0, "Rush Mid");
			else if(selectedAuto == 6) controller.set_text(1, 0, "Prog Skills");
			else if(selectedAuto == 7) controller.set_text(1, 0, "No Auton");
		}
		timer++;
		pros::delay(2);
	}*/
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
int autoChainTarget = -10;
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
	int count = 0;
	while(backLimit.get_value() == 0 && count < 150){
		backLift.move(-127);
		++count;
		pros::delay(10);
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

void turnTune(){
	absturn(-90, false);
	backClawOpen();
	pros::delay(1000);
	absturn(0, false);
	backClawClose();
}

void AWP(){
	jsClawOpen();
	pros::delay(200);
	backClawOpen();
	move(-60);
	absturn(-90, false);
	move(180);
	absturn(0, false);
	move(910);
	absturn(-5, false);
	chainClawOpen();
	chas_move(70, 70);
	pros::delay(200);
	chas_move(0,0);
	chainBar.move(-80);
	pros::delay(1500);
	chainBar.move(0);
	chainBar.move(127);
	pros::delay(200);
	chainBar.move(0);
	move(-160);
	pros::Task chainAutonT(autonChainBr);
	pros::Task backAutonT(autonBackBr);
	absturn(-90, false);
	moveTillChain(127, 70);
	chas_move(-100,-100);
	pros::delay(700);
	chas_move(0,0);


}

void rushMid(){
	backClawOpen();
	jsClawOpen();
	chainClawOpen();
	pros::Task backAutonT(autonBackBr);
	goalYoink();
	pros::Task chainAutonT(autonChainBr);
	pros::delay(200);
	moveTillChain(80, 100);
	autoChainTarget = 700;
	pros::delay(1000);
	absturnTimed(-180, true, 330);
	/*pros::Task backAutonT(autonBackBr);
	absturn(-180);
	autoChainTarget = -30;
	moveTillBack(-100, 100);
	autoBackTarget = 100;*/

}

void platDown(){
	chainBar.move(-127);
	jsClawOpen();
	pros::delay(200);
	chainBar.move(0);
	backClawOpen();
	move(-60);
	pros::Task backAutonT(autonBackBr);
	absturn(107, false);
	moveTillBack(-100, 120);
	autoBackTarget = 100;
	move(370);
	autoBackTarget = 300;
}

void lineSide(){
	backClawOpen();
	jsClawOpen();
	chainClawOpen();
	move(165);
	absturn(90, false);
	move(145);
	absturn(84, false);
	chainBar.move(-100);
	pros::delay(1400);
	chainBar.move(0);
	pros::delay(400);
	chainBar.move(127);
	pros::delay(600);
	chainBar.move(0);
	move(-147);
	pros::Task chainAutonT(autonChainBr);
	pros::Task backAutonT(autonBackBr);
	absturn(180, false);
	moveTillBack(-80, 95);
	chas_move(127,127);
	pros::delay(800);
	chas_move(0,0);
	absturnTimed(0, true, 400);
}

void elims(){
	backClawOpen();
	jsClawOpen();
	chainClawOpen();
	goalYoink();
	pros::Task chainAutonT(autonChainBr);
	absturnTimed(-36, false);
	chainClawOpen();
	stickUp();
	moveTillChain(127, 92);
	move(-369);
	pros::Task backAutonT(autonBackBr);
	absturnTimed(-173, true, 350);
	/*chas_move(-50, 50);
	pros::delay(200);
	chas_move(0,0);*/
	moveTillBack(-80, 120);
	move(270);
}

void noAuton(){
	backClawOpen();
	jsClawOpen();
	chainClawOpen();
	pros::Task chainAutonT(autonChainBr);
	pros::Task backAutonT(autonBackBr);
}

void progSkills(){}

void testing(){
	backClawOpen();
	jsClawOpen();
	chainClawOpen();
}


void autonomous() {
	start_heading = imu.get_rotation();
	backLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	chainBar.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	if(selectedAuto == 1) AWP(); //testing();
	else if(selectedAuto == 2) platDown();
	else if(selectedAuto == 3) lineSide();
	else if(selectedAuto == 4) elims();
	else if(selectedAuto == 5) rushMid();
	else if(selectedAuto == 6) progSkills();
	else if(selectedAuto == 7) noAuton();
	
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

void autonSelect(){
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
	
	if (!(timer % 5)) {
		if(resetNeeded){
			resetNeeded = false;
			controller.clear();
		}
		if(selectedAuto == 1) controller.set_text(1, 0, "AWP"); //will be awp
		else if(selectedAuto == 2) controller.set_text(1, 0, "Plat Down");
		else if(selectedAuto == 3) controller.set_text(1, 0, "Line Side");
		else if(selectedAuto == 4) controller.set_text(1, 0, "Elims");
		else if(selectedAuto == 5) controller.set_text(1, 0, "Rush Mid");
		else if(selectedAuto == 6) controller.set_text(1, 0, "Prog Skills");
		else if(selectedAuto == 7) controller.set_text(1, 0, "No Auton");
	}
	timer++;
}

void opcontrol() {
	autonRunning = false;
	openStick = false;
	chainBar.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	backLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	//pros::Task backDriver(backDriverPID);
	//pros::Task chainDriver(chainDriverPID);
	controller.clear();
	
	while (true) {
		autonSelect();
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
