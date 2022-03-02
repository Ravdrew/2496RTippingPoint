#include "main.h"
#include "PID.h"
#include "movement.h"
#include "robot.h"
#define STRAIGHT_KP 0.55 //0.39 0.5
#define STRAIGHT_KI 0.2 //0.15
#define STRAIGHT_KD 0.0
#define TURN_KP 1.6 //1.2
#define TURN_KI 0.23 //0.4
#define TURN_KD 0.0
#define INTEGRAL_KICK_IN 50
#define MAX_INTEGRAL 40

#define COUNT_CONST 25 //23

//bool autonOn;

void chas_move(int left_power, int right_power){
	leftFront.move(left_power);
	leftMid.move(left_power);
	leftBack.move(left_power);
	rightFront.move(right_power);
	rightMid.move(right_power);
	rightBack.move(right_power);
}

void chainClawOpen(){
	chainClaw.set_value(true);
}

void chainClawClose(){
	chainClaw.set_value(false);
}

void backClawOpen(){
	backClaw.set_value(true);
}

void backClawClose(){
	backClaw.set_value(false);
}

void jsClawOpen(){
	jSClamp.set_value(true);
}

void jsClawClose(){
	jSClamp.set_value(false);
}

void move(int target, bool ask_slew, int slew_rate, int power_cap, int active_cap, bool cP){
    PID straight(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);

    float voltage;
    float encoder_average;

    float imu_offset;
    float heading;
    int count = 0;

    reset_encoders();
    imu_offset = imu.get_rotation();
    while(true){
        
        encoder_average = (leftBack.get_position() + rightBack.get_position())/2;
        
        voltage = straight.calc(target, encoder_average, INTEGRAL_KICK_IN, MAX_INTEGRAL, slew_rate, ask_slew);

        heading = imu.get_rotation() - imu_offset;

        //std::abs(voltage) > power_cap ? voltage = power_cap*voltage/std::abs(voltage) : voltage = voltage;
        if(active_cap != 0){
            if(abs(target-encoder_average) <= active_cap){
                std::abs(voltage) > power_cap ? voltage = power_cap*voltage/std::abs(voltage) : voltage = voltage;
            }
        }
        else{
            std::abs(voltage) > power_cap ? voltage = power_cap*voltage/std::abs(voltage) : voltage = voltage;
        }
        chas_move(voltage - heading, voltage + heading); // (voltage - heading, voltage + heading)
        printf("error: %f\r\n", voltage);
        if (abs(target - encoder_average) <= 3) count++;
        if (count >= COUNT_CONST) break;
        else if(cP && count >= 5) break;

        pros::delay(10);
    }
    
    chas_move(0,0);
}

void goalYoink(bool far){
    PID yoink(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);

    float voltage;
    float encoder_average;

    float imu_offset;
    float heading;
    float target;
    bool stick_down = false;

    reset_encoders();
    imu_offset = imu.get_rotation();

    if(far) target = 2000;
    else target = 1800;
    while(true){ 
        encoder_average = (leftBack.get_position() + rightBack.get_position())/2;
        
        voltage = yoink.calc(target, encoder_average, INTEGRAL_KICK_IN, MAX_INTEGRAL, 0, false);

        heading = imu.get_rotation() - imu_offset;

        chas_move(voltage - heading, voltage + heading); // (voltage - heading, voltage + heading)
        if (stick_down == false && abs(target - encoder_average) <= 100) stick.set_value(true);
        if (abs(target - encoder_average) <= 3) break;

        pros::delay(10);
    }

    while(true){ 
        encoder_average = (leftBack.get_position() + rightBack.get_position())/2;
        
        voltage = yoink.calc(target*-1+1300, encoder_average, INTEGRAL_KICK_IN, MAX_INTEGRAL, 0, false);

        heading = imu.get_rotation() - imu_offset;

        chas_move(voltage - heading, voltage + heading); // (voltage - heading, voltage + heading)
        if (abs(target+1300 - encoder_average) <= 3) break;

        pros::delay(10);
    }
    chas_move(0,0);
}

void turn(int target, bool ask_slew, int slew_rate){
    PID rotate(TURN_KP, TURN_KI, TURN_KD);

    float voltage;
    float position;
    int count = 0;
    float imu_start;

    imu_start = imu.get_rotation();

    while(true){
        position = imu.get_rotation() - imu_start;
        voltage = rotate.calc(target, position, INTEGRAL_KICK_IN, MAX_INTEGRAL, slew_rate, ask_slew);

        chas_move(voltage, -voltage);
        printf("error: %f\r\n", (imu.get_rotation()));
        if (abs(target - position) <= 2) count++;
        if (count >= COUNT_CONST) break;

        pros::delay(10);
    }

    chas_move(0,0);
    printf("count: %d\r\n", (count));
}
void absturn(int abstarget, bool ask_slew, int slew_rate, int power_cap){
   PID absRotate(TURN_KP, TURN_KI, TURN_KD);
  
   float voltage;
   float position;
   int count = 0;
 
   while(true){
       position = std::fmod(imu.get_rotation(), 360);
       voltage = absRotate.calc(abstarget, position, INTEGRAL_KICK_IN, MAX_INTEGRAL, slew_rate, ask_slew);
       std::abs(voltage) > power_cap ? voltage = power_cap*voltage/std::abs(voltage) : voltage = voltage;
       chas_move(voltage, -voltage);
       printf("error: %f\r\n", (imu.get_rotation()));
       if (abs(abstarget - position) <= 1.5) count++;
       if (count >= COUNT_CONST) break;
 
       pros::delay(10);
   }
   chas_move(0,0);
   printf("count: %d\r\n", (count));
}

bool chainGoalDetected(){
    if(chainSense.get_value() < 2000) return true;
    return false;
}

bool backGoalDetected(){
    if(backSense.get_value() < 2000) return true;
    return false;
}

void moveTillChain(int speed, int timer){
    int count = 0;
    while(chainGoalDetected() == false){
        chas_move(speed, speed);
        ++count;
        if(count > timer) break;

        pros::delay(10);
    }
    chainClawClose();
    chas_move(0,0);
}

void moveTillBack(int speed, int timer){
    int count = 0;
    while(backGoalDetected() == false){
        chas_move(speed, speed);
        ++count;
        if(count > timer) break;

        pros::delay(10);
    }
    backClawClose();
    chas_move(0,0);
}


//Ex drive PID:
/*void chainDriverPID(void * param){
	PID chainPID(CHAIN_KP, CHAIN_KI, CHAIN_KD);
	float chain_voltage;

	while(true){
		if(autoLiftOn == true){
			chain_voltage = chainPID.calc(chainTargets[chainIndex], chainBar.get_position(), INTEGRAL_KICK_IN, MAX_INTEGRAL, 0, false);
			chainBar.move(chain_voltage);
		}

		else{
			noAutoLiftsChain();
		}
	}
}

void backDriverPID(void * param){
	PID backPID(BACK_KP, BACK_KI, BACK_KD);
	float back_voltage;

	while(true){
		if(autoLiftOn == true){
			back_voltage = backPID.calc(backTargets[backIndex], backLift.get_position(), INTEGRAL_KICK_IN, MAX_INTEGRAL, 0, false);
			backLift.move(back_voltage);
		}

		else{
			noAutoLiftsBack();
		}

	}
if(controller.get_digital_new_press(DIGITAL_L2)){
		chainIndex += 1;
		if(chainIndex > 2) chainIndex = 2;
	}       
	else if(controller.get_digital_new_press(DIGITAL_L1)){
		chainIndex -= 1;
		if(chainIndex < 0) chainIndex = 0;
	}    

	if(controller.get_digital_new_press(DIGITAL_R2)){
		backIndex += 1;
		if(backIndex > 2) backIndex = 2;
	}       
	else if(controller.get_digital_new_press(DIGITAL_R1)){
		backIndex -= 1;
		if(backIndex < 0) backIndex = 0;
	}   


    /*
void scooperTurnTo(int target, int speed){
    scooper.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    while(abs(target - scooperPot.get_value()) >= 100){
        scooper.move(speed);
    }
    scooper.move(0);
}

void liftTurnTo(int target, int speed){
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    while(abs(target - liftPot.get_value()) >= 100){
        lift.move(speed);
    }
    lift.move(0);
}

void liftSetLow(void * param){
    liftTurnTo(LIFT_GROUND + 133, -127);
}

void liftSetHigh(void * param){
    liftTurnTo(LIFT_GROUND - 1140, 127);
    while(autonOn){
        lift.move(6);
    }
}

void scooperSetLowLimit(){
    while(autonOn && scoopLimit.get_value() == 0){
        scooper.move(-127);
    }
    scooper.move(0);
}

void scooperDropRing(void * param){
    scooperTurnTo(SCOOPER_GROUND + 570, -127); //+620
}

void scooperSetLow(void * param){
    scooperTurnTo(SCOOPER_GROUND - 160, -127); //+620
}

void scooperDropGoal(void * param){
    scooperTurnTo(SCOOPER_GROUND - 200, -117); //+620
}

void scooperHoldUp(void * param){

    scooperTurnTo(SCOOPER_GROUND + 600, 127);
    while(autonOn){
        scooper.move(12);
    }
}

void scooperReturn(void * param){
    scooperTurnTo(SCOOPER_GROUND + 2000, 127);
}

void scooperReturnSlow(void * param){
    scooperTurnTo(SCOOPER_GROUND + 2000, 100);
}*/