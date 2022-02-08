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
*/