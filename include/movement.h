#include "main.h"
#include "robot.h"
#include "PID.h"
#ifndef MOVEMENT_H_
#define MOVEMENT_H_

//extern bool autonOn;

void chas_move(int left_power, int right_power);
/*void scooperTurnTo(int target, int speed);
void scooperDropRing(void * param);
void scooperSetLow(void * param);
void scooperHoldUp(void * param);
void scooperSetLowLimit();
void scooperReturn(void * param);
void scooperReturnSlow(void * param);
void liftTurnTo(int target, int speed);
void liftSetLow(void * param);
void liftSetHigh(void * param);
void scooperDropGoal(void * param);
void move(int target, bool ask_slew = false, int slew_rate = 0, int power_cap = 127, int active_cap = 0, bool cP = false);
void turn(int target, bool ask_slew = false, int slew_rate = 0);
void absturn(int abstarget, bool ask_slew = false, int slew_rate = 0, int power_cap = 127);
void reset_encoders();

//void turn(int target, bool ask_slew, int slew_rate);*/
#endif
