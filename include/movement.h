#include "main.h"
#include "robot.h"
#include "PID.h"
#ifndef MOVEMENT_H_
#define MOVEMENT_H_

//extern bool autonOn;

void chas_move(int left_power, int right_power);
void move(int target, bool ask_slew = false, int slew_rate = 0, int power_cap = 127, int active_cap = 0, bool cP = true);
void goalYoink(int far = 199);
void postGoalReset();
void turn(int target, bool ask_slew = false, int slew_rate = 0);
void absturn(int abstarget, bool goal, bool ask_slew = false, int slew_rate = 0, int power_cap = 127);
void absturnTimed(int abstarget, bool goal, int timer_amt = 80, bool ask_slew = false, int slew_rate = 0, int power_cap = 127);

void chainClawOpen();
void chainClawClose();
void backClawOpen();
void backClawClose();
void jsClawOpen();
void jsClawClose();
void stickDown();
void stickUp();

extern float start_heading;
extern bool openChain;
extern bool openJS;
extern bool openBack;
extern bool openStick;

void moveTillChain(int speed, int timer = 300);
void moveTillBack(int speed, int timer = 300);

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
void scooperDropGoal(void * param);*/

#endif
