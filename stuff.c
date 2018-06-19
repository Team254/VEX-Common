//TEAM 254 GENERAL USE FUNCTIONS
//@author Andy Herbert
//BNSLibaray not associated or created by Team 254.

#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"
#include "BNSLibrary-master/BNSLib.h"

/************************************************************************************************

INITIALIZATION- startup stuff

*************************************************************************************************/

bool RevLeftOrRight = false;

double time = 0;

int currentAuton = 0;
string autonModes[0]; // = {/*put all your autons here, in name format (used for LCD)*/};

//used to simulate either arcade (UD/LR) or tank (L/Rpow) in auton
int UD = 0, LR = 0, LPow = 127, RPow = 127;

//PID struct- contains most necessary values for PID use
typedef struct{
	float Kp, Ki, Kd, increment; /* constants, need to be set */
	int errorPrev, actualValue, targetValue, error, accl, integral, speed; /* variable values, don't need to be set except actualValue and targetValue */
	int min, max, stationaryHeight, pickup, flat; /* also constants, need to be set. Add more if you need more constants i.e. lift position or speed*/
	bool moving, minMaxReversed;
} PIDObject;

PIDObject liftObjR;

void setValues(){

	//example:
	liftObjR.min = 600;//encoder value
	liftObjR.max = 2615;//encoder value
	liftObjR.Kp = .45;//tuned value
	liftObjR.Ki = 0;//tuned value
	liftObjR.Kd = 2;//tuned value
	liftObjR.actualValue = SensorValue(dgtl1);//encoder value, just get actual sencor value
	liftObjR.targetValue = liftObjR.actualValue;//encoder value, at start should be equal to actualValue to prevent PID from messing up robot functionality
	liftObjR.minMaxReversed = true;//if your encoder min/max are actually reversed because you installed the encoder on wrong, set to true (i.e. up = decreasing value)
	liftObjR.moving = false;//moving?
	//add other values to set initially below

	//set more constants
	nMotorEncoder(in1) = 0;
	nMotorEncoder(in2) = 0;

	UD = 0;
	LR = 0;

	time = 0;

}

/************************************************************************************************

UNIVERSAL FUNCTIONS- general use functions that can show up for anything anywhere in the code

*************************************************************************************************/

//PID function- works with the PID struct from up above
//note: renamed to PIDa because BNSlib also contains a method PID
int PIDa(PIDObject *obj){
	obj->error = obj->actualValue - obj->targetValue;//get error: actual-target
	obj->integral += obj->error;//integral: sum of all errors
	if(abs(obj->error)<= 50) obj->integral = 0;//reset integral if met target value
	float derivative = obj->error - obj->errorPrev;//compare previous error with current error
	obj->errorPrev = obj->error;//update errorPrev
	return (int)((obj->error * obj->Kp) + (obj->Ki * obj->integral) + (obj->Kd * derivative));//PID equation. Note the constants.
}

//updates speed and actual values for PID
task update(){
	while(true){
		liftObjR.actualValue = SensorValue(dgtl1);
	}
}

//put more fuctions here

/************************************************************************************************

SUBASSEMBLY TASKS- control individual parts of robot

************************************************************************************************/

//main drivebase control example
//autoPID- whether or not to use PID during auton or raw sensor outputs
task drivebase(){
	while(true){

		//during autonomous, set drivebase power to output of PID * adjustment for LPow, RPow, which would directly affect max power out (for use in turning slowly, for example. Just like how you would control the robot not @ max speed during driver)
		if(bIfiAutonomousMode){
			motor[port1] = (int)((LPow/127.0)*PIDa(&drivebaseObjL));
			motor[port2] = (int)((RPow/127.0)*PIDa(&drivebaseObjR));
		}
		else{
			if(abs(vexRT[Ch3])+abs(vexRT[Ch1]) <= 20){//deadzone check
				//if in deadzone, use PID
				motor[LDEnc] = PIDa(&drivebaseObjL);
				motor[RDEnc] = PIDa(&drivebaseObjR);
				driver = false;

			}

			//if not in deadzone, drive robot with arcade like you would normally, while updating targetValue, so when you release the joystick, PID activates correctly
			else{
				drivebaseObjL.targetValue = drivebaseObjL.actualValue;
				drivebaseObjR.targetValue = drivebaseObjR.actualValue;

				//motor config (for all below): port 1 front left, port 2 front right, port 3 back left, port 4 back right

				//arcade 4 motor control
				//joy left up/down conrols forward/back, right joy left/right controls turning
				//to swap joysticks, replace Ch3 with Ch2 and Ch1 with Ch4
				motor[port1] = motor[port3] = vexRT[Ch3]+vexRT[Ch1];
				motor[port2] = motor[port4] = vexRT[Ch3]-vexRT[Ch1];

				//tank control
				//joy left up/down controls left drivebase, right joy up/down controls right
				motor[port1] = motor[port3] = vexRT[Ch3];
				motor[port2] = motor[port4] = vexRT[Ch2];

				//holonomic arcade control
				//same control as arcade, except joy left left/right controls strafing
				//attach motors correctly! don't reverse polarity accidentally
				motor[port1] = vexRT[Ch3] - vexRT[Ch4] + vexRT[Ch1];
				motor[port2] = vexRT[Ch3] + vexRT[Ch4] + vexRT[Ch1];
				motor[port3] = -vexRT[Ch3] - vexRT[Ch4] + vexRT[Ch1]
				motor[port2] = -vexRT[Ch3] + vexRT[Ch4] + vexRT[Ch1]
			}
		}
	}
}

//runs all the tasks at the start of driver control and auton
void startTasks(){
	setValues();
	startTask(update);
	startTask(drivebase);
	//do for all other subsystem tasks
}

/* replacement startTasks() for a pre recorded auton plan
void startTasksPrerecorded(){
	setValues();
	startTask(update);
}
*/

//handles LCD screen, for choosing auton mode
task autonOptions(){
	displayLCDPos(0,5);
	displayNextLCDString(autonModes[currentAuton]);
	while(true){
		if(nLCDButtons){

			if(nLCDButtons == 4){
				currentAuton = (currentAuton+1)%((int)sizeof(autonModes)/sizeof(autonModes[0]));
			}
			else if(nLCDButtons == 1){
				currentAuton--;
				if(currentAuton < 0)currentAuton = ((int)sizeof(autonModes)/sizeof(autonModes[0]))-1;
			}
			else if(nLCDButtons == 2){

			}
			clearLCDLine(0);
			displayLCDPos(0, 5);
			displayNextLCDString(autonModes[currentAuton]);
			while(nLCDButtons){}
		}
		displayLCDPos(1,0);
		displayNextLCDNumber(currentAuton);
		displayNextLCDString("      ");
		displayNextLCDNumber(nLCDButtons);
	}
}

/************************************************************************************************

AUTON FUNCTIONS- functions for doing various tasks in auton, like liftToPosition or drive, which can be recalled in your procedure

*************************************************************************************************/

//checks if drivebase has arrived to its proposed position and autoPID is true
bool driveInPos(){
	return (drivebaseObjL.targetValue == drivebaseObjL.actualValue && drivebaseObjR.targetValue == drivebaseObjR.actualValue);

}



void driveToPos(int UDPow, int LRPow,int LTarget, int RTarget){
	LPow = UDPow + LRPow;
	RPow = UDPow - LRPow;
	drivebaseObjL.moving = drivebaseObjR.moving = true;
	if(!RevLeftOrRight){
		drivebaseObjL.targetValue = LTarget;
		drivebaseObjR.targetValue = RTarget;
	}
	else{
		drivebaseObjL.targetValue = RTarget;
		drivebaseObjR.targetValue = LTarget;
	}
	if(drivebaseObjL.actualValue > drivebaseObjL.targetValue){

		if(drivebaseObjR.actualValue > drivebaseObjR.targetValue){
			while(!(drivebaseObjL.actualValue <= drivebaseObjL.targetValue+10 && drivebaseObjR.actualValue <= drivebaseObjR.targetValue+10)){
				motor[LDEnc] = (int)(1.0*PIDa(&drivebaseObjL)*(LPow/127.0));
				motor[RDEnc] = (int)(1.0*PIDa(&drivebaseObjR)*(RPow/127.0));
			}
		}
		else{
			while(!(drivebaseObjL.actualValue <= drivebaseObjL.targetValue+10 && drivebaseObjR.actualValue >= drivebaseObjR.targetValue-10)){
				motor[LDEnc] = (int)(1.0*PIDa(&drivebaseObjL)*(LPow/127.0));
				motor[RDEnc] = (int)(1.0*PIDa(&drivebaseObjR)*(RPow/127.0));
			}
		}
	}
	else{
		if(drivebaseObjR.actualValue > drivebaseObjR.targetValue){
			while(!(drivebaseObjL.actualValue >= drivebaseObjL.targetValue-10 && drivebaseObjR.actualValue <= drivebaseObjR.targetValue+10)){
				motor[LDEnc] = (int)(1.0*PIDa(&drivebaseObjL)*(LPow/127.0));
				motor[RDEnc] = (int)(1.0*PIDa(&drivebaseObjR)*(RPow/127.0));
			}
		}
		else{
			while(!(drivebaseObjL.actualValue >= drivebaseObjL.targetValue-10 && drivebaseObjR.actualValue >= drivebaseObjR.targetValue-10)){
				motor[LDEnc] = (int)(1.0*PIDa(&drivebaseObjL)*(LPow/127.0));
				motor[RDEnc] = (int)(1.0*PIDa(&drivebaseObjR)*(RPow/127.0));
			}
		}
	}
	LPow = RPow = UDPow;
	drivebaseObjL.moving = drivebaseObjR.moving = false;
}

void driveToPos(int power, int LTarget, int RTarget){
	driveToPos(power, 0, LTarget, RTarget);
}

void driveToPos(int LTarget, int RTarget){
	driveToPos(127, 0, LTarget, RTarget);
}

void driveUntil(bool *statement, int UDPow, int LRPow){
	LPow = UDPow + LRPow;
	RPow = UDPow - LRPow;
	drivebaseObjL.moving = drivebaseObjR.moving = true;
	while(!statement){
		drivebaseObjL.targetValue = drivebaseObjL.actualValue+(UDPow*1000);
		drivebaseObjR.targetValue = drivebaseObjR.actualValue+(UDPow*1000);
	}
	drivebaseObjL.targetValue = drivebaseObjL.actualValue;
	drivebaseObjR.targetValue = drivebaseObjR.actualValue;
	LPow = RPow = UDPow;
	drivebaseObjL.moving = drivebaseObjR.moving = false;
}

void driveUntil(bool *statement){
	driveUntil(statement, 127, 0);
}

void driveUntil(bool *statement, int UDPow){
	driveUntil(statement, UDPow, 0);
}

bool liftMoving(){
	return liftObjL.moving;
}
//example: liftToPos
//pos is whatever position you want your lift to be
//note that there should be a task that handles (runs PID on) the lift or whatever you want to control
/*
void liftToPos(int pos){
	liftObjL.moving = liftObjR.moving = true;
	liftObjL.targetValue = liftObjR.targetValue = pos;
	waitUntil(liftNearLevel());
	liftObjL.moving = liftObjR.moving = false;
}
*/

void liftCToPos(int pos){
	//CLiftObj.moving = true;
	CLiftObj.targetValue = pos;
	waitUntil(CLiftNearLevel());
	//CLiftObj.moving = false;
}

bool MLiftUp = true;

void liftM(){
	MLiftObj.targetValue = (!MLiftUp)?MLiftObj.min:MLiftObj.max;
	MLiftUp = !MLiftUp;
}


/************************************************************************************************

AUTON RECORDING

*************************************************************************************************/

typedef struct autonSteps{
	int length;
	DynamicArray power, time;
	int step, motorTarget;
} autonSteps_t;

//autonSteps_t DL, DR, LL, LR, ML, CL, CI;
autonSteps_t autoSteps[7];

void defineValues(){
	autoSteps[0].motorTarget = LDEnc;
	autoSteps[1].motorTarget = RDEnc;
	autoSteps[2].motorTarget = liftL;
	autoSteps[3].motorTarget = liftR;
	autoSteps[4].motorTarget = MLift;
	autoSteps[5].motorTarget = CLift;
	autoSteps[6].motorTarget = CIntake;

	for(int i = 0; i < 7; i++){
		autoSteps[i].step = 0;
		DynamicArrayInit(&autoSteps[i].power);
		DynamicArrayInit(&autoSteps[i].time);
	}
}
void record(int ind, int power, int time){
	DynamicArrayAdd(&autoSteps[ind].power, power);
	DynamicArrayAdd(&autoSteps[ind].time, time);
}

void run(){
	int currentStep = 0;

		for(int i = 0; i < 7; i++){
			while(bIfiAutonomousMode && currentStep < DynamicArrayAllocatedSize(&autoSteps[i].power)){
			int currentMotorTarget = autoSteps[i].motorTarget;
			int currentPower = DynamicArrayGet(&autoSteps[i].power, currentStep);
			//int currentTime = DynamicArrayGet(&autoSteps[i].time, currentStep);
			//if(currentTime == time){
			motor[currentMotorTarget] = currentPower;
			wait1Msec(1);
			time+=.001;//}
		}
		currentStep++;
	}
}

/************************************************************************************************

AUTON PROCEDURES

*************************************************************************************************/

void initAuto(){
	motor[CIntake] = 50;
	motor[CLiftm] = -127;
	CLiftManual = true;
	liftToPos(1600);//high enough to lower MLift and CLift
	liftM();
	motor[CLiftm] = 127;
	CLiftManual = false;
	motor[CIntake] = 20;
	liftCToPos(CLiftObj.max);
}

//ISSUE: robot won't drive forward
void driveToMobileGoal(){
	initAuto();
	//liftToPos(1200);
	//driveToPos(100, -100);
	//driveToPos(60, 500, 500);
	driveToPos(1380, 1380);
	liftM();
	wait1Msec(2000);
	liftToPos(liftObjL.min);
	motor[CIntake] = -127;
	wait1Msec(500);
	motor[CIntake] = 20;
	liftToPos(1200);//enough for 1 cone height- getting cone directly in front of robot
	liftCToPos(CLiftObj.min);
	driveToPos(1675, 1675);
	motor[CIntake] = 127;
	liftToPos(liftObjL.min);
	motor[CIntake] = 20;
	liftCToPos(CLiftObj.max);
	motor[CIntake] = -127;
	wait1Msec(1000);
	motor[CIntake] = 20;
	driveToPos(1905,1905);
	//liftToPos(1000);//enough for 2 cone height- getting cone directly in front of robot
	motor[CIntake] = 127;
	liftToPos(liftObjL.min);
	motor[CIntake] = 20;
	liftToPos(800);
	liftCToPos(CLiftObj.max);
	motor[CIntake] = -127;
	wait1Msec(1000);
	motor[CIntake] = 20;

	driveToPos(450, 450);
	/*driveToPos(0, 3000);
	driveToPos(1500, 4500);
	//ends right in front of 5-point zone
	*/
}

void twentyAuto(){
	driveToMobileGoal();
	driveToPos(2000, 4000);
	driveToPos(2500, 4500);
	driveToPos(2000, 5000);
	driveToPos(2500, 5500);
	liftM();
	driveToPos(2000, 5000);
}

void defenceAuto(){
	driveToPos(-80, 10);
	driveToPos(2020, 2210);
	/*
	driveToPos(1920, 2230);
	driveToPos(2160, 2420);
	driveToPos(1930, 2350);
	//initAuto();
	*/
}

void tenAuto(){
	driveToMobileGoal();
	driveToPos(2000, 5000);
	liftM();
	driveToPos(1500, 4500);
}

void fiveAuto(){
	driveToMobileGoal();
	liftM();
	driveToPos(1000, 4000);
}

void altAuto(){
	initAuto();
	liftToPos(liftObjL.max);
	driveToPos(500, 500);
	liftToPos(liftObjL.max-100);
	motor[CIntake] = -127;
	liftToPos(liftObjL.max);
	driveToPos(250, 250);
	driveToPos(500, 0);
}

void autoPlay(){
	run();
}

/************************************************************************************************

REQUIRED FUNCTIONS

*************************************************************************************************/
void pre_auton()
{
	BNS();
	setValues();
	bStopTasksBetweenModes = true;

	bDisplayCompetitionStatusOnLcd = false;

	bLCDBacklight = true;
	startTask(autonOptions);
}


task autonomous()
{
	openOrClose = true;
	if(currentAuton != 11) startTasks();
	else startTasksPrerecorded();
	if(SensorValue[powEx] < 300){
		displayLCDPos(0,0);
		displayNextLCDString("REPLACE/PLUG IN");
		displayLCDPos(1,0);
		displayNextLCDString("POWER EXPANDER");
	}
	else if(nImmediateBatteryLevel<300){
		displayLCDPos(0,0);
		displayNextLCDString("REPLACE");
		displayLCDPos(1,0);
		displayNextLCDString("MAIN BATTERY");
	}
	else{
		switch (currentAuton){
		case 1://20L
			RevLeftOrRight = true;
		case 2://20R
			twentyAuto();
			break;
		case 3://10L
			RevLeftOrRight = true;
		case 4://10R
			tenAuto();
			break;
		case 5://5L
			RevLeftOrRight = true;
		case 6://5R
			fiveAuto();
			break;
		case 7://altL
			RevLeftOrRight = true;
		case 8://altR
			altAuto();
			break;
		case 9://defenceL
			RevLeftOrRight = true;
		case 10://defenceR
			defenceAuto();
			break;
		case 11://recordedPlay
			run();
			break;
		case 12://record
			break;

		}
	}
}

bool autoRecord = false;

task usercontrol()
{
	BNS();
	startTasks();
	stopTask(autonOptions);
	datalogClear();

	string mainBattery, backupBattery;
	while (true)
	{
		clearLCDLine(0);
		clearLCDLine(1);

		//Display the Primary Robot battery voltage
		displayLCDString(0, 0, "Primary: ");
		sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0,'V');
		displayNextLCDString(mainBattery);

		//Display the Backup battery voltage
		displayLCDString(1, 0, "Secondary: ");
		sprintf(backupBattery, "%1.2f%c", SensorValue[powEx]/1000.0, 'V');
		displayNextLCDString(backupBattery);
		if(currentAuton == 12){
			autoRecord = true;
			defineValues();
			//datalogDataGroupStart();
			for(int i = 0; i < 7; i++){
				DynamicArrayClear(&autoSteps[i].power);
				DynamicArrayClear(&autoSteps[i].time);

				int loggedData = currentMotors[i];
				datalogAddValueWithTimeStamp(i, currentMotors[i]);
				record(i, currentMotors[i], time);
				prevMotors[i] = loggedData;
				wait1Msec(1);
				time+=.001;
			}
			currentAuton = 11;
		}
		else if(currentAuton == 11){
			//while(time <= 15000){
			for(int i = 0; i < 7; i++){
				//currentMotors = {motor[LDEnc], motor[RDEnc], motor[liftL], motor[liftR], motor[MLiftL], motor[CLift], motor[CIntake]};
				//if((currentMotors[i]/10)*10!=(prevMotors[i]/10)*10){
				datalogAddValueWithTimeStamp(i, currentMotors[i]);
				record(i, currentMotors[i], time);
				prevMotors[i] = currentMotors[i];
				//}
				wait1Msec(1);
				time+=.001;
			}
			//wait1Msec(1);
			//}
			//datalogDataGroupEnd();

		}

	}
}
