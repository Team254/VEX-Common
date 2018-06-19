//TEAM 254 GENERAL USE FUNCTIONS
//@author Andy Herbert
//BNSLibaray not associated or created by Team 254.

#pragma platform(VEX2)
#pragma competitionControl(Competition)
#include "Vex_Competition_Includes.c"
#include "BNSLibrary-master/BNSLib.h" //library with PID stuff, ArrayLists + more

/************************************************************************************************

INITIALIZATION- startup stuff

*************************************************************************************************/

bool RevLeftOrRight = false;

double time = 0;

//used mostly with LCD screen
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

PIDObject liftObjR, drivebaseObjL, drivebaseObjR;

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
		//note: code in if-true statement for 2 motor drivebase
		if(bIfiAutonomousMode){
			motor[port1] = (int)((LPow/127.0)*PIDa(&drivebaseObjL));
			motor[port2] = (int)((RPow/127.0)*PIDa(&drivebaseObjR));
		}
		else{
			if(abs(vexRT[Ch3])+abs(vexRT[Ch1]) <= 20){//deadzone check
				//if in deadzone, use PID
				motor[port1] = PIDa(&drivebaseObjL);
				motor[port2] = PIDa(&drivebaseObjR);

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
	displayNextLCDString(autonModes[currentAuton]);//not sure about these errors below, it was working the last time I uploaded to my robot...
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

//example use: liftCToPos
//lifts cone lift from ITZ to a certain position w/ PID active
void liftCToPos(int pos){
	liftObjR.targetValue = pos;//PID function running during auton
	waitUntil(CLiftNearLevel());//important unless you want to multitask
}

/************************************************************************************************

AUTON RECORDING- easy and cheap to get an auton. NOTE: RobotC has no way to save files- you cannot restart the robot or probably even turn off the controller or your new auton procedure will be erased! It is also an on-the-fly type of thing, good for skills

*************************************************************************************************/
//note: this code is still in beta and probably doesn't work entirely as intended. Because it's for advanced users only, there will be minimal comments/editing.
//You have been warned.

//like struct for PID system, one for each subsystem you want to record
typedef struct autonSteps{
	int length;
	DynamicArray power, time;
	int step, motorTarget;
} autonSteps_t;

//autonSteps_t DL, DR, LL, LR, ML, CL, CI;
autonSteps_t autoSteps[7];

void defineValues(){
	//point each autonSteps struct to its corresponding motor to record- recording auton takes in motor power and time rather than sensor values and PID
	autoSteps[0].motorTarget = port1;
	autoSteps[1].motorTarget = port2;
	autoSteps[2].motorTarget = port3;
	autoSteps[3].motorTarget = port4;
	autoSteps[4].motorTarget = port5;
	autoSteps[5].motorTarget = port6;
	autoSteps[6].motorTarget = port7;

	for(int i = 0; i < 7; i++){
		autoSteps[i].step = 0;
		DynamicArrayInit(&autoSteps[i].power);
		DynamicArrayInit(&autoSteps[i].time);
	}
}

//BNSlib use- easily add to dynamic arrays
//note: VEX cortex has limited RAM for this. It has been optimized to not use a lot of RAM, but long/complex auton procedures might overflow the memory.
//Check the debug log in the debugger to see if BNSlib is complaining about memory limits.
void record(int ind, int power, int time){
	DynamicArrayAdd(&autoSteps[ind].power, power);
	DynamicArrayAdd(&autoSteps[ind].time, time);
}

//runs only during auton
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

AUTON PROCEDURES- define different types of autons here. Good for your robot so you can adjust w/ your alliance when needed.

*************************************************************************************************/

void initAuto(){//common code that might need to be run at the start of auton, like if you need to lower a lift before anything. Can be optional, see below
}

//example
void driveToMobileGoal(){
	initAuto();//you put this depending on if you need the initAuto function to run

	/* NOTE: commented out because not all functions shown in code. Pretty much it's a hardcoded procedure moving the robot around the field and other stuff accurately with PID and auton functions from the above section


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

//place other autons below...
void twentyAuto(){
	//code here
}

void defenceAuto(){
	//code here
}

//for recording auton, could be simplified if you don't like one line functions
void autoPlay(){
	run();
}

//more autons below...

/************************************************************************************************

REQUIRED FUNCTIONS- the base of your code

*************************************************************************************************/
void pre_auton()
{
	BNS();//loads BNS
	setValues();//see above

	//see VEX documentation for this variable
	bStopTasksBetweenModes = true;

	//false for using the LCD
	bDisplayCompetitionStatusOnLcd = false;

	//optional, could save battery?
	bLCDBacklight = true;

	//see above. Mainly used with LCD
	startTask(autonOptions);
}

//doesn't actually run any auton procedure- it calls the functions from the above section
task autonomous()
{

	startTasks();//pretty much just call this at the start of auton/driver

	if(SensorValue[dgtl1] < 300){//check if power expander is plugged in/charged
		displayLCDPos(0,0);
		displayNextLCDString("REPLACE/PLUG IN");
		displayLCDPos(1,0);
		displayNextLCDString("POWER EXPANDER");
	}

	else if(nImmediateBatteryLevel<300){//check if main battery is charged enough. Note: may need to adjust the 300 value
		displayLCDPos(0,0);
		displayNextLCDString("REPLACE");
		displayLCDPos(1,0);
		displayNextLCDString("MAIN BATTERY");
	}

	//if all is good, start auton from whatever index currentAuton is above (which is changed in autonOptions)
	else{
		switch (currentAuton){
		case 1://first auton in autonModes[]
			RevLeftOrRight = true;//flip left/right drivebase values, for mirroring left/right
		case 2://second auton, which is just the first but on other side of field
			driveToMobileGoal();
			break;
		case 3://10L
			//...
		}
	}
}

bool autoRecord = false;//if autoRecord was set on LCD for auto recording an auton

task usercontrol()
{
	BNS();
	startTasks();
	stopTask(autonOptions);//LCD is used in user control to display battery levels

	string mainBattery, backupBattery;

	while (true)//important!!!
	{
		clearLCDLine(0);
		clearLCDLine(1);

		//Display the Primary Robot battery voltage
		displayLCDString(0, 0, "Primary: ");
		sprintf(mainBattery, "%1.2f%c", nImmediateBatteryLevel/1000.0,'V');
		displayNextLCDString(mainBattery);

		//Display the Backup battery voltage
		displayLCDString(1, 0, "Secondary: ");
		sprintf(backupBattery, "%1.2f%c", SensorValue[dgtl1]/1000.0, 'V');
		displayNextLCDString(backupBattery);

		/* auton recording while driving commented out below- can be moved into its own function

		//initially set all values as needed
		//note: data log used for help/debug while recording

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

		//continuously add values until out of memory, will continuously add to datalog
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
	*/
	}
}
