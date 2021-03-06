#pragma config(Motor,  port2,           driveL,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port3,           intake,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           shooter1,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port5,           shooter2,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           shooter3,      tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port8,           shooter4,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           driveR,        tmotorVex393_MC29, openLoop, reversed)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VEX EDR                      */
/*                                                                           */
/*---------------------------------------------------------------------------*/

// This code is for the VEX cortex platform
#pragma platform(VEX2)

// Select Download method as "competition"
#pragma competitionControl(Competition)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton()
{
	// Set bStopTasksBetweenModes to false if you want to keep user created tasks
	// running between Autonomous and Driver controlled modes. You will need to
	// manage all user created tasks if set to false.
	bStopTasksBetweenModes = true;

	// Set bDisplayCompetitionStatusOnLcd to false if you don't want the LCD
	// used by the competition include file, for example, you might want
	// to display your team name on the LCD in this function.
	// bDisplayCompetitionStatusOnLcd = false;

	// All activities that occur before the competition starts
	// Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task autonomous()
{
	// ..........................................................................
	// Insert user code here.
	// ..........................................................................

	// Remove this function call once you have "real" code.
	AutonomousCodePlaceholderForTesting();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

task usercontrol()
{
	// User control code here, inside the loop

	while (true)
	{
		motor[driveLeft] = vexRT[Ch3] + vexRT[Ch1];
		motor[driveRight] = vexRT[Ch3] - vexRT[Ch1];

		if(vexRT[Btn6U]){
			motor[intake] = 127;
		}
		else if (vexRT[Btn6D]){
			motor[intake] = -127;
		}

		else{
			motor[intake] = 0;
		}

		if(vexRT[Btn5U]){
			motor[shooter1] = motor[shooter2] = motor[shooter3] = motor[shooter4] = 127;
		}
		else if (vexRT[Btn5D]){
			motor[shooter1] = motor[shooter2] = motor[shooter3] = motor[shooter4] = -127;
		}

		else{
			motor[shooter1] = motor[shooter2] = motor[shooter3] = motor[shooter4] = 0;
		}
	}
}
