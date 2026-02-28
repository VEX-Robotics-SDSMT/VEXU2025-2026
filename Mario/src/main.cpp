#include "main.h"
#include "DiffDrive.h"
#include "botFunctions.h"
#include "globals.h"
#include "pros/rtos.h"
#include "vexGPS.h"


using namespace Mines;


void initialize()
{
	intertialSensor.reset();
}

void disabled()
{
}

void competition_initialize()
{
}

void autonomous()
{
	EncoderWheelSensorInterface encoderInterface(driveEncoder);
	DiffDrive drive(leftDriveMotors, rightDriveMotors, &encoderInterface, intertialSensor);
	LoggerSettings(verbose);
	drive.setDrivePIDVals(0.18, 0, 0); // tuned 1/11/26
	drive.setDrivePIDTol(50);
	drive.setTurnPIDVals(2.8, 0, 0); // tuned 1/11/26
	drive.setTurnPIDTol(2);
	drive.setMaxDriveSpeed(0.4);
	drive.setMaxTurnSpeed(0.7);
	drive.setMaxDriveAccel(0.12);

	// for some reason forward is tuned to right distance but when backing up it actually goes further
	int default_timeout = 1000;
	// drive toward center goal
	if (!lower)
	{
		drive.driveTiles(1100, default_timeout+1000);
		drive.turnDegreesAbsolute(36, default_timeout);
		drive.driveTiles(50, default_timeout);
		intakeScoreTop(true);
		pros::delay(800);
		intakeBrake();
		drive.driveTiles(-550, default_timeout);
		drive.turnDegreesAbsolute(50, default_timeout);
		drive.driveTiles(-800, default_timeout);
		drive.turnDegreesAbsolute(185, default_timeout);
	}
	else {
		drive.driveTiles(1100, default_timeout + 1000);
		drive.turnDegreesAbsolute(36, default_timeout);
		drive.driveTiles(300, default_timeout);
		intakeOut();
		pros::delay(1000);
		drive.driveTiles(-550, default_timeout);
		drive.turnDegreesAbsolute(50, default_timeout);
		drive.driveTiles(-1100, default_timeout);
		drive.turnDegreesAbsolute(185, default_timeout);
	}
	pros::delay(500);
	//empty loader
	Arm.set_value(1);
	Arm2.set_value(1);
	pros::delay(500);
	intakeBasket();
	for(int i = 0; i < 6; i++) {
		drive.driveTiles(1000, default_timeout);
		drive.driveTiles(-100, 200);
		if(i == 2) {
			drive.driveTiles(-400, default_timeout);
			intakeOut();
			pros::delay(500);
			intakeBasket();
			drive.driveTiles(400, default_timeout);
			drive.turnDegreesAbsolute(180, default_timeout);
			
		}
	}

	drive.driveTiles(-400, default_timeout);
	drive.turnDegreesAbsolute(90, default_timeout + 500);
	intakeOut();
	intakeBasket();
	pros::delay(500);
	Arm.set_value(0);
	Arm2.set_value(0);
	drive.turnDegreesAbsolute(5, default_timeout);
	
	//go up to long goal
	drive.driveTiles(620, default_timeout + 500);
	intakeScoreTop();
	pros::delay(4000);
	drive.driveTiles(-500);
	intakeBrake();



	drive.killPIDs();
}


/**
 * Runs the text based controller selector to setup robot configuration
 */
void runSelector()
{
	MasterController.clear();
	MasterController.print(0, 0, "Running Selector...\n");

	while(true)
	{
		
		pros::delay(20);
	}
}

void opcontrol()
{
	intakeBrake();

	if (red_team)
	{
		COLOR_MAX = BLUE_MAX;
		COLOR_MIN = BLUE_MIN;
		MasterController.clear_line(0);
		MasterController.print(0, 0, "Filter BLUE : MAX: %d", COLOR_MAX);
	}
	else
	{
		COLOR_MAX = RED_MAX;
		COLOR_MIN = RED_MIN;
		MasterController.clear_line(0);
		MasterController.print(0, 0, "%s"
									 "FILTER RED : MAX: %d",
							   COLOR_MAX);
	}

	bool togArm = 0, togColor = 0;
	colorSensor.set_led_pwm(100);

	//runSelector();

	while (true)
	{
		// ********************DRIVE********************
		// 2 stick arcade
		double leftAxisY = MasterController.get_analog(axisLeftY);
		double rightAxisX = MasterController.get_analog(axisRightX);
		double leftVelocity = ((leftAxisY + rightAxisX));
		double rightVelocity = ((leftAxisY - rightAxisX));

		// 1 stick arcade
		// double leftAxisY = MasterController.get_analog(axisLeftY);
		// double leftAxisX = MasterController.get_analog(axisLeftX);
		// double rightAxisX = MasterController.get_analog(axisRightX);
		// double aimVelocityLeft = (rightAxisX) * 0.06;
		// double aimVelocityRight = -rightAxisX * 0.06;
		// double leftVelocity = ((leftAxisY + leftAxisX + aimVelocityLeft));
		// double rightVelocity = ((leftAxisY - leftAxisX + aimVelocityRight));

		// Tank
		// double leftAxisY = MasterController.get_analog(axisLeftY);
		// double rightAxisY = MasterController.get_analog(axisRightY);
		// double leftVelocity = ((leftAxisY) * axisPercentBlue);
		// double rightVelocity = ((rightAxisY) * axisPercentBlue);

		// INTAKE MOTORS
		// Score HIGH
		if (MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			intakeScoreTop();
		}
		// Outtake, score mid won't work rn
		else if (MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			intakeScoreMid();
		}
		// Basket
		else if (MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			//check if motor is stalling
			if(IntakeFront.get_torque() >= .89)
			{
				IntakeFront.move(127);
			}
			else
			{
				intakeBasket();
			}
		}
		// Outtake
		else if (MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			intakeOut();
		}
		else
		{
			intakeBrake();
		}

		// ARM
		if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
		{
			togArm = !togArm;
			if (togArm)
			{
				Arm.set_value(1);
				Arm2.set_value(1);
			}
			else
			{
				Arm.set_value(0);
				Arm2.set_value(0);
			}
		}

		// Toggle Color changing
		if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
		{
			togColor = !togColor;
			// We are blue filter out blue or red filter blue
			if ((togColor && !red_team) || (!togColor && red_team))
			{
				COLOR_MAX = BLUE_MAX;
				COLOR_MIN = BLUE_MIN;
				MasterController.clear_line(0);
				MasterController.print(0, 0, "Filter BLUE : MAX: %d", COLOR_MAX);
			}
			// We are red filtering out red or blue filtering red
			else
			{
				COLOR_MAX = RED_MAX;
				COLOR_MIN = RED_MIN;
				MasterController.clear_line(0);
				MasterController.print(0, 0, "%s"
											 "FILTER RED : MAX: %d",
									   COLOR_MAX);
			}
		}

		driveLoop(leftDriveMotors, rightDriveMotors, leftVelocity, rightVelocity);

		//*********************************************
	}
}
