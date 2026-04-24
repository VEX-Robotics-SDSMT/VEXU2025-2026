#include "main.h"
#include "DiffDrive.h"
#include "botFunctions.h"
#include "globals.h"
#include "Logger.h"
#include "pros/rtos.h"
#include "vexGPS.h"

// globals

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

using namespace Mines;

void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(2, "I was pressed!");
	}
	else
	{
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize()
{
	intertialSensor.reset();
	// intakeDropR.set_value(0);
	// intakeDropL.set_value(0);
	// pros::vision_signature_s_t RED_GOAL_SIG = vision.signature_from_utility(1, 4391, 7505, 5948, -1303, -147, -725, 1.6, 0);
	// vision.set_signature(RED_GOAL_SIG_ID, &RED_GOAL_SIG);
	// pros::vision_signature_s_t BLUE_GOAL_SIG = vision.signature_from_utility(2, -3073, -1323, -2198, 4405, 9923, 7164, 1.5, 0);
	// vision.set_signature(BLUE_GOAL_SIG_ID, &BLUE_GOAL_SIG);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled()
{
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
void competition_initialize()
{
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
void autonomous()
{
	EncoderWheelSensorInterface encoderInterface(driveEncoder);
	DiffDrive drive(leftDriveMotors, rightDriveMotors, &encoderInterface, intertialSensor);
	drive.setDrivePIDVals(0.3, 0, 0); //0.3, 0, 0 for Luigi
	drive.setDrivePIDTol(50);
	drive.setTurnPIDVals(5, 0, 20); //5, 0, 20 tuned 4/20/2026
	drive.setTurnPIDTol(2);
	drive.setMaxDriveSpeed(0.6);
	drive.setMaxTurnSpeed(0.6);
	drive.setMaxDriveAccel(0.12);

	//drive toward loader
	drive.driveTiles(1050);
	drive.turnDegreesAbsolute(-90);

	//start getting blocks
	intakeSlow();
	drive.driveTiles(-400, 1000);
	//modify here to get blocks from loader
	pros::delay(2000);

	//back away from goal and score on long
	drive.setMaxDriveSpeed(0.2);
	drive.driveTiles(500);
	intakeLiftL.set_value(1);
	intakeLiftR.set_value(1);
	drive.turnDegreesAbsolute(-85);
	drive.driveTiles(800, 1000);
	scoreTop();
	//modify delay to score blocks
	pros::delay(3000);
	intakeBrake();

	//drive back to loader
	drive.driveTiles(-1000);
	intakeLiftL.set_value(0);
	intakeLiftR.set_value(0);
	intakeSlow();
	drive.driveTiles(-300, 1000);

	//modify here to get blocks
	pros::delay(2000);
	//drive toward center low goal
	intakeBrake();
	intakeWheels.move(127);
	drive.driveTiles(500);
	drive.turnDegreesAbsolute(50);
	drive.driveTiles(-1500);
	drive.turnDegreesAbsolute(42, 800);
	outTake();
	drive.driveTiles(-75, 200);
	pros::delay(2000);

	//back away for match begin
	intakeBrake();
	drive.driveTiles(1000);



	drive.killPIDs();

	
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

void opcontrol()
{
	bool togLift = false;
	bool togWing = true;
	wing.set_value(1);
	while (true)
	{
		// ********************DRIVE********************
		// 2 stick arcade
		// double leftAxisY = MasterController.get_analog(axisLeftY);
		// double rightAxisX = MasterController.get_analog(axisRightX);
		// double leftVelocity = ((leftAxisY + rightAxisX));
		// double rightVelocity = ((leftAxisY - rightAxisX));

		// 1 stick arcade

		// Left stick (normal arcade drive)
		double leftAxisY = MasterController.get_analog(axisLeftY);
		double leftAxisX = -1 * MasterController.get_analog(axisLeftX);

		// Right stick (same arcade drive, but Y is flipped)
		double rightAxisY = -1 * MasterController.get_analog(axisRightY);
		double rightAxisX = -1 * MasterController.get_analog(axisRightX);

		// Left stick velocities
		double leftVelocityLeftStick = 4 * (leftAxisY + leftAxisX);
		double rightVelocityLeftStick = 4 * (leftAxisY - leftAxisX);

		// Right stick velocities
		double leftVelocityRightStick = 4 * (rightAxisY + rightAxisX);
		double rightVelocityRightStick = 4 * (rightAxisY - rightAxisX);

		double leftVelocity = leftVelocityLeftStick + leftVelocityRightStick;
		double rightVelocity = rightVelocityLeftStick + rightVelocityRightStick;

		// Tank
		// double leftAxisY = MasterController.get_analog(axisLeftY);
		// double rightAxisY = MasterController.get_analog(axisRightY);
		// double leftVelocity = ((leftAxisY) * axisPercentBlue);
		// double rightVelocity = ((-rightAxisY) * axisPercentBlue);

		// INTAKE MOTORS
		// Score HIGH
		if (MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			scoreTop();
		}
		// Intake but not score
		else if (MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			intakeSlow();
		}
		// Outtake
		else if (MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_R1) || MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			outTake();
		}
		else
		{
			intakeBrake();
		}

		// INTAKE LIFT
		if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
		{
			if (togLift == 1)
				togLift = 0;
			else
				togLift = 1;
			intakeLiftR.set_value(togLift);
			intakeLiftL.set_value(togLift);
		}

		// WING
		if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
		{
			if (togWing == 1)
				togWing = 0;
			else
				togWing = 1;
			wing.set_value(togWing);
		}
		driveLoop(leftDriveMotors, rightDriveMotors, leftVelocity, rightVelocity);

		//*********************************************
	}
}
