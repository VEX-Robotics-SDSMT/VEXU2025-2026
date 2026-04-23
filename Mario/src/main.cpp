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
	EncoderWheelSensorInterface encoderInterface(rotationSensor);
	DiffDrive drive(leftDriveMotors, rightDriveMotors, &encoderInterface, intertialSensor);
	LoggerSettings(verbose);
	drive.setDrivePIDVals(0.18, 0, 0); // tuned 1/11/26
	drive.setDrivePIDTol(50);
	drive.setTurnPIDVals(2.8, 0, 0); // tuned 1/11/26
	drive.setTurnPIDTol(2);
	drive.setMaxDriveSpeed(0.4);
	drive.setMaxTurnSpeed(0.7);
	drive.setMaxDriveAccel(0.12);

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
	ArmMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	bool togLift = 0, togFlap = 0, togArm = 0;

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
		if (MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			intakeScoreTop();
		}
		else if (MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			intakeOut();
		}
		else
		{
			intakeBrake();
		}

		// ARM
		if (MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			ArmMotor.move(-127);
		}
		else if(MasterController.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			ArmMotor.move(127);
		}
		else {
			ArmMotor.brake();
		}

		if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
		{
			togLift = !togLift;
			if(togLift) {
				lift.set_value(1);
				lift1.set_value(1);
				flap.set_value(1);
			}
			else {
				lift.set_value(0);
				lift1.set_value(0);
				flap.set_value(0);
			}
			
		}

		if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
		{
			if(togFlap) {
				flap.set_value(1);
			}
			else {
				flap.set_value(0);
			}
			togFlap = !togFlap;
		}

		if (MasterController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
		{
			if(togArm) {
				Arm.set_value(1);
			}
			else {
				Arm.set_value(0);
			}
			togArm = !togArm;
		}

		driveLoop(leftDriveMotors, rightDriveMotors, leftVelocity, rightVelocity);

		//*********************************************
	}
}
