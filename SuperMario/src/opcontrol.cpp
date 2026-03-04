#include "main.h"


void tankDrive(pros::v5::Controller& controller, const pros::v5::AbstractMotor& left, const pros::v5::AbstractMotor& right)
{
	left.move(controller.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_Y));
	right.move(controller.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_RIGHT_Y));
}

void leftArcadeDrive(pros::v5::Controller& controller, const pros::v5::AbstractMotor& left, const pros::v5::AbstractMotor& right)
{
	left.move(controller.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_Y) + controller.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_X ));
	right.move(controller.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_Y) - controller.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_X ));
}

void rightArcadeDrive(pros::v5::Controller& controller, const pros::v5::AbstractMotor& left, const pros::v5::AbstractMotor& right)
{
	left.move(controller.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_RIGHT_Y) + controller.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_RIGHT_X ));
	right.move(controller.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_RIGHT_Y) - controller.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_RIGHT_X ));
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

void opcontrol() {


	int target;
	
	Mines::Drivetrain drive(&leftMotors, &rightMotors, &enc, &imu);
	drive.capVoltage(127);


	drive.setDrivePID(.0009, 1, 0.0, 3);
	drive.setTurnPID(1.8, 0.6, 0.0, 1.0);
	drive.driveDistance(48, 5000, 130);
	drive.turnTo(86, 1.5, 130);
	drive.driveDistance(48, 5000, 190);
	drive.turnTo(83, 1.5, 190);
	drive.driveDistance(48, 5000);
	rightMotors.move(0);
	leftMotors.move(0);

	/**
	 while(true)
	 {
		tankDrive(master,leftMotors,rightMotors);
		pros::delay(20);
	 }
	 */
	pros::delay(100000);

}