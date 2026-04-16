#include "main.h"

int32_t mapExponential(int32_t input)
{
    return (input * input * input)/17000;
}


void tankDrive(pros::v5::Controller& controller, const pros::v5::AbstractMotor& left, const pros::v5::AbstractMotor& right)
{
	left.move(controller.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_Y));
	right.move(controller.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_RIGHT_Y));
}

void leftArcadeDrive(pros::v5::Controller& controller, const pros::v5::AbstractMotor& left, const pros::v5::AbstractMotor& right, pros::controller_analog_e_t axis1, pros::controller_analog_e_t axis2)
{
	int32_t drivePower = mapExponential(controller.get_analog(axis1));
	int32_t turnPower = std::clamp(mapExponential(controller.get_analog(axis2)), (int32_t)-70, (int32_t)70);
	left.move(drivePower +  turnPower);
	right.move(drivePower -  turnPower);
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
	 while(true)
	 {
		leftArcadeDrive(master,leftMotors,rightMotors, pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_Y, 
			pros::controller_analog_e_t::E_CONTROLLER_ANALOG_RIGHT_X);
		
		
		pros::delay(20);
	 }

}