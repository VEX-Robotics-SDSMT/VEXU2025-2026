#include "main.h"

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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor motor(1,pros::v5::MotorGears::green,pros::v5::MotorEncoderUnits::degrees);

	Mines::PID pid(5, .001, .5, .5);
	target = 5100;
	pid.setTarget(target);

	while(true)
	{
		motor.move(pid.calculate(motor.get_position()));
		pros::delay(10);
	}

}