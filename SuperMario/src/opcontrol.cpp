#include "main.h"

int32_t mapExponentialDrive(int32_t input)
{
    return (input * input * input)/17000;
}

int32_t mapExponentialTurn(int32_t input)
{
	if (input < 0)
    	return -(input * input * input * input)/5000000;
	else
		return (input * input * input * input)/5000000;
}


void tankDrive(pros::v5::Controller& controller, const pros::v5::AbstractMotor& left, const pros::v5::AbstractMotor& right)
{
	left.move(controller.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_Y));
	right.move(controller.get_analog(pros::controller_analog_e_t::E_CONTROLLER_ANALOG_RIGHT_Y));
}

void leftArcadeDrive(pros::v5::Controller& controller, const pros::v5::AbstractMotor& left, const pros::v5::AbstractMotor& right, pros::controller_analog_e_t axis1, pros::controller_analog_e_t axis2)
{
	int32_t drivePower = mapExponentialDrive(controller.get_analog(axis1));
	int32_t turnPower = std::clamp(mapExponentialTurn(controller.get_analog(axis2)), (int32_t)-70, (int32_t)70);
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


void opcontrol() 
{	
	leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	while(true)
	{
		
		leftArcadeDrive(master, leftMotors, rightMotors, pros::controller_analog_e_t::E_CONTROLLER_ANALOG_LEFT_Y, 
		pros::controller_analog_e_t::E_CONTROLLER_ANALOG_RIGHT_X);
		
		if(master.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R1))
		{
			intakeMotors.move(127);
		}
		else if(master.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_R2))
		{
			intakeMotors.move(-127);
		}
		else
		{
			intakeMotors.move(0);
		}

		if(master.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L1))
		{
			unLoader.move(127);																			
		}
		else if(master.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L2))
		{
			unLoader.move(-127);
		}
		else
		{
			unLoader.move(0);
		}

		if(master.get_digital_new_press(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_UP))
		{
			
			lift1.toggle();
			lift2.toggle();
		}		

		if(master.get_digital_new_press(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_B))
		{
			hood1.toggle();
			hood2a.toggle();
			hood2b.toggle();
		}

		if(master.get_digital_new_press(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_A))
		{
			arm.toggle();
		}

		pros::delay(20);
	 }

}