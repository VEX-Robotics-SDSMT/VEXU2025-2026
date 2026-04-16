#include "main.h"

struct AutonRoute
{
	const char* name;
	RouteType type;
};

static const std::vector<AutonRoute> autonomousRouteOptions = 
{
	{"Left Autonomous", RouteType::left},
	{"Right Autonomous", RouteType::right},
	{"Left Skills", RouteType::skillsLeft},
	{"Right Skills", RouteType::skillsRight}
};


void initializeDevices()
{
	drive.initilizeDevices();
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() 
{
	initializeDevices();
	/*
	pros::screen::set_eraser(pros::Color::black);
	pros::screen::erase();

	arm.retract();
	lift.retract();

	std::size_t size = autonomousRouteOptions.size();
	std::size_t idx = 0;
	std::size_t lastIdx = 0;
	master.clear();
	pros::delay(110);
	master.print(0,0,"Select an auton");
	pros::delay(110);
	master.print(2,0,autonomousRouteOptions[idx].name);
	while(!master.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_A))
	{	
		if(master.get_digital_new_press(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_UP))
		{
			if (idx + 1 < size)
			{
				idx++;
			}
		}
		else if(master.get_digital_new_press(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_DOWN))
		{
			if (idx > 0)
			{
				idx--;
			}
		}

		if(idx != lastIdx)
		{
			master.clear_line(2);
			pros::delay(110);
			master.print(2,0,autonomousRouteOptions[idx].name);
			lastIdx = idx;
		}	
		pros::delay(20);
	}
	selectedRoute = autonomousRouteOptions[idx].type;
	master.clear();
	pros::delay(110);
	master.print(0,0,"Ready for match!");
*/
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
void competition_initialize() {}