#include "main.h"

void leftAutonRoute(void)
{
}

void leftAutonRoute2(void)
{

}

void AutonRouteSkills(void)
{

}

void autonomous() 
{
    drive.setDrivePID(0.001, 0.03, 0.0, 20.0);
    drive.capVoltage(127);

    drive.driveDistance(24.0,5000);
    drive.driveDistance(-24.0,5000);
}