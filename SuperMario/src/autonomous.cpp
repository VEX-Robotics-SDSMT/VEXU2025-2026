#include "main.h"

void leftAutonRoute(void)
{


    drive.driveDistance(24.0,5000);
    drive.driveDistance(-24.0,5000);
}

void rightAutonRoute(void)
{

}

void leftRouteSkills(void)
{

}

void rightRouteSkills(void)
{

}

void autonomous() 
{
    drive.setDrivePID(0.001, 0.03, 0.0, 20.0);
    drive.capVoltage(127);
    
    switch (selectedRoute)
    {
    case RouteType::left:
        leftAutonRoute();
        break;
    case RouteType::right:
        rightAutonRoute();
        break;
    case RouteType::skillsLeft:
        leftRouteSkills();
        break;
    case RouteType::skillsRight:
        rightRouteSkills();
        break;
    default:
        break;
    }
    
}