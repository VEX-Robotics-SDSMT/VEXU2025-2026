#include "main.h"

void leftAutonRoute(void)
{
    drive.driveDistance(200,5);
    drive.driveDistance(-24.0,5);
    drive.turnTo(90);
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
    leftAutonRoute();
    /*
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
        */
    
}