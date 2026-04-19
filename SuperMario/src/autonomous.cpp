#include "main.h"

void leftAutonRoute(void)
{

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
    drive.setDrivePID(0.001, 0.03, 0.001, 20.0);
    drive.setTurnPID(1.5, 0.000000001, 0, 99999999.0);
    drive.capVoltage(127);
    drive.turnTo(90,0.75,200);
    drive.turnTo(-90,0.75,200);
    

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