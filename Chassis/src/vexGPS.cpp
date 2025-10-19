#include "vexGPS.h"

//write excess functions here for driving with GPS

/*
gpsdrive


*/ 
void gpsdriveX( double xtgt, Mines::MinesMotorGroup left, Mines::MinesMotorGroup right, pros::GPS gps)
{
    double tol = 0.5;
    double speed = 80;
    bool go = true;
    pros::c::gps_status_s_t coord = gps.get_status();
	double x = coord.x;
	double y = coord.y;
    if(xtgt > x)
        speed *= -1;
    
    while(xtgt < x < (xtgt+tol) && go) {
        double distance = abs(xtgt-x);
        if(distance < 0.2)
            speed *= 0.35;
        right.move(speed), left.move(speed);
        coord = gps.get_status();
        x = coord.x;
        if(xtgt <= x && x <= (xtgt+tol)) {
            right.brake(), left.brake(), go = false;
        }
    }
    
    return;
} 

void gpsturn(double tgt, double speed, Mines::MinesMotorGroup left, Mines::MinesMotorGroup right, pros::GPS gps)
{
    double tol = 30.0; //tolerance on turning
    double angle = gps.get_heading();
    bool go = true;

    //should find an algorithm to determine clock vs counter
    //higher speed requires higher tolerance to hit tgt than lower speeds
    //counter-clockwise
    while(((tgt) < angle < (tgt + tol)) && go)
    {
        right.move(speed), left.move(-speed);
        angle = gps.get_heading();
        MasterController.clear_line(0);
        MasterController.print(0, 0, "%f", angle);
        if(((tgt) < angle) && (angle < (tgt + tol)))
            right.brake(), left.brake(), go = false;
    }
    MasterController.clear_line(0);
    MasterController.print(0, 0, "%f", angle);
    right.brake(), left.brake();
    
    return;
}