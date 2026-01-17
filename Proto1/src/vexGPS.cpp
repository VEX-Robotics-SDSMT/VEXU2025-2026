#include "vexGPS.h"

//write excess functions here for driving with GPS

/*
gpsdrive


*/ 
void gpsdriveX( double xtgt, Mines::MinesMotorGroup left, Mines::MinesMotorGroup right, pros::GPS gps, int forward)
{
    double tol = 0.05;
    double speed = 80;
    bool go = true;
    pros::c::gps_status_s_t coord = gps.get_status();
	double x = coord.x;
	double y = coord.y;
    MasterController.print(0, 0, "%f", x);
    if(xtgt < x) {
        speed *=-1;
    }
    
    while(xtgt < x < (xtgt+tol) && go) {
        MasterController.clear_line(0);
        MasterController.print(0, 0, "%f", x);
        double distance = abs(xtgt-x);
        if(distance < 0.2)
            speed *= 0.35;
        right.move(speed), left.move(speed);
        coord = gps.get_status();
        x = coord.x;
        if(xtgt <= x && x <= (xtgt+tol)) {
            MasterController.print(0, 0, "stop: %f", x);
            right.brake(), left.brake(), go = false;
            return;
        }
    }
    
    return;
}

void gpsdriveY( double ytgt, Mines::MinesMotorGroup left, Mines::MinesMotorGroup right, pros::GPS gps, int forward)
{
    double tol = 0.5;
    double speed = 80;
    bool go = true;
    pros::c::gps_status_s_t coord = gps.get_status();
	double x = coord.x;
	double y = coord.y;
    if(ytgt > y)
        speed *= -1;
    
    while(ytgt < y < (ytgt+tol) && go) {
        double distance = abs(ytgt-y);
        if(distance < 0.2)
            speed *= 0.35;
        right.move(speed), left.move(speed);
        coord = gps.get_status();
        y = coord.y;
        if(ytgt <= y && x <= (ytgt+tol)) {
            right.brake(), left.brake(), go = false;
        }
    }
    
    return;
} 

void gpsturn(double tgt, double speed, Mines::MinesMotorGroup left, Mines::MinesMotorGroup right, pros::GPS gps)
{
    double tol = 5.0; // initial tolerance on turning
    double angle = gps.get_heading();
    double pct = 1.0; //percentage of the speed to have the motors run
    int cw = 0, ccw = 0;
    bool go = true;

    if(tgt > angle || angle - tgt > 180)
        speed = -speed, cw = 1; // Turn CW
    else if(tgt < angle)
        speed = speed, ccw = 1; //Turn CCW

    //while outside tolerance range, turn
    while(((tgt - (cw * tol)) < angle < (tgt + (ccw * tol))) && go)
    {
        angle = gps.get_heading();
        
        //if close to tgt, slow down and set lower tolerance
        if((abs(tgt-angle)) < 40)
            pct = 0.5, tol = 2;
        right.move(speed * pct), left.move(-speed * pct);
        angle = gps.get_heading();
        //if in range, stop looping and brake motors
        if((tgt < angle) && (angle < (tgt + tol)) && (tgt > angle - tol))
            right.brake(), left.brake(), go = false;
    }
    right.brake(), left.brake();
        
    return;
}

void gpsturncall(double tgt, double speed, int times, Mines::MinesMotorGroup left, Mines::MinesMotorGroup right, pros::GPS gps)
{
    for(int i = 0; i < times; i++)
    {
        //call turn a number of times to correct overshoots
        gpsturn(tgt, speed, left, right, gps);
        if(speed > 50)
            speed = 50;
        pros::delay(200);
    }
}