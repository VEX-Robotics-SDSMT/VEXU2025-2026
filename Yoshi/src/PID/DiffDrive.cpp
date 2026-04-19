#include "DiffDrive.h"

using namespace Mines;
using namespace std;


DiffDrive::DiffDrive(MinesMotorGroup left, MinesMotorGroup right, pros::IMU imu) : 
    leftMotors(left), rightMotors(right), inertial(imu),
    driveInterface(this), turnInterface(this),
    drivePID(&driveInterface, LoggerSettings::none), turnPID(&turnInterface, LoggerSettings::none),
    logger(LoggerSettings::none)
{
    MAX_SPEED = rightMotors.getMaxVelocity();

    DriveSensorInterface driveSensors(left, right);
    driveSensorInterface = &driveSensors;
    driveSensorInterface->Reset();
    StartPIDs();
}

DiffDrive::DiffDrive(MinesMotorGroup left, MinesMotorGroup right, SensorInterface *driveSensors, pros::Imu imu) :
    leftMotors(left), rightMotors(right), inertial(imu),
    driveInterface(this), turnInterface(this),
    drivePID(&driveInterface, LoggerSettings::none), turnPID(&turnInterface, LoggerSettings::verbose),
    logger(LoggerSettings::none)
{
    MAX_SPEED = rightMotors.getMaxVelocity();

    logger.Log("status: constructor called", 10, LoggerSettings::verbose);

    driveSensorInterface = driveSensors;
    driveSensorInterface->Reset();
    StartPIDs();
}

DiffDrive::~DiffDrive()
{
    killPIDs();
}

// *****************************************
// returns average current velocity
double DiffDrive::getDriveVelocity()
{
    return (leftMotors.getActualVelocity() + rightMotors.getActualVelocity()) / 2;
}

//returns current spinning velocity
double DiffDrive::getTurnVelocity()
{
    return inertial.get_gyro_rate().z;
}

//will cause the robot to drive for the target distance and may wait for completion
/*EX call:
    * DiffDrive drive.driveTiles(1000, true) //will wait for completion
    * DiffDrive drive.driveTiles(1000, false) //will not wait, just start and stop
*/
void DiffDrive::driveTiles(double target, bool waitForCompletion)
{
    driveSensorInterface->Reset();

    drivePID.SetTarget(target);
    if(waitForCompletion)
    {
        while(drivePID.GetTimeSinceTargetReached() < GOAL_TIME)
        {
            pros::c::delay(20);
        }
    }
}

//will cause the robot to drive for the target distance and stop after the timeout
/*EX call:
    * DiffDrive drive.driveTiles(1000, 1000) //will move until reach target or 1s has passed
    * DiffDrive drive.driveTiles(1000, 2000) //will move until reach target or 2s has passed
*/
void DiffDrive::driveTiles(double target, int timeOut)
{
    driveSensorInterface->Reset();
    drivePID.SetTarget(target);

    while(drivePID.GetTimeSinceTargetReached() < GOAL_TIME && drivePID.GetTimeSinceTargetSet() < timeOut)
    {
        pros::c::delay(20);
    }

    drivePID.SetTarget(getDrivePosition());
}

//will cause the robot to turn to the absolute angle based on initialization angle
/*EX call:
    * DiffDrive drive.turnDegreesAbsolute(180, true) //will turn to 180 degrees relative to initial position
    * DiffDrive drive.turnDegreesAbsolute(90, true) //will turn to 90 degrees relative to initial position
*/
void DiffDrive::turnDegreesAbsolute(double target, bool waitForCompletion)
{
    turnPID.SetTarget(target);
    if(waitForCompletion)
    {
        while(turnPID.GetTimeSinceTargetReached() < GOAL_TIME)
        {
            pros::c::delay(20);
        }
    }
}

//will cause the robot to turn to the absolute angle based on initialization angle
/*EX call:
    * DiffDrive drive.turnDegreesAbsolute(180, 1000) //will turn to 180 degrees relative to initial position
    * DiffDrive drive.turnDegreesAbsolute(90, 1000) //will turn to 90 degrees relative to initial position
*/
void DiffDrive::turnDegreesAbsolute(double target, int timeOut)
{
    turnPID.SetTarget(target);
    while(turnPID.GetTimeSinceTargetReached() < GOAL_TIME && turnPID.GetTimeSinceTargetSet() < timeOut)
    {
        pros::c::delay(20);
    }

    turnPID.SetTarget(getTurnPosition());
}

//sets the brake mode of the motors to a different mode
/*MODES:
    * pros::E_MOTOR_BRAKE_COAST //Motor coasts after calling brake()
    * pros::E_MOTOR_BRAKE_BRAKE //Motor short brakes ^^
    * pros::E_MOTOR_BRAKE_HOLD  //Motor will constant brake **WARNING** using this can over-exert motors
*/
void DiffDrive::setBrakeMode(pros::motor_brake_mode_e mode)
{
    leftMotors.setBrakeMode(mode);
    rightMotors.setBrakeMode(mode);
}

//Sets the PID values for driving, these values will need to be tuned to increase efficiency and accuracy
/*EX call:
    * drive.setDrivePIDVals(2, 1.2, 0); //will set P=2, I=1.2, D=0
*/

void DiffDrive::setDrivePIDVals(double kp, double ki, double kd)
{
    drivePID.SetPIDConst(kp, ki, kd);
}

//Sets the PID values for turning, these values will need to be tuned to increase efficiency and accuracy
/*EX call:
    * drive.setTurnPIDVals(1, 0.8, 0); //will set P=1, I=0.8, D=0
*/
void DiffDrive::setTurnPIDVals(double kp, double ki, double kd)
{
    turnPID.SetPIDConst(kp, ki, kd); 
}

//Sets the drive PID tolerance to +/- tolerance passed in
/*EX call:
    * drive.setDrivePIDTol(50); //will set the drive tolerance to 50 ticks
*/
void DiffDrive::setDrivePIDTol(double tolerance)
{
    drivePID.SetTolerance(tolerance);
}

//Sets the turn PID tolerance to +/- tolerance passed in
/*EX call:
    * drive.setTurnPIDTol(2); //will set the turn tolerance to 2 degrees
*/
void DiffDrive::setTurnPIDTol(double tolerance)
{
    turnPID.SetTolerance(tolerance);
}

//Sets the max drive speed to drive at by a percentage of the max allowed by the motor gearing
/*EX call:
    * drive.setMaxDriveSpeed(0.8); //will set the max speed to 80% of max
*/
void DiffDrive::setMaxDriveSpeed(double percent)
{
    MAX_DRIVE_PERCENT = percent;
}

//Sets the max turn speed to drive at by a percentage of the max allowed by the motor gearing
/*EX call:
    * drive.setMaxTurnSpeed(0.8); //will set the max speed to 80% of max
*/
void DiffDrive::setMaxTurnSpeed(double percent)
{
    MAX_TURN_PERCENT = percent;
}

//Sets the max drive acceleration
/*EX call:
    * drive.setMaxDriveAccel(0.2); //will set the acceleration to 0.2 ticks/sec
*/
void DiffDrive::setMaxDriveAccel(double value)
{
    MAX_DRIVE_ACCEL = value;
}

//Sets the max turn acceleration
/*EX call:
    * drive.setMaxTurnAccel(0.2); //will set the acceleration to 0.2 ticks/sec
*/
void DiffDrive::setMaxTurnAccel(double value)
{
    MAX_TURN_ACCEL = value;
}

//gets the current position relative to start point
double DiffDrive::getDrivePosition()
{
    double sensorVal = driveSensorInterface->Get();
    std::cout << sensorVal << "\n";
    return sensorVal;
}

//sets the new drive velocity based on acceleration and speed
void DiffDrive::setDriveVelocity(double value)
{
    double adjustedDriveMaxAccel = MAX_DRIVE_ACCEL * MAX_SPEED;
    double dyanamicMax = fabs(getDriveVelocity()) + adjustedDriveMaxAccel;
    double clampedVal = std::clamp(value, -dyanamicMax, dyanamicMax);

    logger.Log("adj: " + std::to_string(adjustedDriveMaxAccel) +
    " dyn: " + std::to_string(dyanamicMax), 3, LoggerSettings::verbose);
    logger.Log("Target drive velocity: " + std::to_string(clampedVal), 4, LoggerSettings::verbose);

    driveVelocity = clampedVal;
    setMotorVelocities();
}

//gets the current turn angle
double DiffDrive::getTurnPosition()
{
    double current = inertial.get_heading();

    double target = turnPID.GetTarget();

    if (current - target > 180)
    {
        return 360 - current;
    }
    else if (target - current > 180)
    {
        return current + 360;
    }
    else
    {
        return current;
    }
}

//sets the turning velocity
void DiffDrive::setTurnVelocity(double value)
{
    turnVelocity = value;
    setMotorVelocities();
}

//sets the motor velocities based on current state
void DiffDrive::setMotorVelocities()
{
    double adjustedDriveMax = MAX_DRIVE_PERCENT * MAX_SPEED;
    double adjustedTurnMax = MAX_TURN_PERCENT * MAX_SPEED;

    double adjustedDriveVelocity = clamp(driveVelocity, -adjustedDriveMax, adjustedDriveMax);
    double adjustedTurnVelocity = clamp(turnVelocity, -adjustedTurnMax, adjustedTurnMax);

    double targetLeftSpeed = adjustedDriveVelocity + adjustedTurnVelocity;
    double targetRightSpeed = adjustedDriveVelocity - adjustedTurnVelocity;

    double scaleFactor = min(MAX_SPEED / max(fabs(targetLeftSpeed), fabs(targetRightSpeed)), 1.0);

    int targetLeftVoltage = ((targetLeftSpeed * scaleFactor) * 12000) / 127;
    int targetRightVoltage = ((targetRightSpeed * scaleFactor) * 12000) / 127;

    if (ACTIVE)
    {
        leftMotors.moveVoltage(targetLeftVoltage);
        rightMotors.moveVoltage(targetRightVoltage);
    }
}

//will activate the PIDs from a paused state
void DiffDrive::setActive(bool active)
{
    ACTIVE = active;

    if (active == true)
    {
        driveTiles(0, 50);
        turnDegreesAbsolute(inertial.get_heading(), 50);
    }
}

//will turn off the PIDs
void DiffDrive::killPIDs()
{
    drivePID.KillTask();
    turnPID.KillTask();
    leftDriveMotors.brake();
    rightDriveMotors.brake();
    pros::delay(200);
}

//will start the PIDs
void DiffDrive::StartPIDs()
{
    drivePID.StartTask("drive PID");
    turnPID.StartTask("turn PID");
}

//will pause the PIDs
void DiffDrive::SetPausedPID(bool paused)
{
    PIDPaused = paused;
    drivePID.SetTaskPaused(paused);
    turnPID.SetTaskPaused(paused);
}

//will return if the PID is paused or not
bool DiffDrive::GetPausedPID()
{
    return PIDPaused; 
}


//--------------------nested classes-----------------------


DiffDrive::DriveInterface::DriveInterface(DiffDrive* pParent)
{
    parent = pParent;
}

double DiffDrive::DriveInterface::getPositionPID()
{
    return parent->getDrivePosition();
}

void DiffDrive::DriveInterface::setVelocityPID(double value)
{
    parent->setDriveVelocity(value);
}

DiffDrive::TurnInterface::TurnInterface(DiffDrive* pParent)
{
    parent = pParent;
}

double DiffDrive::TurnInterface::getPositionPID()
{
    return parent->getTurnPosition();
}

void DiffDrive::TurnInterface::setVelocityPID(double value)
{
    parent->setTurnVelocity(value);
}


//Encoder Wheel Sensor
EncoderWheelSensorInterface::EncoderWheelSensorInterface(pros::ADIEncoder encoder) : encoder(encoder) {}

double EncoderWheelSensorInterface::Get()
{
    double sensorVal = encoder.get_value();
    std::cout << "encoder val: " << sensorVal <<"errno:" <<errno <<  "\n";

    return sensorVal;
}

void EncoderWheelSensorInterface::Reset()
{
    encoder.reset();
}

//Motor Wheel Sensor
DiffDrive::DriveSensorInterface::DriveSensorInterface(MinesMotorGroup left, MinesMotorGroup right) : left(left), right(right) {}

double DiffDrive::DriveSensorInterface::Get()
{
    return (left.getPosition() + right.getPosition()) / 2;
}

void DiffDrive::DriveSensorInterface::Reset()
{
    left.tarePosition();
    right.tarePosition();
}


