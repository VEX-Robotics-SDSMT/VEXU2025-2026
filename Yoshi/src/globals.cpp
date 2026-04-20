#include "../include/globals.h"
#include "pros/adi.hpp"

pros::Controller MasterController (pros::E_CONTROLLER_MASTER);

pros::Imu intertialSensor(INERTIAL_SENSOR);
pros::Vision vision(VISION_SENSOR);
pros::ADIEncoder driveEncoder(ENCODER_TOP, ENCODER_BOTTOM, true);
pros::GPS gps(VEX_GPS);

pros::Motor leftFront(L_FRONT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor leftFrontMid(L_FRONTM, pros::E_MOTOR_GEARSET_06, false);
pros::Motor leftRearMid(L_REARM, pros::E_MOTOR_GEARSET_06, false);
pros::Motor leftRear(L_REAR, pros::E_MOTOR_GEARSET_06, false);
pros::Motor rightFront(R_FRONT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor rightFrontMid(R_FRONTM, pros::E_MOTOR_GEARSET_06, true);
pros::Motor rightRearMid(R_REARM, pros::E_MOTOR_GEARSET_06, true);
pros::Motor rightRear(R_REAR, pros::E_MOTOR_GEARSET_06, true);

pros::Motor IntakeRear(I_BACK, pros::E_MOTOR_GEARSET_06, false);
pros::Motor IntakeMid(I_MID, pros::E_MOTOR_GEARSET_06, true);
pros::Motor IntakeBot(I_BOT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor IntakeTop(I_TOP, pros::E_MOTOR_GEARSET_18, true);
pros::Motor IntakeScore(I_SCORE, pros::E_MOTOR_GEARSET_18, true);
pros::Motor IntakeFrontL(I_FRONTL, pros::E_MOTOR_GEARSET_18, true);
pros::Motor IntakeFrontR(I_FRONTR, pros::E_MOTOR_GEARSET_18, false);
pros::Motor IntakeTiny(I_TINY, pros::E_MOTOR_GEARSET_18, true);


pros::ADIDigitalOut intakeLiftR(LIFT1);
pros::ADIDigitalOut intakeLiftL(LIFT2);
pros::ADIDigitalOut wing(WING);

std::vector<pros::Motor> leftDriveVector = {leftFront, leftFrontMid, leftRearMid, leftRear};
std::vector<pros::Motor> rightDriveVector = {rightFront, rightFrontMid, rightRearMid, rightRear};
std::vector<pros::Motor> intakeWheelsVector = {IntakeFrontL, IntakeFrontR};
Mines::MinesMotorGroup leftDriveMotors(leftDriveVector);
Mines::MinesMotorGroup rightDriveMotors(rightDriveVector);
Mines::MinesMotorGroup intakeWheels(intakeWheelsVector);

double axisPercentBlue = 600.0 / 127;
double axisPercentGreen = 200.0 / 127;
double axisPercentRed = 100.0 / 127;
int blueGearbox = 600;
int greenGearbox = 200;
int redGearbox = 100;


uint8_t RED_GOAL_SIG_ID = 1;
uint8_t BLUE_GOAL_SIG_ID = 2;

int requiredColorLoops = 3;
const double ROLLER_TIMEOUT = 3000;

bool skills = false;
bool lower = true;