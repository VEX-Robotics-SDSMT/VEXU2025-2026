#include "../include/globals.h"
#include "pros/adi.hpp"

pros::Controller MasterController (pros::E_CONTROLLER_MASTER);

pros::Imu intertialSensor(INERTIAL_SENSOR);
pros::Optical colorSensor(COLOR_SENSOR);
pros::ADIEncoder driveEncoder(ENCODER_TOP, ENCODER_BOTTOM, true);
pros::GPS gps(VEX_GPS);

pros::Motor leftFront(L_FRONT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor leftMid(L_MID, pros::E_MOTOR_GEARSET_06, true);
pros::Motor leftRearTop(L_TREAR, pros::E_MOTOR_GEARSET_06, false);
pros::Motor leftRearBot(L_BREAR, pros::E_MOTOR_GEARSET_06, true);
pros::Motor rightFront(R_FRONT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor rightMid(R_MID, pros::E_MOTOR_GEARSET_06, false);
pros::Motor rightRearTop(R_TREAR, pros::E_MOTOR_GEARSET_06, true);
pros::Motor rightRearBot(R_BREAR, pros::E_MOTOR_GEARSET_06, false);

pros::Motor IntakeBack(I_BACK, pros::E_MOTOR_GEARSET_06, false);
pros::Motor IntakeBot(I_BOT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor IntakeMid(I_MID, pros::E_MOTOR_GEARSET_06, false);
pros::Motor IntakeFront(I_FRONT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor IntakeTop1(I_TOP1, pros::E_MOTOR_GEARSET_06, false);
pros::Motor IntakeTop2(I_TOP2, pros::E_MOTOR_GEARSET_06, false);

pros::Motor Arm(ARM_MOTOR, pros::E_MOTOR_GEARSET_06, true);

pros::ADIDigitalOut intakeDropR(TESTPNEU);
pros::ADIDigitalOut intakeDropL(TESTPNEU1);

std::vector<pros::Motor> leftDriveVector = {leftFront, leftMid, leftRearTop, leftRearBot};
std::vector<pros::Motor> rightDriveVector = {rightFront, rightMid, rightRearTop, rightRearBot};
Mines::MinesMotorGroup leftDriveMotors(leftDriveVector);
Mines::MinesMotorGroup rightDriveMotors(rightDriveVector);

double axisPercentBlue = 600.0 / 127;
double axisPercentGreen = 200.0 / 127;
double axisPercentRed = 100.0 / 127;
int blueGearbox = 600;
int greenGearbox = 200;
int redGearbox = 100;
double RED_MAX = 40;
double RED_MIN = 0;
double BLUE_MAX = 290;
double BLUE_MIN = 150;
double COLOR_MAX;
double COLOR_MIN;


uint8_t RED_GOAL_SIG_ID = 1;
uint8_t BLUE_GOAL_SIG_ID = 2;

int requiredColorLoops = 3;
const double ROLLER_TIMEOUT = 3000;

bool skills = true;
bool red_team = false;