#pragma once
#include "pros/motor_group.hpp"

//===============================================
//Ports
//===============================================

//Left Motors
const inline int LM1 = 16;
const inline int LM2 = 17;
const inline int LM3 = 18;
const inline int LM4 = 19;

//Right Motors
const inline int RM1 = 4;
const inline int RM2 = 5;
const inline int RM3 = 6;
const inline int RM4 = 7;

//Intake Motors
const inline int IN1 = 8;
const inline int IN2 = 9;

//===============================================
//Motor Groups
//===============================================

//Drive Train Motors
const inline pros::MotorGroup leftMotors({-LM1, -LM2, -LM3, -LM4});
const inline pros::MotorGroup rightMotors({RM1, RM2, RM3, RM4});

//Intake Group
const inline pros::MotorGroup intakeMotors({IN1, IN2});