#pragma once
#include "pros/motor_group.hpp"
#include "Drivetrain.h"

//===============================================
//Controller
//===============================================
inline pros::Controller master(pros::controller_id_e_t::E_CONTROLLER_MASTER);

//===============================================
//Ports
//===============================================

//Left Motors
const inline int8_t LM1 = 1;
const inline int8_t LM2 = 2;
const inline int8_t LM3 = 3;
const inline int8_t LM4 = 20;

//Right Motors
const inline int8_t RM1 = 10;
const inline int8_t RM2 = 13;
const inline int8_t RM3 = 12;
const inline int8_t RM4 = 11;

//Intake Motors
const inline int8_t IN1 = 21;
const inline int8_t IN2 = 16;
const inline int8_t IN3 = 19;
const inline int8_t IN4 = 18;

//Encoder
const inline char ENC_A = 'A';
const inline char ENC_B = 'B';

//Inertial
const inline uint8_t INERT = 14;

//Pneumatics
const inline char ARM_PORT = 'H';
const inline char LIFT_PORT = 'G';


//===============================================
//Motor Groups
//===============================================

//Drive Train Motors
inline pros::MotorGroup leftMotors({-LM1, -LM2, -LM3, -LM4});
inline pros::MotorGroup rightMotors({RM1, RM2, RM3, RM4});

//Intake Group
inline pros::MotorGroup intakeMotors({IN1, IN2, IN3});

//===============================================
//Sensors
//===============================================

//Inertial Senor
inline pros::Imu imu(INERT);

//Rotational Senor
inline pros::adi::Encoder enc(ENC_A, ENC_B, false);

//===============================================
//Pneumatics
//===============================================
inline pros::adi::Pneumatics arm(ARM_PORT, false);
inline pros::adi::Pneumatics lift(LIFT_PORT, false);


//===============================================
//Drivetrain
//===============================================
inline Mines::Drivetrain drive(&leftMotors, &rightMotors, &enc, &imu);