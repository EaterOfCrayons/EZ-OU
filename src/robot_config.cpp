#include "robot_config.hpp"
#include "main.h"

//device configuration

int left_intake_port = 7;
int right_intake_port = -8;

int shooter_port = 9;

#define PTOL_PORT 'A'
#define PTOR_PORT 'B'
#define HOOK_PORT 'C'
#define WING_PORT 'D'

// Robot setup ========================================================================================================================================================================

// Defines motors:
pros::Motor left_intake(left_intake_port, pros::E_MOTOR_GEAR_GREEN); // green cartridge, 5.5W
pros::Motor right_intake(right_intake_port, pros::E_MOTOR_GEAR_GREEN); // green cartridge, 5.5W
pros::Motor shooter(shooter_port, pros::E_MOTOR_GEAR_GREEN); // green cartridge, 11W
pros::MotorGroup intakes({left_intake, right_intake});

//Define pneumatics:

pros::ADIDigitalOut ptol(PTOL_PORT); //left power-take-off piston
pros::ADIDigitalOut ptor(PTOR_PORT); //right power-take-off piston
pros::ADIDigitalOut hook(HOOK_PORT); //hook mechanism piston
pros::ADIDigitalOut wing(WING_PORT); //wing mechanism piston
