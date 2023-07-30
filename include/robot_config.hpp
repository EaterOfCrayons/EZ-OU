#pragma once
#include "api.h"
#include "main.h"

using namespace pros;


// externalize device configurations
//extern Motor left_intake;
//extern Motor right_intake;
extern Motor shooter;
extern MotorGroup intakes;

extern ADIDigitalOut ptol;
extern ADIDigitalOut ptor;
extern ADIDigitalOut wing_left;
extern ADIDigitalOut wing_right;

extern Drive chassis;

extern Motor& left_pto_1;
extern Motor& left_pto_2;

extern bool pto_hook_enabled;
void pto_hook(bool toggle);
void set_hook(int speed, float time, bool coasted);


