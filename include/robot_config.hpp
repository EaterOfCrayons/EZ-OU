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
extern ADIDigitalOut wings;


extern Drive chassis;

extern Motor& left_pto;
extern Motor& right_pto;


extern bool pto_hook_enabled;
void pto_hook(bool toggle);
void set_hook(int speed, float time, bool coasted);


