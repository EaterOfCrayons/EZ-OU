#pragma once
#include "api.h"
#include "main.h"

using namespace pros;


// externalize device configurations
extern Motor intake;
extern Motor shooter;
extern ADIDigitalOut right_wing;
extern ADIDigitalOut left_wing;
extern ADIDigitalOut hang;
extern ADIDigitalOut hook;
extern ADIDigitalOut sled;

extern MotorGroup left_motors;
extern MotorGroup right_motors;

extern Drive chassis;

extern Rotation left_rot;
extern Rotation right_rot;
extern Rotation back_rot;
extern Rotation cata_rot;

class catapult // creates the catapult class
{
public:
    bool is_continuous{false}; // whether the catapult is set to continuous fire
    bool override{false};      // whether to manually override catapult control
    int state{0};              // the state of the catapult. 0: unloaded. 1: loading. 2: loaded. 3:continuous
    void lower();              // Declare reset function
    void launch();             // Declare function to fire the catapult
    void continuous();         // Declare function to fire the catapult continuously
};


extern catapult cata; // creates an instance of the catapult object

