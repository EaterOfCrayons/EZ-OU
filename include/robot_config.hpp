#pragma once
#include "api.h"
#include "main.h"

using namespace pros;


// externalize device configurations
extern Motor intake;
extern Motor shooter;
extern ADIDigitalOut right_wing;
extern ADIDigitalOut left_wing;
extern ADIDigitalOut pto_piston;
extern ADIDigitalOut ratchet;
extern ADIDigitalOut sled;
extern ADIDigitalOut hook;

extern MotorGroup left_motors;
extern MotorGroup right_motors;

extern Drive chassis;

extern Rotation cata_rot;
extern Rotation lift_rot;

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

class ptoClass // creates the pto class
{
    public:
    bool pto_enable{false}; // pto toggle variable
    void set_pto(bool toggle); // pto toggle procedure
};

extern ptoClass pto; // creates an instance of the pto class object
extern catapult cata; // creates an instance of the catapult object
