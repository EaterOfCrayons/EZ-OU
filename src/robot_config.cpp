#include "robot_config.hpp"
#include "main.h"

// ANCHOR device configuration

int left_front_port = -4;
int left_middle_port = -2;
int left_back_port = -3;

int right_front_port = 17;
int right_middle_port = 5;
int right_back_port = 6;

int intake_port = -14;

int shooter_port = -12;

int cata_rot_port = 8;
int lift_rot_port = 20;

int inertial_sensor_port = 11;

#define LEFT_WING_PORT 'A'
#define RIGHT_WING_PORT 'B'
#define PTO_PORT 'C'
#define RATCHET_PORT 'D'
#define SLED_PORT 'E'
#define HOOK_PORT 'F'

// ANCHOR Robot setup 

// Defines motors:

pros::Motor left_front_motor(left_front_port, pros::E_MOTOR_GEAR_BLUE);      // blue cartridge, 11W
pros::Motor left_middle_motor(left_middle_port, pros::E_MOTOR_GEARSET_06);   // blue cartridge, 11W
pros::Motor left_back_motor(left_back_port, pros::E_MOTOR_GEAR_BLUE);        // blue cartridge, 11W
pros::Motor right_front_motor(right_front_port, pros::E_MOTOR_GEAR_BLUE);    // blue cartridge, 11W
pros::Motor right_middle_motor(right_middle_port, pros::E_MOTOR_GEARSET_06); // blue cartridge, 11W
pros::Motor right_back_motor(right_back_port, pros::E_MOTOR_GEAR_BLUE);      // blue cartridge, 11W

pros::MotorGroup left_motors({left_front_motor, left_middle_motor, left_back_motor});
pros::MotorGroup right_motors({right_front_motor, right_middle_motor, right_back_motor});

pros::Motor intake(intake_port, pros::E_MOTOR_GEAR_BLUE);  // blue cartridge, 11W
pros::Motor shooter(shooter_port, pros::E_MOTOR_GEAR_RED); // red cartridge, 11W

// Define pneumatics:

pros::ADIDigitalOut left_wing(LEFT_WING_PORT);   // wing mechanism piston
pros::ADIDigitalOut right_wing(RIGHT_WING_PORT); // wing mechanism piston
pros::ADIDigitalOut sled(SLED_PORT);             // sled
pros::ADIDigitalOut pto_piston(PTO_PORT);        // pto piston
pros::ADIDigitalOut hook(HOOK_PORT);             // hook piston
pros::ADIDigitalOut ratchet(RATCHET_PORT);       // ratchet piston

// Defines rotation sensors:

pros::Rotation cata_rot(cata_rot_port, true);  // catapult rotation sensor
pros::Rotation lift_rot(lift_rot_port, false); // lift rotation sensor

// ANCHOR Chassis constructor

Drive chassis(
    // Left Chassis Ports (negative port will reverse it!)
    //   the first port is the sensored port (when trackers are not used!)
    {left_front_port, left_middle_port, left_back_port}

    // Right Chassis Ports (negative port will reverse it!)
    //   the first port is the sensored port (when trackers are not used!)
    ,
    {right_front_port, right_middle_port, right_back_port}

    // IMU Port
    ,
    inertial_sensor_port

    // Wheel Diameter
    ,
    3.25

    // Cartridge RPM
    ,
    600

    // External Gear Ratio
    ,
    1.667

    // Uncomment if using tracking wheels
    /*
    // Left Tracking Wheel Ports (negative port will reverse it!)
    // ,{1, 2} // 3 wire encoder
    // ,8 // Rotation sensor

    // Right Tracking Wheel Ports (negative port will reverse it!)
    // ,{-3, -4} // 3 wire encoder
    // ,-9 // Rotation sensor
    */

    // Uncomment if tracking wheels are plugged into a 3 wire expander
    // 3 Wire Port Expander Smart Port
    // ,1
);

/**
 * ANCHOR toggle pto procedure
 * @param bool toggle
 * sets the pto state with given parameter
*/
void ptoClass::set_pto(bool toggle) 
{
    pto_enable = toggle; //sets the global state variable to the deried state

    if (toggle) // if the pto is desired to be raised
    {
        
        pto_piston.set_value(toggle); // actuate both pto pistons
        chassis.left_motors[2] = 30;
        chassis.right_motors[2] = 30;
        pros::delay(200);
        chassis.left_motors[2] = 0;
        chassis.right_motors[2] = 0;
        chassis.pto_toggle({chassis.left_motors[2], chassis.right_motors[2]}, toggle); // disables the pto motors from the chassis copntrols
        chassis.right_motors[2].set_brake_mode(E_MOTOR_BRAKE_HOLD); // sets both motors to hold
        chassis.left_motors[2].set_brake_mode(E_MOTOR_BRAKE_HOLD);
        while (lift_rot.get_position() < 9900 && !pto_override) // continuously spins motors until the pto is raised to 85 degrees
        {
            chassis.left_motors[2] = 120;
            chassis.right_motors[2] = 120;
        }
        chassis.right_motors[2] = 0;
        chassis.left_motors[2] = 0;
    } else if (!toggle){ // if the pto is desired to be lowerd
        chassis.right_motors[2].set_brake_mode(E_MOTOR_BRAKE_HOLD); // disable hold from both motors
        chassis.left_motors[2].set_brake_mode(E_MOTOR_BRAKE_HOLD);
        while (lift_rot.get_position() > 500 && !pto_override){ //continuously spins the pto motors in reverse until thje pto is lowered to 15 degrees
            chassis.left_motors[2] = -120;
            chassis.right_motors[2] = -120;
        }
        chassis.right_motors[2] = 0;
        chassis.left_motors[2] = 0;
        if(!pto_override){
            pto_piston.set_value(toggle); // shift the pto pistons back to drive state
            chassis.right_motors[2].set_brake_mode(E_MOTOR_BRAKE_COAST); // disable hold from both motors
            chassis.left_motors[2].set_brake_mode(E_MOTOR_BRAKE_COAST);
            chassis.left_motors[2] = -30;
            chassis.right_motors[2] = -30;
        }
        
        pros::delay(200);
        chassis.right_motors[2] = 0;
        chassis.left_motors[2] = 0;
        chassis.pto_toggle({chassis.left_motors[2], chassis.right_motors[2]}, toggle); // re-enables the pto motors and assigns them to the drive class

    }
    master.rumble("-"); // vibrates the controller to signal the driver that the state change has finished
}

// ANCHOR catapult control
void catapult::lower()
{ // catapult reset function
    shooter.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    state = 1; // sets the state to "loading"
    while (cata_rot.get_position() < 4000)
    {                  // executes while catapult is not at the desired angle
        shooter = 127; // outputs power to the catapult motor
        pros::delay(10);
    }
    shooter.brake(); // stops the catapult motor
    state = 2;       // sets the state to "loaded"
}

void catapult::launch()
{ // function to fire the catapult
    shooter.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    shooter = 120;
    pros::delay(200);
    shooter.brake();
}

void catapult::continuous()
{ // function to repeatedly fire the catapult

    while (true)
    {

        if (cata.state == 2 && cata.is_continuous)
        {

            cata.launch();
            pros::delay(100);
        }
        pros::delay(20);
    }
}

catapult cata; // creates an instance of the catapult object
ptoClass pto;  // creates an instance of the pto object
