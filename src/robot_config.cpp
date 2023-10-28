#include "robot_config.hpp"
#include "main.h"

// device configuration

int left_front_port = -1;
int left_middle_port = -2;
int left_back_port = 3;

int right_front_port = 4;
int right_middle_port = 5;
int right_back_port = -6;

int intake_port = -7;

int shooter_port = -9;

int left_rot_port = 10;
int right_rot_port = 11;
int back_rot_port = 12;
int cata_rot_port = 13;

int inertial_sensor_port = 14;

#define LEFT_WING_PORT 'A'
#define RIGHT_WING_PORT 'B'
#define LIFT_PORT 'C'
#define SLED_PORT 'E'
#define HOOK_PORT 'F'

// Robot setup ========================================================================================================================================================================

// Defines motors:
/*
pros::Motor left_front_motor(left_front_port, pros::E_MOTOR_GEAR_BLUE);      // blue cartridge, 11W
pros::Motor left_middle_motor(left_middle_port, pros::E_MOTOR_GEARSET_06);   // blue cartridge, 11W
pros::Motor left_back_motor(left_back_port, pros::E_MOTOR_GEAR_BLUE);        // blue cartridge, 11W
pros::Motor right_front_motor(right_front_port, pros::E_MOTOR_GEAR_BLUE);    // blue cartridge, 11W
pros::Motor right_middle_motor(right_middle_port, pros::E_MOTOR_GEARSET_06); // blue cartridge, 11W
pros::Motor right_back_motor(right_back_port, pros::E_MOTOR_GEAR_BLUE);      // blue cartridge, 11W
*/
pros::Motor intake(intake_port, pros::E_MOTOR_GEAR_BLUE);                    // blue cartridge, 11W
pros::Motor shooter(shooter_port, pros::E_MOTOR_GEAR_RED);                   // red cartridge, 11W

// Define pneumatics:

pros::ADIDigitalOut left_wing(LEFT_WING_PORT);   // wing mechanism piston
pros::ADIDigitalOut right_wing(RIGHT_WING_PORT); // wing mechanism piston
pros::ADIDigitalOut sled(SLED_PORT);             // sled
pros::ADIDigitalOut hang(LIFT_PORT);             // lift piston
pros::ADIDigitalOut hook(HOOK_PORT);             // hook piston

// Defines catapult rotation sensor:

pros::Rotation cata_rot(cata_rot_port, true); // catapult rotation sensor

// Chassis constructor
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

    // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
    //    (or tracking wheel diameter)
    ,
    3.25

    // Cartridge RPM
    //   (or tick per rotation if using tracking wheels)
    ,
    600

    // External Gear Ratio (MUST BE DECIMAL)
    //    (or gear ratio of tracking wheel)
    // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
    // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
    ,
    1.3333

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

void catapult::lower()
{ // catapult reset function
    shooter.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    state = 1; // sets the state to "loading"
    while (cata_rot.get_position() < 5500)
    {                  // executes while catapult is not at the desired angle
        shooter = 75; // outputs power to the catapult motor
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
    // }
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

