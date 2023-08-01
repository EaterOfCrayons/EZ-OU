#include "robot_config.hpp"
#include "main.h"

//device configuration

int left_intake_port = 7;
int right_intake_port = -8;

int shooter_port = 9;

#define PTOL_PORT 'A'
#define PTOR_PORT 'B'
#define WING_PORTS 'C'
#define WING_RIGHT_PORT 'D'

// Robot setup ========================================================================================================================================================================

// Defines motors:
pros::Motor left_intake(left_intake_port, pros::E_MOTOR_GEAR_GREEN); // green cartridge, 5.5W
pros::Motor right_intake(right_intake_port, pros::E_MOTOR_GEAR_GREEN); // green cartridge, 5.5W
pros::Motor shooter(shooter_port, pros::E_MOTOR_GEAR_GREEN); // green cartridge, 11W
pros::MotorGroup intakes({left_intake, right_intake});

//Define pneumatics:

pros::ADIDigitalOut ptol(PTOL_PORT); //left power-take-off piston
pros::ADIDigitalOut ptor(PTOR_PORT); //right power-take-off piston
pros::ADIDigitalOut wings(WING_PORTS); //hook mechanism piston

// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {-1, -2, 3}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{4, 5, -6}

  // IMU Port
  ,13

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,3.25

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,1.3333

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

//PTO setup
pros::Motor& left_pto = chassis.left_motors[2];
pros::Motor& right_pto = chassis.right_motors[2];

bool pto_hook_enabled = false;

void pto_hook(bool toggle) {
  pto_hook_enabled = toggle;
  right_pto.set_brake_mode(E_MOTOR_BRAKE_COAST);
  chassis.pto_toggle({left_pto, right_pto}, toggle);
  ptol.set_value(toggle);
  
  for (int i = 0; i < 6; i++){
    left_pto = 120;
    pros::delay(20);
    left_pto = -120;
    pros::delay(200); 
  }
  left_pto = 0;
  
}

void set_hook(int speed, float time, bool coasted) {
  if (!pto_hook) return;
  left_pto.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  left_pto = speed;;
  pros::delay(time);
  if (coasted){
    left_pto.set_brake_mode(E_MOTOR_BRAKE_COAST);
    left_pto = 0;
  }
  else{
    left_pto = 0;
  }
  
  return;
}



