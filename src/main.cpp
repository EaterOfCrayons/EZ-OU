#include "main.h"
#include "robot_config.hpp"
#include "autons.hpp"

// initialize runtime objects
double fwd;
double turning;
float up;
float down;
bool hooker = false;
bool wedge = false;
bool lifted = false;

void cata_control()
{
    pros::Task continuous  {[=] { // creates a lambda task for cata.continuous
        cata.continuous();
    }};
    while (true)
    {
        if (cata_rot.get_position() < 1000 && cata.state != 1 ) // checks if the catapult should be lowered
        {
            cata.state = 0;
        } 
                                                                    // checks if the catapult is in override mode
        if (cata.state == 0) // starts lowering the catapult if the cata is unloaded and not currently loading
        {
            cata.lower(); // calls the catapult reset function
        }
        

        pros::delay(15);
    }
}

void terminal_print()
{
    int counter {0}; 
    
    while (true)
    {

        float voltage_float = (fabs(chassis.left_velocity()) + fabs(chassis.right_velocity())) / 2; // gets and stores the current power output in mV of the motors in float variables
        float position_float = (fabs(chassis.left_sensor()) + fabs(chassis.right_sensor())) / 2;
        std::string voltage_str = std::to_string(voltage_float); // converts the float variables to strings
        std::string position_str = std::to_string(position_float);
        std::cout << counter << "," << voltage_str << "," << position_str << "\n";
        pros::delay(50);
        counter++;
        
    }
}


void initialize() {
  ez::print_ez_template();
  pros::delay(500);
  // Configure chassis controls
  chassis.toggle_modify_curve_with_controller(false); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0); // Sets the active brake kP. 
  chassis.set_curve_default(0, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to constants
  exit_condition_defaults(); // Set the exit conditions constants

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
    Auton("Right Auton", right_auton),
    Auton("Left Auton", left_auton),
    Auton("Skills", skills),
    Auton("Trust alliance", no_auton),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  cata_rot.reset_position();
  lift_rot.reset_position();
  pros::Task cata_task(cata_control);
  //pros::Task record(terminal_print);
}

void disabled() {

}


void competition_initialize() {

}



void autonomous() {

  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.
  ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
}

// function implementing input curve in arcade drive controls
void arcadeCurv(pros::controller_analog_e_t power, pros::controller_analog_e_t turn, pros::Controller mast, float t)
{
    up = mast.get_analog(power);
    down = mast.get_analog(turn);
 
    fwd = (exp(-t / 10) + exp((fabs(up) - 127) / 10) * (1 - exp(-t / 10))) * up;
    turning = -1*down;
    if(pto.pto_enable){
        chassis.left_motors[0] = (fwd - turning);
        chassis.left_motors[1] = (fwd - turning);
        chassis.right_motors[0] = (fwd + turning);
        chassis.right_motors[1] = (fwd + turning);
        
    } else{
        left_motors = (fwd - turning);
        right_motors = (fwd + turning);
    }
}

void opcontrol() {
  // This is preference to what you like to drive on.
  chassis.set_drive_brake(MOTOR_BRAKE_COAST);
  left_wing.set_value(false);
  while (true)
  {    // calls the arcade drive function


    arcadeCurv(pros::E_CONTROLLER_ANALOG_LEFT_Y, pros::E_CONTROLLER_ANALOG_RIGHT_X, master, 10);

    // intake
    if (master.get_digital(DIGITAL_L1)) // intake
    {
        intake = 120;
    }
    if (master.get_digital(DIGITAL_L2)) // outtake
    {
        intake = -120;
    }
    if (master.get_digital(DIGITAL_L1) == false && master.get_digital(DIGITAL_L2) == false) // stop intake
    {
        intake = 0;
    }

    // wing
    if (master.get_digital(DIGITAL_R1)) // wing retract
    {
        left_wing.set_value(true);
        right_wing.set_value(true);
    }
    else if (!master.get_digital(DIGITAL_R1)) // wing expand
    {
        right_wing.set_value(false);
        left_wing.set_value(false);
    }

    // catapult
    if (master.get_digital(DIGITAL_B)) // continuous launch
    {
        cata.is_continuous = !cata.is_continuous;
        pros::delay(500);
    }


    // hook mechanism
    if (master.get_digital(DIGITAL_X))
    {
        hooker = !hooker;
        pros::delay(200);
    }
    if (hooker)
    {
        hook.set_value(true);
    }
    if (!hooker)
    {
        hook.set_value(false);
    }

    // pto lift mechanism
    if(master.get_digital(DIGITAL_UP)){
        lifted = !lifted;
        pros::Task engagePto  {[=] { // creates a lambda task for the pto
            pto.set_pto(lifted);
        }};
        pros::delay(500);
    }

    
    // sled/wedge mechanism
    if (master.get_digital(DIGITAL_DOWN))
    {
        wedge = !wedge;
        pros::delay(200);
    }
    if (wedge)
    {
        sled.set_value(true);
    }
    if (!wedge)
    {
        sled.set_value(false);
    }

    pros::delay(20);
  }
}
