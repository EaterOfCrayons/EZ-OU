#include "main.h"


/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////




void terminal_print()
{
    // left_front_motor.set_encoder_units(E_MOTOR_ENCODER_DEGREES);
    while (true)
    {

        float voltage_float = (fabs(chassis.left_velocity()) + fabs(chassis.right_velocity())) / 2; // gets and stores the current power output in mV of the motors in float variables
        float position_float = (fabs(chassis.left_sensor()) + fabs(chassis.right_sensor())) / 2;
        std::string voltage_str = std::to_string(voltage_float); // converts the float variables to strings
        std::string position_str = std::to_string(position_float);
        std::cout << voltage_str << "," << position_str << "\n";
        pros::delay(50);
    }
}


void initialize() {
  // Print our branding over your terminal :D
  ez::print_ez_template();
  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure.

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(9, 9); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults(); // Set the exit conditions to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
    Auton("Right Auton", right_auton),
    Auton("Example Turn\n\nTurn 3 times.", turn_example),
    Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
    Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
    Auton("Swing Example\n\nSwing, drive, swing.", swing_example),
    Auton("Combine all 3 movements", combining_movements),
    Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}


void competition_initialize() {

}



void autonomous() {

  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.
  
  pros::Task data_logging(terminal_print);
  ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
}




void opcontrol() {
  // This is preference to what you like to drive on.
  chassis.set_drive_brake(MOTOR_BRAKE_COAST);
  int wing_toggle = -1;
  while (true) {
    chassis.arcade_standard(ez::SPLIT); // Standard split arcade

    // intake
    if (master.get_digital(DIGITAL_L1))
    {
        intakes = 120;
    }
    if (master.get_digital(DIGITAL_L2))
    {
        intakes = -120;
    }
    if (master.get_digital(DIGITAL_L1) == false && master.get_digital(DIGITAL_L2) == false)
    {
        intakes = 0;
    }

    // wing
    if (master.get_digital(DIGITAL_A))
    {
      wing_toggle = wing_toggle * -1;
      pros::delay(200);
    }
    if (wing_toggle == 1){
      wing_left.set_value(true);
      wing_right.set_value(true);
    }
    if (wing_toggle == -1) {
      wing_left.set_value(false);
      wing_right.set_value(false);
    }

    
    // hook
    if (master.get_digital(DIGITAL_B)) {
      pto_hook(!pto_hook_enabled);
      pros::delay(500);
    }

    if (pto_hook_enabled){
      if (master.get_digital(DIGITAL_UP)){
        left_pto_1 = 40;
        left_pto_2 = 40;
      } else if (master.get_digital(DIGITAL_DOWN)){
        left_pto_1 = -40;
        left_pto_2 = -40;
      } else
        left_pto_1 = 0;
        left_pto_2 = 0;
    }
    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
