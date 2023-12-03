#include "main.h"
#include "robot_config.hpp"
#include "autons.hpp"

const int DRIVE_SPEED = 110; // This is 110/127 (around 87% of max speed).  We don't suggest making this 127.
                             // If this is 127 and the robot tries to heading correct, it's only correcting by
                             // making one side slower.  When this is 87%, it's correcting by making one side
                             // faster and one side slower, giving better heading correction.
const int TURN_SPEED = 110;
const int SWING_SPEED = 100;

///
// ANCHOR Constants
///

void push_constants()
{
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 12, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.7, 0, 4, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.7, 0, 4, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 47, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 50, 0);
}

void default_constants()
{
  chassis.set_slew_min_power(80, 80);
  chassis.set_slew_distance(7, 7);
  chassis.set_pid_constants(&chassis.headingPID, 12, 0, 20, 0);
  chassis.set_pid_constants(&chassis.forward_drivePID, 0.45, 0, 4, 0);
  chassis.set_pid_constants(&chassis.backward_drivePID, 0.45, 0, 4, 0);
  chassis.set_pid_constants(&chassis.turnPID, 5, 0.003, 47, 15);
  chassis.set_pid_constants(&chassis.swingPID, 7, 0, 50, 0);
}

void exit_condition_defaults()
{
  chassis.set_exit_condition(chassis.turn_exit, 35, 1, 150, 3, 250, 500);
  chassis.set_exit_condition(chassis.swing_exit, 50, 2, 250, 5, 250, 500);
  chassis.set_exit_condition(chassis.drive_exit, 35, 30, 150, 150, 250, 500);
}

///
// Drive Example
///
void drive_example()
{
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater then the slew distance + a few inches

  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();
  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();
  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();
  std::cout << "process has finished"
            << "\n";
}

///
// Turn Example
///
void turn_example()
{
  // The first parameter is target degrees
  // The second parameter is max speed the robot will drive at

  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();
}

///
// Combining Turn + Drive
///
void drive_and_turn()
{
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed()
{
  // wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches, the robot will travel the remaining distance at a max speed of 40
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_until(6);
  chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 40 speed
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(-45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  // When the robot gets to -6 inches, the robot will travel the remaining distance at a max speed of 40
  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_until(-6);
  chassis.set_max_speed(40); // After driving 6 inches at DRIVE_SPEED, the robot will go the remaining distance at 40 speed
  chassis.wait_drive();
}

///
// Swing Example
///
void swing_example()
{
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is target degrees
  // The third parameter is speed of the moving side of the drive

  chassis.set_swing_pid(ez::LEFT_SWING, 45, SWING_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_until(12);

  chassis.set_swing_pid(ez::RIGHT_SWING, 0, SWING_SPEED);
  chassis.wait_drive();
}

///
// Auto that tests everything
///
void combining_movements()
{
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  chassis.set_turn_pid(45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_swing_pid(ez::RIGHT_SWING, -45, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_turn_pid(0, TURN_SPEED);
  chassis.wait_drive();

  chassis.set_drive_pid(-24, DRIVE_SPEED, true);
  chassis.wait_drive();
}

///
// Interference example
///
void tug(int attempts)
{
  for (int i = 0; i < attempts - 1; i++)
  {
    // Attempt to drive backwards
    printf("i - %i", i);
    chassis.set_drive_pid(-12, 127);
    chassis.wait_drive();

    // If failsafed...
    if (chassis.interfered)
    {
      chassis.reset_drive_sensor();
      chassis.set_drive_pid(-2, 20);
      pros::delay(1000);
    }
    // If robot successfully drove back, return
    else
    {
      return;
    }
  }
}

// If there is no interference, robot will drive forward and turn 90 degrees.
// If interfered, robot will drive forward and then attempt to drive backwards.
void interfered_example()
{
  chassis.set_drive_pid(24, DRIVE_SPEED, true);
  chassis.wait_drive();

  if (chassis.interfered)
  {
    tug(3);
    return;
  }

  chassis.set_turn_pid(90, TURN_SPEED);
  chassis.wait_drive();
}

// ANCHOR right auto
void right_auton()
{
  intake = -127;
  chassis.set_drive_pid(5.5, 100);
  chassis.wait_drive();
  chassis.set_drive_pid(-37, 80);
  chassis.wait_drive();
  chassis.set_turn_pid(180, 120);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::RIGHT_SWING, 135, 120);
  chassis.wait_drive();
  right_wing.set_value(true);
  left_wing.set_value(true);
  chassis.set_drive_pid(13, 120);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::RIGHT_SWING, 90, 120);
  chassis.wait_drive();
  intake = 127;
  chassis.set_drive_pid(17, 127);
  chassis.wait_drive();
  chassis.set_drive_pid(-20, 120);
  chassis.wait_drive();
  right_wing.set_value(false);
  left_wing.set_value(false);
  chassis.set_turn_pid(-70, 120);
  chassis.wait_drive();
  chassis.set_drive_pid(-24, 120);
  chassis.wait_drive();
  chassis.set_drive_pid(21, 120);
  chassis.wait_drive();
  chassis.set_turn_pid(27, 120);
  chassis.wait_drive();
  intake = -127;
  chassis.set_drive_pid(49, 120);
  chassis.wait_drive();
  chassis.set_turn_pid(90, 120);
  chassis.wait_drive();
  chassis.set_drive_pid(20, 120);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::LEFT_SWING, 180, 120);
  chassis.wait_drive();
  left_wing.set_value(true);
  right_wing.set_value(true);
  chassis.set_drive_pid(40, 127);
  chassis.wait_drive();
}

// ANCHOR left auto
void left_auton()
{
}

void skillsStart()
{
  chassis.reset_pid_targets();  // Resets PID targets to 0
  chassis.reset_gyro();         // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD);
  chassis.set_turn_pid(45, 120);
  chassis.wait_drive();
  chassis.set_drive_pid(10, 120);
  chassis.wait_drive();
  chassis.set_turn_pid(-20, 120);
  chassis.wait_drive();
  hook.set_value(true);
  cata.is_continuous = true;
  pros::delay(35000); //****************************
  cata.is_continuous = false;
  hook.set_value(false);
  chassis.set_turn_pid(45, 120);
  chassis.wait_drive();
  chassis.set_drive_pid(-18, 120);
  chassis.wait_drive();
  chassis.set_turn_pid(2, 120);
  chassis.wait_drive();
  chassis.set_drive_brake(MOTOR_BRAKE_COAST);
}

// ANCHOR prog
void skills()
{
  chassis.set_turn_pid(45, 120);
  chassis.wait_drive();
  chassis.set_drive_pid(10, 120);
  chassis.wait_drive();
  chassis.set_turn_pid(160, 120);
  chassis.wait_drive();
  hook.set_value(true);
  cata.is_continuous = true;
  pros::delay(33000);
  cata.is_continuous = false;
  hook.set_value(false);
  chassis.set_turn_pid(45, 120);
  chassis.wait_drive();
  chassis.set_drive_pid(-17, 120);
  chassis.wait_drive();
  chassis.set_turn_pid(2, 120);
  chassis.wait_drive();
  //=======================================================
  chassis.set_drive_pid(-83, 120);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::LEFT_SWING, -45, 120);
  chassis.wait_drive();
  chassis.set_drive_pid(-20, 120);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::LEFT_SWING, -90, 120);
  chassis.wait_drive();
  chassis.mode = ez::DISABLE;
  chassis.set_tank(-127, -127);
  pros::delay(2000);
  chassis.set_tank(0, 0);
  chassis.mode = ez::DRIVE;
  chassis.set_drive_pid(14, 120);
  chassis.wait_drive();
  chassis.set_turn_pid(-80, 120);
  chassis.wait_drive();
  chassis.mode = ez::DISABLE;
  chassis.set_tank(-127, -127);
  pros::delay(2000);
  chassis.set_tank(0, 0);
  chassis.mode = ez::DRIVE;
  chassis.set_drive_pid(13, 120);
  chassis.wait_drive();
  chassis.set_turn_pid(0, 120);
  chassis.wait_drive();
  chassis.set_drive_pid(13, 120);
  chassis.wait_drive();
  chassis.set_turn_pid(50, 120);
  chassis.wait_drive();
  left_wing.set_value(true);
  right_wing.set_value(true);
  chassis.set_drive_pid(20, 120);
  chassis.wait_drive();
  left_wing.set_value(false);
  right_wing.set_value(false);
  chassis.set_turn_pid(0, 120);
  chassis.wait_drive();
  chassis.set_drive_pid(23, 120);
  chassis.wait_drive();
  chassis.set_turn_pid(90, 120);
  chassis.wait_drive();
  chassis.set_drive_pid(30, 120);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::LEFT_SWING, 180, 120);
  chassis.wait_drive();
  left_wing.set_value(true);
  right_wing.set_value(true);
  chassis.mode = ez::DISABLE;
  chassis.set_tank(-127, -127);
  pros::delay(3000);
  chassis.set_tank(0, 0);
  chassis.mode = ez::DRIVE;
  chassis.set_drive_pid(-31, 120);
  chassis.wait_drive();
  left_wing.set_value(false);
  right_wing.set_value(false);
  chassis.set_turn_pid(90, 120);
  chassis.wait_drive();
  chassis.set_drive_pid(-20, 120);
  chassis.wait_drive();
  chassis.set_swing_pid(ez::LEFT_SWING, 180, 120);
  chassis.wait_drive();
  left_wing.set_value(true);
  right_wing.set_value(true);
  chassis.mode = ez::DISABLE;
  chassis.set_tank(-127, -127);
  pros::delay(3000);
  chassis.set_tank(0, 0);
  chassis.mode = ez::DRIVE;
  chassis.set_drive_pid(-30, 120);
  chassis.wait_drive();
  left_wing.set_value(false);
  right_wing.set_value(false);
}

void no_auton()
{
}
