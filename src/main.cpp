#include "main.h"
#define DIGITAL_SENSOR_PORT 'A'
#define DIGITAL_SENSOR_PORT_B 'B'
#define DIGITAL_SENSOR_PORT_C 'C'

/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////


// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {-10, -20}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{3, 11}

  // IMU Port
  ,8

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,3.25

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,200

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,0.6

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
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::print_ez_template();
  
  pros::delay(500); // Stop the user from doing anything while legacy ports configure.

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0.1); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(0, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)  
  default_constants(); // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults(); // Set the exit conditions to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used. 
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons({
    Auton("Far Side", far_side),
    Auton("Long Ball Rush", long_ball_rush),
    Auton("TEST", test_example),
    /*Auton("Example Drive\n\nDrive forward and come back.", drive_example),
    Auton("Example Turn\n\nTurn 3 times.", turn_example),
    Auton("Drive and Turn\n\nDrive forward, turn, come back. ", drive_and_turn),
    Auton("Drive and Turn\n\nSlow down during drive.", wait_until_change_speed),
    Auton("Swing Example\n\nSwing, drive, swing.", swing_example),
    Auton("Combine all 3 movements", combining_movements),
    Auton("Interference\n\nAfter driving forward, robot performs differently if interfered or not.", interfered_example),*/
 // initialize brain screen
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  pros::lcd::initialize();
}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}



/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}



/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.
}



/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  // This is preference to what you like to drive on.
  chassis.set_drive_brake(MOTOR_BRAKE_COAST);

  pros::Controller master(pros::E_CONTROLLER_MASTER);

  pros::Motor intake(7, pros::E_MOTOR_GEARSET_06, false);

	pros::Motor flywheelLeft(9, pros::E_MOTOR_GEARSET_36, true);
  pros::Motor flywheelRight(19, pros::E_MOTOR_GEARSET_36, false);
  pros::MotorGroup flywheel({flywheelLeft, flywheelRight});

	pros::Motor leftHang(6, pros::E_MOTOR_GEARSET_36, false);
	pros::Motor rightHang(5, pros::E_MOTOR_GEARSET_36, true);
	pros::MotorGroup hang({leftHang,rightHang});
	hang.set_brake_modes(MOTOR_BRAKE_HOLD);

	pros::Motor hook(19, pros::E_MOTOR_GEARSET_18, false);
	pros::ADIDigitalOut piston(DIGITAL_SENSOR_PORT);
  pros::ADIDigitalOut wings(DIGITAL_SENSOR_PORT_C);
	hook.set_brake_mode(MOTOR_BRAKE_HOLD);

  pros::ADIDigitalIn limit_switch(DIGITAL_SENSOR_PORT_B);
  bool down = false;
  bool forward = true;
  bool out = false; // piston
  bool wingsOut = false;

  while (true) {
    chassis.arcade_standard(ez::SPLIT); // Standard split arcade
    // chassis.arcade_standard(ez::SINGLE); // Standard single arcade
    // chassis.arcade_flipped(ez::SPLIT); // Flipped split arcade
    // chassis.arcade_flipped(ez::SINGLE); // Flipped single arcade
    pros::lcd::print(1, "Value " + limit_switch.get_value());
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
      while (!(limit_switch.get_value())){
        piston.set_value(true);
        hang.move(100);
      }
      while (limit_switch.get_value()){
        hang.move(0);
        piston.set_value(false);
      }
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
    { 
      forward = !forward;
      if (!forward) {
        intake.move(50);
      }
      else{
        intake.move(-127);
      }
    }
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
    {
      intake.move(0);
    }
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			hang.move_velocity(300);
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			hang.move_velocity(-300);
		}
		else{
			hang.move_velocity(0);
		}


		//flywheel
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			flywheel.move(127);
		}
		else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			flywheel.move(0);
		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
		{
      down = !down;
			piston.set_value(down);
		}

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
    {
      wingsOut = !wingsOut;
      wings.set_value(wingsOut);
    }

    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
