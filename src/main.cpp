#include "main.h"

// ====================
// Global drive object
// ====================
ez::Drive chassis(
    // Drive motors, the first motor is used for sensing!
    {-12, 11, -1},   // Left Chassis Ports  (negative = reversed)
    {9, 19, -20},    // Right Chassis Ports (negative = reversed)

    7,      // IMU Port
    3.25,   // Wheel Diameter (inches)
    480     // Wheel RPM = cartridge * (motor gear / wheel gear)
);

// ====================
// Piston state booleans
// ====================
bool descoreState = false;
bool dexState     = false;
bool middleState  = false;

// ====================
// initialize()
// ====================
void initialize() {
  // Optional EZ-Template banner
  // ez::ez_template_print();

  pros::delay(500);  // Wait for legacy ports, sensors, etc.

  // Configure chassis controls
  chassis.opcontrol_curve_buttons_toggle(false);   // Curve buttons on controller (off)
  chassis.opcontrol_drive_activebrake_set(0);   // Active brake kP (0 disables)
  chassis.opcontrol_curve_default_set(4, 2.5);       // Default curve settings

  // Set drive PID constants from autons.cpp
  default_constants();

  // Left/right curve button bindings (optional)
  chassis.opcontrol_curve_buttons_left_set(
      pros::E_CONTROLLER_DIGITAL_LEFT,
      pros::E_CONTROLLER_DIGITAL_RIGHT
  );
  chassis.opcontrol_curve_buttons_right_set(
      pros::E_CONTROLLER_DIGITAL_Y,
      pros::E_CONTROLLER_DIGITAL_A
  );

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
    {"Right Auton\n\nRight side", rightauton},
    {"Left Auton\n\nLeft side", leftauton},
    
      
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();

  // Rumble: "." if IMU calibrated, "---" if not
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");

  // Make sure PID tuner is off
  if (chassis.pid_tuner_enabled())
    chassis.pid_tuner_disable();

  // Set starting positions for pistons
  descoreState = false;
  dexState     = false;
  middleState  = false;

  descore.set(descoreState);   // start retracted
  dex.set(dexState);           // start extended (or whatever your mech needs)
  middle.set(middleState);     // start retracted
}

// ====================
// disabled()
// ====================
void disabled() {
  // Optional: things to do while disabled
}

// ====================
// competition_initialize()
// ====================
void competition_initialize() {
  // Optional: pre-auton stuff (e.g. side selector on screen)
}

// ====================
// autonomous()
// ====================
void autonomous() {
  // Reset everything for consistent auton
  chassis.pid_targets_reset();             // Reset PID targets
  chassis.drive_imu_reset();               // Reset gyro
  chassis.drive_sensor_reset();            // Reset drive encoders
  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // Start odom at (0,0,0)
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Hold brake for consistency
  // Run the selected auton from the selector
  ez::as::auton_selector.selected_auton_call();
}

// ====================
// ez_template_extras()
// (PID tuner / run auton from driver if wanted)
// ====================
void ez_template_extras() {
  // Only run this when NOT connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // Enable / Disable PID Tuner with X if you want:
    // if (master.get_digital_new_press(DIGITAL_X))
    //   chassis.pid_tuner_toggle();

    // Trigger auton in driver control with B + DOWN if you want:
    // if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
    //   pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
    //   autonomous();
    //   chassis.drive_brake_set(preference);
    // }

    // Let the PID tuner iterate
    // chassis.pid_tuner_iterate();
  }
  // If connected to field, you could disable tuner here if desired.
}

// ====================
// opcontrol()
// ====================
void opcontrol() {
  // Drive setup for driver control
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);           // Coast during driver
  chassis.opcontrol_joystick_practicemode_toggle(false);

  bool flipped = false;  // false = normal, true = reversed

  while (true) {
    ez_template_extras();

    if (master.get_digital_new_press(DIGITAL_LEFT)) {
      flipped = !flipped;
      chassis.opcontrol_drive_reverse_set(flipped);
      pros::lcd::print(0, flipped ? "Drive: REVERSED" : "Drive: NORMAL");
    }
    chassis.opcontrol_tank();

    if (master.get_digital(DIGITAL_RIGHT)) {
      intake.move(127);
      score.move(127);
    } 
    else if (master.get_digital(DIGITAL_Y)) {
      // Intake backward only
      intake.move(-127);
      score.move(0);
    } 
    else if (master.get_digital(DIGITAL_R1)) {
      // Intake + score backward (e.g., descore out)
      intake.move(-127);
      score.move(-127);
    }
    else if (master.get_digital_new_press(DIGITAL_R2)) {
      chassis.drive_angle_set(180_deg); 
      descore.set(false);
      chassis.pid_drive_set(4_in, 80, true);
      chassis.pid_wait_quick_chain();

      chassis.pid_turn_set(90_deg, 80);
      chassis.pid_wait_quick_chain();

      chassis.pid_drive_set(6.5_in, 70, true);
      chassis.pid_wait_quick_chain();

      chassis.pid_turn_set(180_deg, 80);
      chassis.pid_wait_quick_chain();

      chassis.pid_drive_set(-28_in, 127, true);
      chassis.pid_wait_quick_chain();
    }
    else if (master.get_digital_new_press(DIGITAL_L2)) {
      chassis.drive_angle_set(180_deg); 
      descore.set(false);
      chassis.pid_drive_set(4_in, 80, true);
      chassis.pid_wait_quick_chain();

      chassis.pid_turn_set(-90_deg, 80);
      chassis.pid_wait_quick_chain();

      chassis.pid_drive_set(6_in, 70, true);
      chassis.pid_wait_quick_chain();

      chassis.pid_turn_set(0_deg, 80);
      chassis.pid_wait_quick_chain();

      chassis.pid_drive_set(28_in, 127, true);
      chassis.pid_wait_quick_chain();
    }
    else {
      intake.move(0);
      score.move(0);
    }

    if (master.get_digital_new_press(DIGITAL_X)) {
      descoreState = !descoreState;
      descore.set(descoreState);
    }

    if (master.get_digital_new_press(DIGITAL_UP)) {
      dexState = !dexState;
      dex.set(dexState);
    }
    if (master.get_digital_new_press(DIGITAL_L1)) {
      middleState = !middleState;
      middle.set(middleState);
    }
    pros::delay(ez::util::DELAY_TIME);
  }
}