#include "main.h"
#include <cmath> 

const int JOYSTICK_DEADBAND = 10;  // you can tweak this

int applyDeadband(int input) {
  if (std::abs(input) < JOYSTICK_DEADBAND) {
    return 0;
  }
  return input;
}

int applyCurve(int input) {
  // input: -127..127
  double x = input / 127.0;          // -1..1
  double a = 0.85;                    // curve strength (0 = straight, 1 = very curvy)
  double y = a * x * x * x + (1 - a) * x;  // cubic mix

  int out = static_cast<int>(y * 127.0);
  // safety clamp
  if (out > 127) out = 127;
  if (out < -127) out = -127;
  return out;
}

int slewStep(int target, int current, int step) {
  if (target > current + step) return current + step;
  if (target < current - step) return current - step;
  return target;
}

ez::Drive chassis(
    {-1, -2, 3},   // Left Chassis Ports  
    {-17, 9, 10},    // Right Chassis Ports
    16,      // IMU Port
    3.25,   // Wheel Diameter
    480     // Wheel RPM = cartridge * (motor gear / wheel gear)
);

ez::tracking_wheel vert_tracker(8, 2.0, -0.39);

void initialize() {
  pros::delay(500);  
  // Configure chassis controls
  chassis.opcontrol_curve_buttons_toggle(false);   // Curve buttons on controller
  chassis.opcontrol_drive_activebrake_set(0);   // Active brake kP (0 disables)
  chassis.opcontrol_curve_default_set(4.5,0);       // Default curve settings
  chassis.odom_tracker_left_set(&vert_tracker);
 


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
  

  ez::as::auton_selector.autons_add({
    {"ODOM TEST", odomtest},
    {"FAST BLUE RIGHT\n\nRAPIDO AZUL DERECHA", fastbluerightauton},
    {"RED LEFT\n\nROJO IZQUIERDA", redleftauton},
    {"BLUE LEFT\n\n AZUL IZQUIERDA", blueleftauton},
    {"BLUE SOLO AWP\n\nAZUL SOLO AWP", bluesoloawp},
    {"RED SOLO AWP\n\nROJO SOLO AWP", redsoloawp},
    {"MEASURE OFFSETS", measure_offsets},
  });

  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
  // Make sure PID tuner is off
  if (chassis.pid_tuner_enabled())
    chassis.pid_tuner_disable();
}

void disabled() {
}

void competition_initialize() {
  // Optional: pre-auton stuff (e.g. side selector on screen)
}

void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (pros::competition::is_connected()) {
  if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
    if (ez::as::page_blank_is_on(0)) {
      ez::screen_print(
        "x: " + util::to_string_with_precision(chassis.odom_x_get()) +
        "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
        "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
        1
      );

      screen_print_tracker(chassis.odom_tracker_left, "l", 4);
      screen_print_tracker(chassis.odom_tracker_right, "r", 5);
      screen_print_tracker(chassis.odom_tracker_back, "b", 6);
      screen_print_tracker(chassis.odom_tracker_front, "f", 7);
    }
  } else {
    pros::lcd::print(0, "Odom disabled or PID tuner on");
    pros::lcd::print(1, "odom_enabled=%d tuner=%d",
      chassis.odom_enabled(),
      chassis.pid_tuner_enabled()
    );
  }
}
    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }
    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

void autonomous() {
  // Reset everything for consistent auton
  chassis.pid_targets_reset();             
  chassis.drive_imu_reset();               
  chassis.drive_sensor_reset();            
  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // Start odom at (0,0,0)
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Hold brake for consistency
  
  // Run the selected auton from the selector
  ez::as::auton_selector.selected_auton_call();
}
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

void opcontrol() {
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  chassis.opcontrol_joystick_practicemode_toggle(false);

  // Drive direction toggle (your existing flipper)
  bool flipped = false;

  // NEW: drive mode toggle
  bool useEzArcade = false;   // false = your custom tank, true = EZ split arcade

  int leftOut = 0;
  int rightOut = 0;

  const int SLEW_STEP_UP = 8;
  const int SLEW_STEP_DOWN = 18;

  while (true) {
    ez_template_extras();

    // Toggle drive direction (your existing)
    if (master.get_digital_new_press(DIGITAL_LEFT)) {
      flipped = !flipped;
      pros::lcd::print(0, flipped ? "Drive: REVERSED" : "Drive: NORMAL");
    }

    // NEW: toggle drive mode
    if (master.get_digital_new_press(DIGITAL_X)) {
      useEzArcade = !useEzArcade;

      // reset slew outputs so it doesn't "jump" when switching modes
      leftOut = 0;
      rightOut = 0;

      pros::lcd::print(1, useEzArcade ? "Drive Mode: EZ SPLIT ARCADE" : "Drive Mode: CUSTOM TANK");
    }

    // =========================
    // DRIVE CONTROL (two modes)
    // =========================
    if (!useEzArcade) {
      // ----- YOUR CUSTOM TANK -----
      int left  = master.get_analog(ANALOG_LEFT_Y);
      int right = master.get_analog(ANALOG_RIGHT_Y);

      left  = applyDeadband(left);
      right = applyDeadband(right);
      left  = applyCurve(left);
      right = applyCurve(right);

      if (flipped) {
        int temp = left;
        left  = -right;
        right = -temp;
      }

      if (std::abs(left - right) < 5) {
        int avg = (left + right) / 2;
        left = avg;
        right = avg;
      }

      int leftTarget  = left;
      int rightTarget = right;

      int leftStep  = (std::abs(leftTarget)  > std::abs(leftOut))  ? SLEW_STEP_UP : SLEW_STEP_DOWN;
      int rightStep = (std::abs(rightTarget) > std::abs(rightOut)) ? SLEW_STEP_UP : SLEW_STEP_DOWN;

      leftOut  = slewStep(leftTarget, leftOut, leftStep);
      rightOut = slewStep(rightTarget, rightOut, rightStep);

      chassis.drive_set(leftOut, rightOut);
    } else {
      // ----- EZ-TEMPLATE SPLIT ARCADE -----
      // IMPORTANT: this reads the controller internally, so don't also do your own drive_set.
      chassis.opcontrol_arcade_standard(ez::SPLIT);

      // If you want the same “flipped” behavior in arcade mode too, use:
      // chassis.opcontrol_drive_reverse_set(flipped);
      // (Put that once per loop, outside the if/else.)
    }

    // =========================
    // INTAKE + OTHER CONTROLS
    // =========================
    if (master.get_digital(DIGITAL_Y)) {
      intake.move(100);
    }
    else if (master.get_digital(DIGITAL_R1)) {
      intake.move(127);
    }
    else if (master.get_digital(DIGITAL_R2)) {
      intake.move(-127);
    }
    else if (master.get_digital(DIGITAL_L2)) {
      intake.move(127);
    }
    else {
      intake.move(0);
    }

    hood.set(master.get_digital(DIGITAL_R1) || master.get_digital(DIGITAL_L2));
    goal.set(master.get_digital(DIGITAL_L2));
    dex.button_toggle(master.get_digital(DIGITAL_UP));
    wing.set(master.get_digital(DIGITAL_L1));


    pros::delay(ez::util::DELAY_TIME);
  }
}
