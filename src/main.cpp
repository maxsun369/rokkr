#include "main.h"
#include <cmath>
bool keepRed = true;  
void score_auto(int intakePower, int scorePower, int timeMs);

pros::Optical optical(9);
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
  double a = 0.6;                    // curve strength (0 = straight, 1 = very curvy)
  double y = a * x * x * x + (1 - a) * x;  // cubic mix

  int out = static_cast<int>(y * 127.0);
  // safety clamp
  if (out > 127) out = 127;
  if (out < -127) out = -127;
  return out;
}

ez::Drive chassis(
    {-11, -12, 13},   // Left Chassis Ports  
    {20, 19, -18},    // Right Chassis Ports
    4,      // IMU Port
    2.75,   // Wheel Diameter
    450     // Wheel RPM = cartridge * (motor gear / wheel gear)
);

void initialize() {
  pros::delay(500);  
  optical.set_led_pwm(100);
  // Configure chassis controls
  chassis.opcontrol_curve_buttons_toggle(false);   // Curve buttons on controller
  chassis.opcontrol_drive_activebrake_set(0);   // Active brake kP (0 disables)
  chassis.opcontrol_curve_default_set(4.5,0);       // Default curve settings
 


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
    {"RED SOLO AWP\n\nROJO SOLO AWP", redsoloawp},
    {"RED LEFT\n\nROJO IZQUIERDA", redleftauton},
    {"FAST BLUE RIGHT\n\nRAPIDO AZUL DERECHA", fastbluerightauton},
    {"FAST RED RIGHT Auton\n\nRAPIDO ROJO DERECHA", fastredrightauton},
    {"BLUE LEFT\n\n AZUL IZQUIERDA", blueleftauton},
    {"BLUE SOLO AWP\n\nAZUL SOLO AWP", bluesoloawp},
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

void score_auto(int intakePower, int scorePower, int timeMs) {
    int start = pros::millis();
    while (pros::millis() - start < timeMs) {

        int sortedScore = color_sort_adjust(scorePower);

        intake.move(intakePower);
        score.move(sortedScore);

        pros::delay(20);
    }

    // stop after action
    intake.move(0);
    score.move(0);
}

void autonomous() {
  // Reset everything for consistent auton
  chassis.pid_targets_reset();             
  chassis.drive_imu_reset();               
  chassis.drive_sensor_reset();            
  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // Start odom at (0,0,0)
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD); // Hold brake for consistency
  score.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  
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
// true = we are keeping RED balls, ejecting BLUE

int color_sort_adjust(int manualPower) {
  // If you're not moving the intake, don't do anything
  if (manualPower == 0) return manualPower;

  int prox = optical.get_proximity();   // 0–255, higher = closer
  if (prox < 50) {
    // Nothing close -> don't change anything
    return manualPower;
  }

  double hue = optical.get_hue();  // 0–360

  bool isRed  = (hue < 30) || (hue > 330);
  bool isBlue = (hue > 180 && hue < 260);

  // We only care when you're trying to take a ball IN (manualPower > 0)
  if (manualPower > 0) {
    if (keepRed) {
      // We want RED. If we see BLUE, override to spit it out.
      if (isBlue) {
        return -127;  // Eject wrong color
      }
    } else {
      // We want BLUE. If we see RED, override to spit it out.
      if (isRed) {
        return -127;
      }
    }
  }

  // Otherwise, keep your command
  return manualPower;
}

void opcontrol() {
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);    
  score.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);     
  chassis.opcontrol_joystick_practicemode_toggle(false);

  bool flipped = false;

  while (true) {
    ez_template_extras();

    // CHASSIS CONTROL
    if (master.get_digital_new_press(DIGITAL_LEFT)) {
      flipped = !flipped;
      pros::lcd::print(0, flipped ? "Drive: REVERSED" : "Drive: NORMAL");
    }
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
    chassis.drive_set(left, right);

    // INTAKE + SCORING CONTROL WITH COLOR SORTER
    int manualIntake = 0;
    int manualScore  = 0;

    if (master.get_digital(DIGITAL_RIGHT)) {
      manualIntake = 127;    // eject
      manualScore  = -127;
    } 
    else if (master.get_digital(DIGITAL_Y)) {
      manualIntake = 127;   // load intake
      manualScore  = 0;
    } 
    else if (master.get_digital(DIGITAL_R1)) {
      manualIntake = 127;    // score long goal
      manualScore  = 127;
    }
    else if (master.get_digital(DIGITAL_R2)) {
      manualIntake = -127;   // score bottom goal
      manualScore  = -127;
    }

    int finalScore = color_sort_adjust(manualScore);
    if (finalScore != manualScore) {
      master.rumble(".");
    }

    intake.move(manualIntake);
    score.move(finalScore);

    descore.button_toggle(master.get_digital(DIGITAL_L1));
    dex.button_toggle(master.get_digital(DIGITAL_UP));
    middle.set(master.get_digital(DIGITAL_R2));

    pros::delay(ez::util::DELAY_TIME);
  }
}
