#include "main.h"
const int DRIVE_SPEED = 127;
const int TURN_SPEED = 100;
const int SWING_SPEED = 100;

void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
  chassis.pid_swing_behavior_set(ez::shortest);
  chassis.slew_drive_set(true);
  chassis.slew_swing_set(true);  // Enables global slew
  chassis.slew_swing_constants_set(5_deg, 50);
}


void fastredrightauton() {
keepRed = true;
chassis.pid_drive_set(5_in,DRIVE_SPEED);
intake.move(127);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(340_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(11.4_in,DRIVE_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(90_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(20_in,60);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(135_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(22_in,DRIVE_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(180_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-9_in,DRIVE_SPEED);
chassis.pid_wait_quick_chain();
dex.set(true);
score_auto(127,127, 1250);
chassis.drive_angle_set(180_deg);
chassis.pid_drive_set(27_in, 75);
chassis.pid_wait_quick_chain();
score_auto(127, 0, 850);
intake.move(127);
chassis.pid_drive_set(-28_in,DRIVE_SPEED);
chassis.pid_wait_quick_chain();
score_auto(127,127, 1500);
dex.set(false);
chassis.pid_drive_set(8_in,DRIVE_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(90_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-6_in,DRIVE_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(180_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-26_in,85);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(140_deg, 40);
chassis.pid_wait_quick_chain();
}


void redleftauton() {
keepRed = true;
chassis.pid_drive_set(5_in, DRIVE_SPEED);
intake.move(127);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(20_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(11.4_in, DRIVE_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(270_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(20_in, 60);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(225_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(20_in, DRIVE_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(180_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-9_in, DRIVE_SPEED);
chassis.pid_wait_quick_chain();
dex.set(true);
score_auto(127, 127, 1300);
chassis.drive_angle_set(180_deg);
chassis.pid_drive_set(27_in, 75);
chassis.pid_wait_quick_chain();
score_auto(127, 0, 800);
intake.move(127);
chassis.pid_drive_set(-28_in, DRIVE_SPEED);
chassis.pid_wait_quick_chain();
score_auto(127, 127, 1350);
dex.set(false);
chassis.pid_drive_set(8_in, DRIVE_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(90_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(6_in, DRIVE_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(0_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(30_in, 80);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(50_deg, 40);
chassis.pid_wait_quick_chain();
}

void redsoloawp() {
keepRed = true;
chassis.pid_drive_set(5_in, DRIVE_SPEED);
intake.move(127);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(20_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(11.4_in, DRIVE_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(270_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(20_in, 60);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(225_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(20_in, DRIVE_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(180_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-9_in, DRIVE_SPEED);
chassis.pid_wait_quick_chain();
score_auto(127, 127, 1300);
chassis.pid_drive_set(5_in, DRIVE_SPEED);
intake.move(-127);
score.move(127);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(77_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
intake.move(127);
chassis.pid_drive_set(65_in, DRIVE_SPEED);
chassis.pid_wait_quick_chain();
score.move(0);
chassis.pid_drive_set(10_in, 50);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-.5_in, 60);

chassis.pid_wait_quick_chain();
chassis.pid_turn_set(315_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(9_in, DRIVE_SPEED);
chassis.pid_wait_quick_chain();
score_auto(-127,-127, 1250);
intake.move(-127);
chassis.pid_drive_set(-38.75_in, DRIVE_SPEED);
pros::delay(500);
dex.set(true);

chassis.pid_wait_quick_chain();
chassis.pid_turn_set(180_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
intake.move(127);
chassis.pid_drive_set(10.5_in, 75);
chassis.pid_wait_quick_chain();
score_auto(127, 0, 700);
intake.move(127);
chassis.pid_drive_set(-27_in,DRIVE_SPEED);
chassis.pid_wait_quick_chain();
score_auto(127,127, 2000);
}

void fastbluerightauton() {
keepRed = false;
chassis.pid_drive_set(5_in,DRIVE_SPEED);
intake.move(127);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(340_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(11.4_in,DRIVE_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(90_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(20_in,60);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(135_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(22_in,DRIVE_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(180_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-9_in,DRIVE_SPEED);
chassis.pid_wait_quick_chain();
dex.set(true);
score_auto(127,127, 1250);
chassis.drive_angle_set(180_deg);
chassis.pid_drive_set(27_in, 75);
chassis.pid_wait_quick_chain();
score_auto(127, 0, 850);
intake.move(127);
chassis.pid_drive_set(-28_in,DRIVE_SPEED);
chassis.pid_wait_quick_chain();
score_auto(127,127, 1500);
dex.set(false);
chassis.pid_drive_set(8_in,DRIVE_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(90_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-6_in,DRIVE_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(180_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-26_in,85);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(140_deg, 40);
chassis.pid_wait_quick_chain();
}

void blueleftauton() {
keepRed = false;
chassis.pid_drive_set(5_in, DRIVE_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(20_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(11.4_in, DRIVE_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(270_deg, TURN_SPEED);
intake.move(127);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(20_in, 60);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(225_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(20_in, DRIVE_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(180_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-9_in, DRIVE_SPEED);
chassis.pid_wait_quick_chain();
dex.set(true);
score_auto(127, 127, 1300);
chassis.drive_angle_set(180_deg);
chassis.pid_drive_set(27_in, 75);
chassis.pid_wait_quick_chain();
score_auto(127, 0, 800);
intake.move(127);
chassis.pid_drive_set(-28_in, DRIVE_SPEED);
chassis.pid_wait_quick_chain();
score_auto(127, 127, 1350);
dex.set(false);
chassis.pid_drive_set(8_in, DRIVE_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(90_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(6_in, DRIVE_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(0_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(30_in, 80);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(50_deg, 40);
chassis.pid_wait_quick_chain();
}

void bluesoloawp() {
keepRed = false;
chassis.pid_drive_set(5_in, DRIVE_SPEED);
intake.move(127);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(20_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(11.4_in, DRIVE_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(270_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(20_in, 60);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(225_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(20_in, DRIVE_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(180_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-8_in, DRIVE_SPEED);
chassis.pid_wait_quick_chain();
score_auto(127, 127, 1300);
chassis.pid_drive_set(5_in, DRIVE_SPEED);
intake.move(-127);
score.move(127);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(77_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
intake.move(127);
chassis.pid_drive_set(65_in, DRIVE_SPEED);
chassis.pid_wait_quick_chain();
score.move(0);
chassis.pid_drive_set(10_in, 57);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(-1.3_in, 60);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(315_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
chassis.pid_drive_set(9_in, DRIVE_SPEED);
chassis.pid_wait_quick_chain();
score_auto(-127,-127, 1250);
intake.move(-127);
chassis.pid_drive_set(-41.5_in, DRIVE_SPEED);
pros::delay(500);
dex.set(true);
chassis.pid_wait_quick_chain();
chassis.pid_turn_set(180_deg, TURN_SPEED);
chassis.pid_wait_quick_chain();
intake.move(127);
chassis.pid_drive_set(9.5_in, 75);
chassis.pid_wait_quick_chain();
score_auto(127, 0, 700);
intake.move(127);
chassis.pid_drive_set(-27_in,DRIVE_SPEED);
chassis.pid_wait_quick_chain();
score_auto(127,127, 2000);
}

void bluerightauton() {
keepRed = false;
 
}
void redrightauton() {
keepRed = true;
}





void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

