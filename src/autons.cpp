#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
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

void rightauton() {
chassis.pid_drive_set(8.5_in, 80, true);
chassis.pid_wait_quick_chain();

chassis.pid_turn_set(55_deg, 80);
chassis.pid_wait_quick_chain();

chassis.pid_drive_set(19_in, 25, true);
intake.move(-127);
chassis.pid_wait_quick_chain();

chassis.pid_turn_set(135_deg, 80);
chassis.pid_wait_quick_chain(); 

chassis.pid_drive_set(21_in, 70, true);
intake.move(0);
chassis.pid_wait_quick_chain();

chassis.pid_turn_set(180_deg, 80);
dex.set(true);
chassis.pid_wait_quick_chain();


chassis.pid_drive_set(-8_in, 70, true);
chassis.pid_wait_quick_chain();

intake.move(-127);
score.move(-127);
pros::delay(1750);
score.move(0);

chassis.pid_drive_set(25_in, 80, true);
chassis.pid_wait_quick_chain();

chassis.pid_drive_set(5_in, 20, true);
chassis.pid_wait_quick_chain();

pros::delay(650);
intake.move(0);

chassis.pid_drive_set(-7_in, 50, true);
chassis.pid_wait_quick_chain();

chassis.pid_turn_set(-45_deg, 80, ez::cw);
chassis.pid_wait_quick_chain();

dex.set(false);

chassis.pid_drive_set(47_in, 80, true);
chassis.pid_wait_quick_chain();

intake.move(127);
score.move(127);
pros::delay(800);

chassis.pid_drive_set(-24_in, 80, true);
intake.move(0);
score.move(0);
chassis.pid_wait_quick_chain();

chassis.pid_turn_set(0_deg, 80);
chassis.pid_wait_quick_chain();

chassis.pid_drive_set(18_in, 100, true);
chassis.pid_wait();

chassis.pid_turn_set(-50_deg, 100);
chassis.pid_wait_quick_chain();
}

void leftauton() {
  chassis.pid_drive_set(8.5_in, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-55_deg, 80);   // MIRRORED
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(19_in, 25, true);
  intake.move(-127);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-135_deg, 80);  // MIRRORED
  chassis.pid_wait_quick_chain(); 

  chassis.pid_drive_set(21_in, 70, true);
  intake.move(0);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(180_deg, 80);   // same (backwards)
  dex.set(true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-8_in, 70, true);
  chassis.pid_wait_quick_chain();

  intake.move(-127);
  score.move(-127);
  pros::delay(1750);
  score.move(0);

  chassis.pid_drive_set(25_in, 80, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(5_in, 20, true);
  chassis.pid_wait_quick_chain();

  pros::delay(650);
  intake.move(0);

  chassis.pid_drive_set(-7_in, 50, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-135_deg, 80);   // MIRRORED (was -45)
  chassis.pid_wait_quick_chain();

  dex.set(false);

  chassis.pid_drive_set(-50_in, 80, true);
  chassis.pid_wait_quick_chain();

  intake.move(-127);
  middle.set(true);
  pros::delay(800);

  chassis.pid_drive_set(27_in, 80, true);
  intake.move(0);
  middle.set(false);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(180_deg, 80);  // same direction for facing forward
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-18_in, 100, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-130_deg, 100);  // MIRRORED (was -50)
  chassis.pid_wait_quick_chain();
}

