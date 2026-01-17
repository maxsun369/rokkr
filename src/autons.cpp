#include "main.h"
#include <cmath>

// ---------- Helpers ----------
constexpr double FIELD_MIN = 0.0;
constexpr double FIELD_MAX = 144.0; // inches (12 ft field)

inline double deg_to_rad(double d) { return d * M_PI / 180.0; }
inline double mm_to_in(double mm) { return mm / 25.4; }

double read_dist_in(pros::Distance& d) {
  double mm = d.get();          // PROS Distance is mm
  if (mm <= 0) return -1;       // invalid
  return mm_to_in(mm);
}

// Ray from (ox,oy) along unit vector (vx,vy) to field walls; returns inches
double ray_to_field(double ox, double oy, double vx, double vy) {
  const double EPS = 1e-6;
  double best = 1e9;

  if (std::abs(vx) > EPS) {
    double t = (FIELD_MIN - ox) / vx;
    double y = oy + t * vy;
    if (t > 0 && y >= FIELD_MIN && y <= FIELD_MAX) best = std::min(best, t);

    t = (FIELD_MAX - ox) / vx;
    y = oy + t * vy;
    if (t > 0 && y >= FIELD_MIN && y <= FIELD_MAX) best = std::min(best, t);
  }
  if (std::abs(vy) > EPS) {
    double t = (FIELD_MIN - oy) / vy;
    double x = ox + t * vx;
    if (t > 0 && x >= FIELD_MIN && x <= FIELD_MAX) best = std::min(best, t);

    t = (FIELD_MAX - oy) / vy;
    x = ox + t * vx;
    if (t > 0 && x >= FIELD_MIN && x <= FIELD_MAX) best = std::min(best, t);
  }

  if (best > 1e8) return -1;
  return best;
}

// ---------- Sensor model ----------
struct DistModel {
  pros::Distance* s;
  const char* name;
  double dx;      // inches: +forward
  double dy;      // inches: +right
  double dir_deg; // 0=front, 90=right, 180=back, 270=left
};

// YOUR INITIAL GUESSES (edit these)
DistModel F{&distFront, "Front",  5.0,  0.0,   0.0};
DistModel R{&distRight, "Right",  0.0,  5.0,  90.0};
DistModel B{&distBack,  "Back",  -5.0,  0.0, 180.0};
DistModel L{&distLeft,  "Left",   0.0, -5.0, 270.0};

DistModel* allS[] = {&F, &R, &B, &L};

// Predict distance for a sensor using current odom pose
double predict_dist_in(const DistModel& m, double x, double y, double th_deg) {
  double th = deg_to_rad(th_deg);

  // Sensor global position
  double sx = x + (m.dx * std::sin(th) + m.dy * std::cos(th));
  double sy = y + (m.dx * std::cos(th) - m.dy * std::sin(th));

  // Sensor global direction
  double gdeg = th_deg + m.dir_deg;
  double gr = deg_to_rad(gdeg);

  // EZ convention: 0deg faces +Y, so unit vector is (sin, cos)
  double vx = std::sin(gr);
  double vy = std::cos(gr);

  return ray_to_field(sx, sy, vx, vy);
}

// Average multiple readings to reduce noise
double avg_read_in(pros::Distance& d, int samples = 25, int delay_ms = 10) {
  double sum = 0;
  int cnt = 0;
  for (int i = 0; i < samples; i++) {
    double v = read_dist_in(d);
    if (v > 0) { sum += v; cnt++; }
    pros::delay(delay_ms);
  }
  if (cnt == 0) return -1;
  return sum / cnt;
}

// ---------- "Distance Calibrate" routine ----------
// Put this in your autons.cpp (or any .cpp that can access distFront/distRight/distBack/distLeft and master).
// Then add it to your auton selector like: {"DIST CALIBRATE", distance_calibrate},

void distance_calibrate() {
  // ====== SETTINGS YOU CAN CHANGE ======
  const double TRUE_DIST_IN = 12.0;   // Use a ruler/tape: place each sensor exactly this far from a wall
  const int SAMPLES = 25;             // More = smoother average
  const int SAMPLE_DELAY_MS = 15;

  // ====== helpers (kept inside this function so you can paste easily) ======
  auto mm_to_in = [](double mm) -> double { return mm / 25.4; };

  auto read_avg_in = [&](pros::Distance &d) -> double {
    double sum = 0.0;
    int good = 0;
    for (int i = 0; i < SAMPLES; i++) {
      // PROS Distance sensor "get()" returns mm
      double mm = d.get();
      // basic validity check (tweak if needed)
      if (mm > 10 && mm < 4000) {
        sum += mm_to_in(mm);
        good++;
      }
      pros::delay(SAMPLE_DELAY_MS);
    }
    if (good == 0) return -1.0;
    return sum / good;
  };

  auto lcd_clear_all = []() {
    for (int i = 0; i < 8; i++) pros::lcd::clear_line(i);
  };

  auto wait_release = [&]() {
    // prevent double-press
    while (master.get_digital(pros::E_CONTROLLER_DIGITAL_A) ||
           master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      pros::delay(10);
    }
  };

  auto prompt_and_capture = [&](const char *name, pros::Distance &sensor) -> double {
    while (true) {
      lcd_clear_all();
      pros::lcd::print(0, "DIST CAL: %s", name);
      pros::lcd::print(1, "Put %0.1fin from wall", TRUE_DIST_IN);
      pros::lcd::print(2, "A=sample  B=skip");

      // live reading
      double live_in = mm_to_in(sensor.get());
      pros::lcd::print(4, "Live: %0.2f in", live_in);

      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        wait_release();
        double avg_in = read_avg_in(sensor);
        lcd_clear_all();
        pros::lcd::print(0, "%s sampled", name);
        if (avg_in < 0) {
          pros::lcd::print(1, "No valid readings");
          pros::lcd::print(2, "Check wiring/port");
          pros::delay(1200);
          return 0.0; // fallback offset
        } else {
          double offset_in = avg_in - TRUE_DIST_IN; // measured - true
          pros::lcd::print(1, "Avg: %0.2f in", avg_in);
          pros::lcd::print(2, "Offset: %0.2f in", offset_in);
          pros::lcd::print(3, "(meas - true)");
          pros::delay(1200);
          return offset_in;
        }
      }

      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        wait_release();
        lcd_clear_all();
        pros::lcd::print(0, "%s skipped", name);
        pros::delay(600);
        return 0.0; // skip => no correction
      }

      pros::delay(20);
    }
  };

  // ====== IMPORTANT ======
  // You must have PROS LCD initialized somewhere (usually in initialize()).
  // If not already, add: pros::lcd::initialize();

  // stop drivetrain while calibrating
  chassis.drive_set(0, 0);

  // ====== CALIBRATION ======
  double offF = prompt_and_capture("FRONT", distFront);
  double offR = prompt_and_capture("RIGHT", distRight);
  double offB = prompt_and_capture("BACK",  distBack);
  double offL = prompt_and_capture("LEFT",  distLeft);

  // ====== RESULTS ======
  lcd_clear_all();
  pros::lcd::print(0, "DIST OFFSETS (in)");
  pros::lcd::print(1, "F:%0.2f  R:%0.2f", offF, offR);
  pros::lcd::print(2, "B:%0.2f  L:%0.2f", offB, offL);

  pros::lcd::print(4, "Use as: corrected =");
  pros::lcd::print(5, "measured_in - offset");

  // Keep showing until you exit auton / switch
  while (true) {
    pros::delay(50);
  }
}



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

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}



void odomtest() {
chassis.pid_odom_set({{28.5_in, -5_in, 180_deg}, fwd, 110});
dex.set(true);
chassis.pid_wait_quick_chain();
chassis.pid_odom_set({{28.5_in, -16_in, 180_deg}, fwd, 110});
intake.move(127);
pros::delay(1800);
intake.move(0);
chassis.pid_wait_quick_chain();
chassis.pid_odom_set({{29.25_in, 17.5_in, 180_deg}, rev, 110});
chassis.pid_wait_quick_chain();
hood.set(true);
intake.move(127);
pros::delay(1600);
intake.move(0);

}

void redleftauton() {

}

void redsoloawp() {
}

void fastbluerightauton() {

}

void blueleftauton() {

}

void bluesoloawp() {

}

void bluerightauton() {
 
}
void redrightauton() {
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

