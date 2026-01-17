#include "main.h"
#include <cmath> 

#include <vector>
#include <random>

// ====== FIELD SIZE (inches) ======
constexpr double FIELD_W = 144.0;
constexpr double FIELD_H = 144.0;

// ====== PARTICLE ======
struct Particle {
  double x;      // inches
  double y;      // inches
  double theta;  // radians
  double w;
};

static std::vector<Particle> particles;
static std::mt19937 rng(7);

// ====== MATH HELPERS ======
static double deg_to_rad(double d) { return d * M_PI / 180.0; }
static double rad_to_deg(double r) { return r * 180.0 / M_PI; }

static double wrap_rad(double a) {
  while (a > M_PI) a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

static double clamp(double v, double lo, double hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// ====== SENSOR MOUNTS (MEASURE THESE) ======
// robot frame: +x right, +y forward
struct SensorMount {
  double x;
  double y;
  double heading_deg; // 0=fwd, 90=right, 180=back, -90=left
};

// TODO: replace these offsets with YOUR measurements (inches)
constexpr SensorMount S_FRONT{ 0.0,  5.0,   0.0};
constexpr SensorMount S_BACK { 0.0, -5.0, 180.0};
constexpr SensorMount S_RIGHT{ 5.0,  0.0,  90.0};
constexpr SensorMount S_LEFT {-5.0,  0.0, -90.0};

// ====== RAYCAST TO RECTANGLE WALLS ======
static double raycast_to_walls(double x0, double y0, double ang) {
  const double dx = std::cos(ang);
  const double dy = std::sin(ang);

  double tmin = 1e9;

  if (std::abs(dx) > 1e-6) {
    double t1 = (0.0 - x0) / dx;
    double y1 = y0 + t1 * dy;
    if (t1 >= 0 && y1 >= 0 && y1 <= FIELD_H) tmin = std::min(tmin, t1);

    double t2 = (FIELD_W - x0) / dx;
    double y2 = y0 + t2 * dy;
    if (t2 >= 0 && y2 >= 0 && y2 <= FIELD_H) tmin = std::min(tmin, t2);
  }

  if (std::abs(dy) > 1e-6) {
    double t3 = (0.0 - y0) / dy;
    double x3 = x0 + t3 * dx;
    if (t3 >= 0 && x3 >= 0 && x3 <= FIELD_W) tmin = std::min(tmin, t3);

    double t4 = (FIELD_H - y0) / dy;
    double x4 = x0 + t4 * dx;
    if (t4 >= 0 && x4 >= 0 && x4 <= FIELD_W) tmin = std::min(tmin, t4);
  }

  if (tmin > 1e8) return 9999.0;
  return tmin;
}

static double expected_distance_for_sensor(const Particle& p, const SensorMount& s) {
  double ct = std::cos(p.theta);
  double st = std::sin(p.theta);

  double sx = p.x + (s.x * ct - s.y * st);
  double sy = p.y + (s.x * st + s.y * ct);

  double sang = wrap_rad(p.theta + deg_to_rad(s.heading_deg));
  return raycast_to_walls(sx, sy, sang);
}

// ====== INIT ======
static void mcl_init(double x0, double y0, double theta0_deg, int N = 500) {
  particles.clear();
  particles.reserve(N);

  std::normal_distribution<double> nxy(0.0, 6.0);
  std::normal_distribution<double> nth(0.0, deg_to_rad(10.0));

  for (int i = 0; i < N; i++) {
    Particle p;
    p.x = clamp(x0 + nxy(rng), 0.0, FIELD_W);
    p.y = clamp(y0 + nxy(rng), 0.0, FIELD_H);
    p.theta = wrap_rad(deg_to_rad(theta0_deg) + nth(rng));
    p.w = 1.0 / N;
    particles.push_back(p);
  }
}

// ====== MOTION STATE ======
static double last_tracker_in = 0.0;
static double last_imu_deg = 0.0;

static void mcl_predict(double tracker_now_in, double imu_now_deg) {
  double dS = tracker_now_in - last_tracker_in;
  double dTh = wrap_rad(deg_to_rad(imu_now_deg - last_imu_deg));

  last_tracker_in = tracker_now_in;
  last_imu_deg = imu_now_deg;

  std::normal_distribution<double> nS(0.0, 0.25);
  std::normal_distribution<double> nTh(0.0, deg_to_rad(1.0));

  for (auto& p : particles) {
    double ds = dS + nS(rng);
    double dtheta = dTh + nTh(rng);

    p.theta = wrap_rad(p.theta + dtheta);
    p.x = clamp(p.x + ds * std::cos(p.theta), 0.0, FIELD_W);
    p.y = clamp(p.y + ds * std::sin(p.theta), 0.0, FIELD_H);
  }
}

// ====== MEASUREMENT UPDATE ======
static double gauss_pdf(double e, double sigma) {
  double a = e / sigma;
  return std::exp(-0.5 * a * a);
}

static bool read_dist_in(pros::Distance& d, double& out_in) {
  int mm = d.get();
  if (mm <= 20) return false;
  out_in = mm / 25.4;
  return true;
}

static void mcl_update_with_distances() {
  constexpr double SIGMA = 2.0;

  double zF, zB, zR, zL;
  bool okF = read_dist_in(distFront, zF);
  bool okB = read_dist_in(distBack,  zB);
  bool okR = read_dist_in(distRight, zR);
  bool okL = read_dist_in(distLeft,  zL);

  if (!okF && !okB && !okR && !okL) return;

  double wsum = 0.0;
  for (auto& p : particles) {
    double w = 1.0;
    if (okF) w *= gauss_pdf(zF - expected_distance_for_sensor(p, S_FRONT), SIGMA);
    if (okB) w *= gauss_pdf(zB - expected_distance_for_sensor(p, S_BACK),  SIGMA);
    if (okR) w *= gauss_pdf(zR - expected_distance_for_sensor(p, S_RIGHT), SIGMA);
    if (okL) w *= gauss_pdf(zL - expected_distance_for_sensor(p, S_LEFT),  SIGMA);

    p.w = std::max(w, 1e-12);
    wsum += p.w;
  }

  for (auto& p : particles) p.w /= wsum;
}

// ====== RESAMPLE ======
static void mcl_resample() {
  std::vector<Particle> newP(particles.size());

  std::uniform_real_distribution<double> uni(0.0, 1.0 / particles.size());
  double r = uni(rng);
  double c = particles[0].w;
  int i = 0;

  for (int m = 0; m < (int)particles.size(); m++) {
    double U = r + (double)m / particles.size();
    while (U > c && i < (int)particles.size() - 1) {
      i++;
      c += particles[i].w;
    }
    newP[m] = particles[i];
    newP[m].w = 1.0 / particles.size();
  }
  particles.swap(newP);
}

static void mcl_estimate(double& x, double& y, double& theta_rad) {
  double mx = 0, my = 0;
  double cs = 0, sn = 0;

  for (const auto& p : particles) {
    mx += p.x * p.w;
    my += p.y * p.w;
    cs += std::cos(p.theta) * p.w;
    sn += std::sin(p.theta) * p.w;
  }

  x = mx;
  y = my;
  theta_rad = std::atan2(sn, cs);
}


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
    {"DIST CALIBRATE", distance_calibrate},
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
start_mcl_task();
}

void start_mcl_task();



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

static pros::Task* mclTask = nullptr;

static void mcl_task_fn(void*) {
  // IMPORTANT: set this to your real start tile pose.
  // For now, use center field as a safe default:
  double startX = 72.0;
  double startY = 72.0;
  double startTh = 0.0;

  mcl_init(startX, startY, startTh, 500);

  // set motion baselines
  last_tracker_in = vert_tracker.get();
  last_imu_deg = chassis.drive_imu_get();

  while (true) {
    // 1) motion update
    mcl_predict(vert_tracker.get(), chassis.drive_imu_get());

    // 2) measurement update + resample
    mcl_update_with_distances();
    mcl_resample();

    // 3) estimate pose
    double x, y, th;
    mcl_estimate(x, y, th);

    // 4) re-anchor EZ odom (snap)
    chassis.odom_xyt_set(x * 1_in, y * 1_in, rad_to_deg(th) * 1_deg);

    pros::delay(20);
  }
}

void start_mcl_task() {
  if (mclTask == nullptr) {
    mclTask = new pros::Task(mcl_task_fn, nullptr, "MCL");
  }
}


void autonomous() {
  // Reset everything for consistent auton
  chassis.pid_targets_reset();             
  chassis.drive_imu_reset();               
  pros::delay(300); // give imu a moment
last_tracker_in = vert_tracker.get();
last_imu_deg = chassis.drive_imu_get();
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
