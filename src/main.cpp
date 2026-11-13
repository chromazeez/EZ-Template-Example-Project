#include "main.h"
#include <cmath>

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // the first motor of each side is the one used for distance sensing
    {19, 17, 6},     // Left Chassis Ports (negative port will reverse it!)
    {-20, -18, -3},  // Right Chassis Ports (negative port will reverse it!)

    5,       // IMU Port
    2.75,    // Wheel Diameter
    450);    // Wheel RPM (proven on the old bot: drove 24.25" on a 24" target. verify with a tape measure)

// horizontal tracking pod, 2" omni, reversed like the drive, 0 offset (sits at the tracking center)
ez::tracking_wheel horiz_tracker(-4, 2.0, 0.0);

// ---- other devices ----

pros::Motor claw(2, pros::MotorGears::green);
pros::Motor toggle(8, pros::MotorGears::green);
pros::MotorGroup cascade({-9, 10}, pros::MotorGearset::green); // two motors, 9 faces the other way
pros::Optical optical(16);
pros::Distance distanceLeft(7); // left-facing, reads the left wall for odom X resets (DSR)

// what the claw is doing in driver control, so one tap can drive it to the stop then let go
enum class ClawGoal { hold, closing, opening };

// ---- auto toggle ----
// driver lines up with a toggle, sensor sees something close, motor spins until the right color
// shows up, then stops

constexpr int TOGGLE_MIN_PROXIMITY = 100; // 0-255, below this assume nothing is in front of the sensor

// hue = angle round the color wheel, 0-360, measured off the real toggles
constexpr double RED_HUE = 353;
constexpr double BLUE_HUE = 195;
constexpr double YELLOW_HUE = 51;
constexpr double HUE_TOLERANCE = 25;

constexpr double hueOf(ToggleColor color) {
    switch (color) {
        case ToggleColor::blue: return BLUE_HUE;
        case ToggleColor::yellow: return YELLOW_HUE;
        default: return RED_HUE;
    }
}

bool seesColor(ToggleColor color) {
    double diff = std::fabs(optical.get_hue() - hueOf(color));
    if (diff > 180) diff = 360 - diff; // hue wraps, 359 and 1 are 2 apart not 358
    return diff <= HUE_TOLERANCE;
}

// one pass of the auto behaviour, driver control calls it every loop
void autoToggle(ToggleColor color) {
    const bool somethingClose = optical.get_proximity() > TOGGLE_MIN_PROXIMITY;

    if (somethingClose && !seesColor(color)) {
        toggle.move(TOGGLE_SPEED); // toggle is there, wrong color, keep spinning
    } else {
        toggle.move(0); // nothing there, or we already found our color
    }
}

// failsafe latch for a dead or lying optical sensor, A cuts the sensor out, stays latched
static bool toggleManual = false;
bool toggleManualMode() { return toggleManual; }

// which alliance the selected slot is, so driver control aims the toggle right
//   slot 0 Skills (red), slot 1 Red, slot 2 Blue
ToggleColor allianceColor() {
    return ez::as::auton_selector.auton_page_current == 2 ? ToggleColor::blue : ToggleColor::red;
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 */
void initialize() {
  ez::ez_template_print();
  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // horizontal tracker sits behind the midline, so it's the "back" tracker
  chassis.odom_tracker_back_set(&horiz_tracker);

  // no curve-edit buttons, they'd steal A (our failsafe) and the arrows (our toggle)
  chassis.opcontrol_curve_buttons_toggle(false);
  chassis.opcontrol_drive_activebrake_set(0.0);
  chassis.opcontrol_curve_default_set(0.0, 0.0);

  default_constants();

  // the middle motor of each side is a green cartridge, the outer two are blue
  // set them explicitly so distance sensing (the first motor) reads a blue cartridge
  chassis.left_motors[0].set_gearing(pros::MotorGears::blue);
  chassis.left_motors[1].set_gearing(pros::MotorGears::green);
  chassis.left_motors[2].set_gearing(pros::MotorGears::blue);
  chassis.right_motors[0].set_gearing(pros::MotorGears::blue);
  chassis.right_motors[1].set_gearing(pros::MotorGears::green);
  chassis.right_motors[2].set_gearing(pros::MotorGears::blue);

  cascade.set_brake_mode(pros::MotorBrake::coast); // so it can be pushed down by hand pre match
  cascade.tare_position();                         // call this zero, so the cascade MUST start fully down

  optical.set_led_pwm(100);          // sensor light full blast, keeps hue off room lighting
  optical.set_integration_time(20);  // default 100ms is too slow for a spinning toggle

  toggle.set_brake_mode(pros::MotorBrake::hold); // stop ON the color instead of coasting past
  claw.set_brake_mode(pros::MotorBrake::hold);

  // Autonomous selector using LLEMU (temporary screen, the custom bat screen comes next)
  ez::as::auton_selector.autons_add({
      {"Skills\n\nFull skills route", skills_auton},
      {"Red Alliance\n\nTODO: match route", red_auton},
      {"Blue Alliance\n\nTODO: match route", blue_auton},
      {"Test Drive\n\nDrive 24 inches", test_drive},
      {"Test Turn\n\nTurn to 90 degrees", test_turn},
  });

  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

/**
 * Runs while the robot is disabled.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to a comp switch.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code.
 */
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold helps autonomous consistency
  cascade.set_brake_mode(pros::MotorBrake::hold); // route needs the cascade to stay where it stops

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from the selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());
  }
  ez::screen_print(tracker_value + tracker_width, line);
}

/**
 * Ez screen task, lets you view debug pages during driver control or autonomous
 */
void ez_screen_task() {
  while (true) {
    if (!pros::competition::is_connected()) {
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        if (ez::as::page_blank_is_on(0)) {
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);

          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          // live distance sensor readout, handy for checking DSR on the field
          ez::screen_print("dsr " + util::to_string_with_precision((double)distanceLeft.get() / 25.4) +
                               "in  conf " + util::to_string_with_precision((double)distanceLeft.get_confidence()),
                           7);
        }
      }
    }

    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Extras for opcontrol:
 * - press B + DOWN to run the selected autonomous (only off a comp switch)
 * - EZ's live PID tuner (X) is commented out below, we don't use it
 */
void ez_template_extras() {
  if (!pros::competition::is_connected()) {
    // PID tuner, left here but disabled
    // if (master.get_digital_new_press(DIGITAL_X))
    //   chassis.pid_tuner_toggle();

    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // chassis.pid_tuner_iterate();
  }

  // else {
  //   if (chassis.pid_tuner_enabled())
  //     chassis.pid_tuner_disable();
  // }
}

/**
 * Runs the operator control code.
 *   left stick drive, right stick turn
 *   R1 / R2   claw one way / the other (hold)
 *   L1 / L2   cascade up / down (hold)
 *   L1 tap    cascade drives itself to scoring height
 *   up / down spin the toggle by hand (hold)
 *   A         failsafe latch for the optical sensor
 *   B + DOWN  run the selected auton, practice only
 *   X         live PID tuner
 */
void opcontrol() {
  chassis.drive_brake_set(MOTOR_BRAKE_COAST); // drive coasts, feels better to drive on
  cascade.set_brake_mode(pros::MotorBrake::hold); // but the cascade holds so it doesn't sag

  constexpr std::uint32_t TAP_MS = 200;            // shorter press = tap, longer = hold
  constexpr std::uint32_t MACRO_GIVE_UP_MS = 2000; // stop pushing if it hasn't got there by now

  std::uint32_t upPressedAt = 0;
  bool cascadeMacro = false;

  ClawGoal clawGoal = ClawGoal::hold; // what the claw is doing right now
  std::uint32_t clawStart = 0;        // when the current claw move started

  while (true) {
    ez_template_extras(); // B + DOWN runs the auton, X opens the PID tuner

    chassis.opcontrol_arcade_standard(ez::SPLIT); // left stick drive, right stick turn

    // claw
    // tap R1 to close (in), R2 to open. drive to the stop, catch the stall, then hold position
    // so holding the button never stalls the motor at full power
    if (master.get_digital_new_press(DIGITAL_R1)) {
      clawGoal = ClawGoal::closing;
      clawStart = pros::millis();
    } else if (master.get_digital_new_press(DIGITAL_R2)) {
      clawGoal = ClawGoal::opening;
      clawStart = pros::millis();
    }

    if (clawGoal == ClawGoal::closing || clawGoal == ClawGoal::opening) {
      claw.move(clawGoal == ClawGoal::closing ? CLAW_SPEED : -CLAW_SPEED);
      const std::uint32_t elapsed = pros::millis() - clawStart;
      // stopped moving after it got going, or ran out of time = it's at the stop
      if ((elapsed > CLAW_MIN_MS && std::fabs(claw.get_actual_velocity()) < CLAW_STALL_VEL) ||
          elapsed > CLAW_MAX_MS) {
        clawGoal = ClawGoal::hold;
      }
    } else {
      claw.brake(); // hold where it stopped, brake mode is hold
    }

    // A latches the optical sensor out
    if (master.get_digital_new_press(DIGITAL_A)) {
      toggleManual = !toggleManual;
      master.rumble(toggleManual ? "-" : ".");
    }

    const bool spinFwd = master.get_digital(DIGITAL_UP);
    const bool spinBack = master.get_digital(DIGITAL_DOWN);

    if (spinFwd) {
      toggle.move(TOGGLE_SPEED); // a held arrow always beats the sensor
    } else if (spinBack) {
      toggle.move(-TOGGLE_SPEED);
    } else if (toggleManual) {
      toggle.move(0);
    } else {
      autoToggle(allianceColor());
    }

    // cascade
    // tap L1 -> drives itself to CASCADE_SCORE, hold L1 -> manual up, L2 -> manual down
    const double pos = cascade.get_position();
    const bool up = master.get_digital(DIGITAL_L1);
    const bool down = master.get_digital(DIGITAL_L2);

    if (master.get_digital_new_press(DIGITAL_L1)) {
      upPressedAt = pros::millis();
    }

    if (master.get_digital_new_press(DIGITAL_L1) &&
        pros::millis() - upPressedAt < TAP_MS) {
      cascade.move_absolute(CASCADE_SCORE, CASCADE_RPM);
      cascadeMacro = true;
    }

    const bool holdingUp = up && pros::millis() - upPressedAt >= TAP_MS;

    if (holdingUp) {
      cascadeMacro = false;
      if (pos < CASCADE_TOP) {
        cascade.move(127);
      } else {
        cascade.brake();
      }
    } else if (down) {
      cascadeMacro = false;
      const double room = pos - CASCADE_BOTTOM;
      if (room <= 0) {
        cascade.brake();
      } else if (room < CASCADE_SLOW_ZONE) {
        cascade.move(-CASCADE_SLOW_POWER);
      } else {
        cascade.move(-127);
      }
    } else if (cascadeMacro) {
      if (pros::millis() - upPressedAt > MACRO_GIVE_UP_MS) {
        cascadeMacro = false;
        cascade.brake();
      }
    } else {
      cascade.brake();
    }

    pros::delay(ez::util::DELAY_TIME); // keep this, EZ uses it for timing
  }
}
