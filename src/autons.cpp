#include "main.h"
#include <cmath>

/////
// autonomous routes, EZ-Template
// to add one: write the function, declare it in autons.hpp, add it to the selector in main.cpp
/////

// out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SLOW_SPEED = 70; // turns and drives with the cascade raised, robot is top heavy

///
// drive/turn PID constants
// these are EZ-Template's defaults, a solid starting point but NOT tuned for this robot yet
// use the on-controller PID tuner (press X in driver control) to dial kP/kD on the real bot
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // fwd/rev
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // holds heading straight while driving
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // turn in place
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // swing turns
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // angular for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // boomerang motions

  // exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // slew (ramps power at the start of a motion)
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  chassis.odom_turn_bias_set(0.9);
  chassis.odom_look_ahead_set(7_in);
  chassis.odom_boomerang_distance_set(16_in);
  chassis.odom_boomerang_dlead_set(0.625);

  chassis.pid_angle_behavior_set(ez::shortest); // turns take the shortest way round
}

// ---- building blocks ----

// spin the toggle until the sensor reads `color`, or the timeout runs out
void autoToggle(ToggleColor color, std::uint32_t timeout) {
    const std::uint32_t start = pros::millis();

    while (!seesColor(color) && pros::millis() - start < timeout) {
        toggle.move(TOGGLE_SPEED);
        pros::delay(10);
    }

    toggle.move(0);
}

constexpr double CASCADE_TOLERANCE = 10;        // degrees, close enough to call it arrived
constexpr double CASCADE_STALL_DEGREES = 3;     // moved less than this, for
constexpr std::uint32_t CASCADE_STALL_MS = 100; // this long = against a hard stop, quit

// drive the cascade to a position in motor degrees, blocks until there or stalled
// move_absolute uses the motor's own position controller, the stall check is what ends the move
static void cascadeMoveTo(double target, std::uint32_t timeout, int rpm = CASCADE_RPM) {
    cascade.move_absolute(target, rpm);

    const std::uint32_t start = pros::millis();
    double lastPos = cascade.get_position();
    std::uint32_t lastMoved = start;

    while (pros::millis() - start < timeout) {
        const double pos = cascade.get_position();
        if (std::fabs(target - pos) <= CASCADE_TOLERANCE) break; // close enough

        // stopped climbing, so it's against a hard stop. quit instead of cooking the motors
        if (std::fabs(pos - lastPos) > CASCADE_STALL_DEGREES) {
            lastPos = pos;
            lastMoved = pros::millis();
        } else if (pros::millis() - lastMoved > CASCADE_STALL_MS) {
            break;
        }

        pros::delay(10);
    }
}

// drive the claw to its stop in one direction, then hold position there
// power > 0 closes (in), power < 0 opens. blocks until it stalls against the stop or times out
static void clawDriveToStop(int power) {
    const std::uint32_t start = pros::millis();

    while (true) {
        claw.move(power);
        const std::uint32_t elapsed = pros::millis() - start;
        if (elapsed > CLAW_MIN_MS && std::fabs(claw.get_actual_velocity()) < CLAW_STALL_VEL) break;
        if (elapsed > CLAW_MAX_MS) break;
        pros::delay(10);
    }

    claw.brake(); // hold where it stopped
}

void clawClose() { clawDriveToStop(CLAW_SPEED); }   // in
void clawOpen() { clawDriveToStop(-CLAW_SPEED); }   // out

// ---- distance sensor reset (DSR) ----
// one left-facing sensor, corrects the odom X coordinate off the left field wall
// only meaningful near heading 0 (robot square, facing forward) so the beam hits the wall straight on
// gated hard so a bad reading can never teleport the robot

constexpr double DSR_START_READING = 75.0; // inches the sensor read at the auto start, tape measured
constexpr double DSR_MAX_ANGLE = 7.0;      // degrees off square, past this the beam is too slanted to trust
constexpr int DSR_MIN_CONFIDENCE = 50;     // 0-63, below this the sensor is guessing at what it sees
constexpr double DSR_MAX_MM = 2000.0;      // beyond the sensor's useful range
constexpr double DSR_MAX_JUMP = 8.0;       // inches, a correction bigger than this is a bad reading, skip it
constexpr double DEG2RAD = 3.14159265358979 / 180.0;

void resetXFromLeftWall() {
    // how far off square the robot is. heading wraps, so fold it to the nearest of 0
    double off = std::fmod(chassis.drive_imu_get(), 360.0);
    if (off > 180) off -= 360;
    if (off < -180) off += 360;
    if (std::fabs(off) > DSR_MAX_ANGLE) return; // too crooked, the beam isn't on the wall straight

    if (distanceLeft.get_confidence() < DSR_MIN_CONFIDENCE) return; // sensor unsure
    const double mm = distanceLeft.get();
    if (mm <= 0 || mm > DSR_MAX_MM) return; // nothing there, or out of range

    // the slanted reading is the hypotenuse, cos gives the true perpendicular distance to the wall
    const double perp = (mm / 25.4) * std::cos(off * DEG2RAD);

    // odom started where the sensor read DSR_START_READING, so X is how much farther out we are now
    const double center_x = perp - DSR_START_READING;

    if (std::fabs(center_x - chassis.odom_x_get()) > DSR_MAX_JUMP) return; // sanity, don't jump the robot
    chassis.odom_x_set(center_x);
}

// ---- routes ----

// turns are absolute in EZ, measured from where autonomous() zeroed the heading
// the old LemLib turnBy(-45) off 90 becomes turn to 45, the zero-then-135 becomes turn to 180
void skill_route(ToggleColor color) {
    autoToggle(color, 5000);

    chassis.pid_drive_set(7.25_in, DRIVE_SPEED, true);
    chassis.pid_wait();

    // async, climbs through the turn and drive below instead of the robot waiting for it
    cascade.move_absolute(CASCADE_SCORE, CASCADE_RPM);

    chassis.pid_turn_set(90_deg, SLOW_SPEED);
    chassis.pid_wait();

    chassis.pid_drive_set(12.5_in, 60, true);
    chassis.pid_wait();

    cascadeMoveTo(CASCADE_BOTTOM + 250, 2500, 80); // slow on the way down, blocks until there

    clawOpen(); // release

    chassis.pid_drive_set(-12_in, DRIVE_SPEED);
    chassis.pid_wait();

    chassis.pid_turn_set(45_deg, TURN_SPEED);
    chassis.pid_wait();

    cascadeMoveTo(CASCADE_BOTTOM, 2500, 80);

    clawClose(); // grab and hold, no stall

    // drive 31, full speed for the first 28 then crawl at 37 for the last 3
    chassis.pid_drive_set(31_in, DRIVE_SPEED, true);
    chassis.pid_wait_until(28_in);
    chassis.pid_speed_max_set(37);
    chassis.pid_wait();

    // 20% above scoring height, async so it keeps rising through the turn
    cascade.move_absolute(CASCADE_SCORE * 1.6, CASCADE_RPM);

    // claw keeps holding its grip through the turn

    chassis.pid_turn_set(180_deg, SLOW_SPEED);
    chassis.pid_wait();

    chassis.pid_drive_set(12_in, DRIVE_SPEED);
    chassis.pid_wait();
}

// ---- selector wrappers ----
// the EZ selector calls a plain void(), these hand the alliance color to the real route

void skills_auton() { skill_route(ToggleColor::red); }

void red_auton() {
    // TODO: write a red match route
}

void blue_auton() {
    // TODO: write a blue match route
}
