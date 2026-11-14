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

  // exit conditions, all the times 25% shorter than EZ's defaults so motions settle sooner
  chassis.pid_turn_exit_condition_set(68_ms, 3_deg, 188_ms, 7_deg, 375_ms, 375_ms);
  chassis.pid_swing_exit_condition_set(68_ms, 3_deg, 188_ms, 7_deg, 375_ms, 375_ms);
  chassis.pid_drive_exit_condition_set(68_ms, 1_in, 188_ms, 3_in, 375_ms, 375_ms);
  chassis.pid_odom_turn_exit_condition_set(68_ms, 3_deg, 188_ms, 7_deg, 375_ms, 563_ms);
  chassis.pid_odom_drive_exit_condition_set(68_ms, 1_in, 188_ms, 3_in, 375_ms, 563_ms);
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

// put the claw motor back in a known state. after it has sat braked at a held position the
// motor can stop responding to a plain move(), so drop the command, let the brake go, give it
// a moment to actually be free, then put hold back
void clawReset() {
    claw.move(0);
    claw.set_brake_mode(pros::MotorBrake::coast);
    pros::delay(CLAW_RESET_MS);
    claw.set_brake_mode(pros::MotorBrake::hold);
}

// in. the closed stop is the zero reference, so tare here too, same as driver control does
void clawClose() {
    clawDriveToStop(CLAW_SPEED);

    // the stall check quits as soon as it slows, so push in a bit longer to finish the clamp
    claw.move(CLAW_SPEED);
    pros::delay(CLAW_CLOSE_EXTRA_MS);
    claw.brake();

    claw.tare_position();
    clawZeroed = true;
}

// out. record where the open stop is, that plus the zero gives the claw's real travel
void clawOpen() {
    clawDriveToStop(-CLAW_SPEED);

    if (clawZeroed) {
        clawOpenPos = claw.get_position();
        clawOpenKnown = true;
    }
}

// where the midpoint sits in motor degrees. once both stops have been touched this is a real
// fraction of the travel, so it can never land past the open stop the way a fixed number can
double clawHalfTarget() {
    if (clawOpenKnown) return clawOpenPos * CLAW_HALF_FRACTION;
    return CLAW_HALF; // haven't measured the open stop yet, use the fallback
}

// drive the claw to the midpoint and brake there, the same position X holds in driver control.
// blocks until it arrives, or gives up if something is jamming it
// NOTE: this drives with claw.move(), NOT move_absolute. move_absolute leaves the motor's
// position controller holding the motor, and then the voltage commands in clawClose() and
// clawOpen() get ignored, so the next claw call does nothing at all
void clawHalf() {
    if (!clawZeroed) clawClose(); // the closed stop is the reference, so go find it first

    const double target = clawHalfTarget();

    // which way the midpoint is from here. position counts up toward the closed stop at 0
    const bool outward = claw.get_position() > target;
    const std::uint32_t start = pros::millis();

    while (pros::millis() - start < CLAW_MAX_MS) {
        const double pos = claw.get_position();
        if (outward ? pos <= target : pos >= target) break; // got there

        claw.move(outward ? -CLAW_HALF_POWER : CLAW_HALF_POWER);
        pros::delay(10);
    }

    claw.brake(); // hold it there
}

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

// ---- exit conditions ----
// these are global in EZ, so a motion that doesn't need precision sets the loose ones,
// runs, then puts the tuned ones back. keep these matched to default_constants()

static void exit_conditions_default() {
    chassis.pid_turn_exit_condition_set(68_ms, 3_deg, 188_ms, 7_deg, 375_ms, 375_ms);
    chassis.pid_drive_exit_condition_set(68_ms, 1_in, 188_ms, 3_in, 375_ms, 375_ms);
}

// wider error band, shorter settle. exits sooner, lands less precisely
static void exit_conditions_fast() {
    chassis.pid_turn_exit_condition_set(23_ms, 4_deg, 75_ms, 8_deg, 225_ms, 375_ms);
    chassis.pid_drive_exit_condition_set(23_ms, 2_in, 75_ms, 4_in, 225_ms, 375_ms);
}

// ---- routes ----

// turns are absolute in EZ, measured from where autonomous() zeroed the heading
// claw calls are the no-stall helpers: clawOpen() releases, clawClose() grabs and holds
void skill_route(ToggleColor color) {
    clawClose();

    autoToggle(color, 5000);

    

    chassis.pid_drive_set(10.5_in, DRIVE_SPEED, true);
    chassis.pid_wait();

    // async, climbs through the turn and drive below instead of the robot waiting for it
    cascade.move_absolute(CASCADE_SCORE, CASCADE_RPM);

    chassis.pid_turn_set(90_deg, SLOW_SPEED);
    chassis.pid_wait_quick();

   

    chassis.pid_drive_set(11.75_in, 80, true);
    chassis.pid_wait_quick();


    cascadeMoveTo(CASCADE_BOTTOM + 220, 2500, 200); // slow on the way down, blocks until there

    clawOpen(); // release
    
    exit_conditions_fast();

    chassis.pid_drive_set(-14.5_in, DRIVE_SPEED);
    chassis.pid_wait_quick();

    chassis.pid_turn_set(40_deg, TURN_SPEED);
    chassis.pid_wait_quick();

   

    cascade.move_absolute(CASCADE_BOTTOM, CASCADE_RPM); // async, lowers at max speed through the drive below


    // drive 31, full speed until 24 then crawl at 20 the rest of the way
    chassis.pid_drive_set(26.5_in, 60, true);
    chassis.pid_wait_quick();

    clawClose(); // grab

    pros::delay(250);

    chassis.pid_drive_set(4_in, 40, true);
    chassis.pid_wait_quick();

    exit_conditions_default();

    // 20% above scoring height, async so it keeps rising through the turn
    cascade.move_absolute(CASCADE_SCORE * 1.7, CASCADE_RPM);
    
  
    // claw keeps its grip through the turn

    chassis.pid_drive_set(-3_in, DRIVE_SPEED);
    chassis.pid_wait_quick();

    chassis.pid_turn_set(180_deg, 60);
    chassis.pid_wait_quick();


    chassis.pid_drive_set(19.25_in, 40, true);
    chassis.pid_wait_quick();

    // ---- second scoring cycle ----

    clawOpen(); // release

    // send the cascade down right here. the claw is empty now, so it can lower through the drive
    // back AND the turn AND the 28.5 below, instead of trying to do it all in one drive
    cascade.move_absolute(CASCADE_BOTTOM, 200);

    chassis.pid_drive_set(-19_in, DRIVE_SPEED);
    chassis.pid_wait_quick();

    chassis.pid_turn_set(135_deg, TURN_SPEED);
    chassis.pid_wait_quick();


    chassis.pid_drive_set(28.5_in, 75, true);
    chassis.pid_wait_quick();

    clawClose(); // grab

    chassis.pid_drive_set(-2.5_in, 75, true);
    chassis.pid_wait_quick();

    cascade.move_absolute(CASCADE_SCORE * 1.1, CASCADE_RPM);

    chassis.pid_turn_set(-90_deg, SLOW_SPEED);
    chassis.pid_wait_quick();

    cascade.move_absolute(CASCADE_SCORE * 3.1, 250);

    chassis.pid_drive_set(18_in, 50, true);
    chassis.pid_wait_quick();


    pros::delay(250);
    
    clawOpen();

    pros::delay(150);

    // async, so the cascade comes all the way down during the drive back
    cascade.move_absolute(CASCADE_BOTTOM, 150);


    chassis.pid_drive_set(-30_in, DRIVE_SPEED);
    chassis.pid_wait_quick();
    
    

    chassis.pid_turn_set(178.5_deg, SLOW_SPEED);
    chassis.pid_wait_quick();

    clawHalf(); // midpoint, same as X in driver control

    chassis.pid_drive_set(14.25_in, DRIVE_SPEED);
    chassis.pid_wait_quick();

    clawReset(); // the claw has been braked at the midpoint since line 334, wake the motor up
    clawClose(); // grab

    chassis.pid_drive_set(-2.5_in, DRIVE_SPEED);
    chassis.pid_wait_quick();

  
    chassis.pid_drive_set(2.5_in, DRIVE_SPEED);
    chassis.pid_wait_quick();

    pros::delay(20);

    chassis.pid_drive_set(-13_in, 70);
    chassis.pid_wait_quick();


/*



    // claw keeps its grip through the turn

    chassis.pid_drive_set(-2.5_in, DRIVE_SPEED);
    chassis.pid_wait();



    */
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

// ---- tuning tests ----
// run with B + DOWN, then check the result to tune the PID constants in default_constants()
// autonomous() resets odom/heading to 0 before these run, so no reset needed here

// drive 24", then show what odom thinks on the controller
// the REAL check is a tape measure: if the distance is off, fix wheel diameter / rpm in main.cpp
void test_drive() {
    chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
    chassis.pid_wait();
    master.set_text(1, 0, "y " + util::to_string_with_precision(chassis.odom_y_get()));
}

// turn to 90, then show where it actually landed (target is 90.00)
// watch for overshoot or oscillation and tune pid_turn_constants_set / the turn exit conditions
void test_turn() {
    chassis.pid_turn_set(90_deg, TURN_SPEED);
    chassis.pid_wait();
    master.set_text(1, 0, "h " + util::to_string_with_precision(chassis.drive_imu_get()));
}
