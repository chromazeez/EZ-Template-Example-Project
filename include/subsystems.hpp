#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include <cstdint>

extern Drive chassis;

// ---- devices, built in main.cpp ----

extern pros::Motor claw;
extern pros::Motor toggle;
extern pros::MotorGroup cascade;
extern pros::Optical optical;
extern pros::Distance distanceLeft; // left-facing, for wall resets

// ---- distance sensor reset (DSR) ----
// correct the odom X coordinate off the left field wall
// safe to call anytime, it only acts when the robot is square to the wall and the reading is sane
void resetXFromLeftWall();

// ---- toggle ----

// toggle motor power out of 127
// deliberately slow, fast spinning shoots past the color before the sensor can react
constexpr int TOGGLE_SPEED = 60;

// which color the toggle mech is aiming for
enum class ToggleColor { red, blue, yellow };

// color name for the brain screen
inline const char* nameOf(ToggleColor color) {
    switch (color) {
        case ToggleColor::blue: return "BLUE";
        case ToggleColor::yellow: return "YELLOW";
        default: return "RED";
    }
}

bool seesColor(ToggleColor color); // is the sensor looking at that color right now

void autoToggle(ToggleColor color);                        // one driver control pass, returns right away
void autoToggle(ToggleColor color, std::uint32_t timeout); // blocks until it sees the color, for routes

bool toggleManualMode();     // true once the driver latched the sensor out with A
ToggleColor allianceColor(); // the selected slot's color, so driver control aims the toggle right

// ---- cascade ----
// position = motor degrees from power on, so it has to start fully down

constexpr double CASCADE_TOP = 10000; // top of travel, driver control won't go past it or below 0
constexpr double CASCADE_SCORE = 855; // scoring height, used by the skills route and the L1 tap
constexpr int CASCADE_RPM = 200;      // speed cap for move_absolute, green cartridge tops at 200

constexpr double CASCADE_BOTTOM = 30;     // degrees above the tare point to stop at
constexpr double CASCADE_SLOW_ZONE = 200; // start easing off this far above the floor
constexpr int CASCADE_SLOW_POWER = 40;    // crawl speed inside that zone

// ---- claw ----
// zero is the closed stop. the claw tares there every time it closes, so 0 means fully in and
// nothing is allowed to target past it, same idea as the cascade's tare point.
// open positions are negative, measured out from that stop.
// R1 closes fully, R2 opens fully, X stops at a halfway point. all three drive until they get
// there and then brake, so the motor is on hold and never keeps turning
constexpr int CLAW_SPEED = 110;             // drive speed toward a hard stop, out of 127
constexpr std::uint32_t CLAW_MIN_MS = 150;  // let it get moving before checking for a stall
constexpr std::uint32_t CLAW_MAX_MS = 1000; // give up cap, never drive into the stop longer than this
constexpr double CLAW_STALL_VEL = 10;       // rpm, under this after CLAW_MIN_MS means it reached the stop

// the stall check calls it done the moment the claw slows down, which leaves the last bit of
// squeeze on the table. closing keeps pushing in for this long after the stall before it stops
constexpr std::uint32_t CLAW_CLOSE_EXTRA_MS = 120; // raise to clamp harder, 0 for the old behaviour

constexpr double CLAW_IN = 0;              // closed: the tare point, and as far in as anything can go

// the midpoint as a fraction of the claw's REAL travel, measured between the two hard stops,
// because a fixed degree count can land past the open stop and then nothing moves.
// 0 = fully closed, 1 = fully open. this is the number to tune
constexpr double CLAW_HALF_FRACTION = 0.475;
constexpr double CLAW_HALF = -40;          // fallback only, used until the claw has opened once
constexpr int CLAW_HALF_RPM = 100;         // speed cap on the way to the halfway point (driver control)
constexpr int CLAW_HALF_POWER = 50;        // power out of 127 on the way there in auton, slow so it doesn't overshoot
constexpr double CLAW_HALF_TOLERANCE = 5;  // degrees, close enough to call it arrived and brake

extern bool clawZeroed;    // true once the claw has reached the closed stop and tared there
extern double clawOpenPos; // position of the open stop, measured the first time the claw opens
extern bool clawOpenKnown; // true once that measurement exists

double clawHalfTarget(); // where the midpoint actually is, in motor degrees

constexpr std::uint32_t CLAW_RESET_MS = 60; // how long the motor coasts during a reset

void clawReset(); // auton: put the claw motor back in a known state so the next move takes
void clawClose(); // auton: drive closed and hold, blocks until it reaches the stop
void clawOpen();  // auton: drive open and hold
void clawHalf();  // auton: drive to the midpoint and hold, same position X holds in driver control
