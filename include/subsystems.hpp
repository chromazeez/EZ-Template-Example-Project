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
// R1 closes (in), R2 opens. the motor drives to its stop, then holds position instead of
// grinding at full power, so holding the button can't stall and burn the motor
constexpr int CLAW_SPEED = 110;             // drive speed toward the stop, out of 127
constexpr std::uint32_t CLAW_MIN_MS = 150;  // let it get moving before checking for a stall
constexpr std::uint32_t CLAW_MAX_MS = 1000; // give up cap, never drive into the stop longer than this
constexpr double CLAW_STALL_VEL = 10;       // rpm, under this after CLAW_MIN_MS means it reached the stop

void clawClose(); // auton: drive closed and hold, blocks until it reaches the stop
void clawOpen();  // auton: drive open and hold
