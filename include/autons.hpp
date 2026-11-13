#pragma once

#include "subsystems.hpp" // ToggleColor

// drive/turn PID constants, set in autons.cpp
void default_constants();

// the real route, takes the alliance color so the toggle aims right
void skill_route(ToggleColor color);

// wrappers with no args, this is what the EZ selector calls
// Skills runs skill_route, the alliance slots are still TODO
void skills_auton();
void red_auton();
void blue_auton();

// tuning tests: drive 24", turn to 90
void test_drive();
void test_turn();
