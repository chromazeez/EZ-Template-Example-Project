#pragma once

#include "subsystems.hpp" // ToggleColor

// drive/turn PID constants, set in autons.cpp
void default_constants();

// the real route, takes the alliance color so the toggle aims right
void skill_route(ToggleColor color);

// wrappers with no args, this is what the EZ selector calls
void skills_auton();
void red_right();  // skills route, red toggle, bottom-right start
void blue_right(); // same route, blue toggle, 180-degree mirror (top-left start)
