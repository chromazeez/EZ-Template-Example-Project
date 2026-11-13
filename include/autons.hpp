#pragma once

#include "subsystems.hpp" // ToggleColor

// drive/turn PID constants, set in autons.cpp
void default_constants();

// the real route, takes the alliance color so the toggle aims right
void skill_route(ToggleColor color);
