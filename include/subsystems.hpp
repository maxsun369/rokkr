#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern bool keepRed;
int color_sort_adjust(int manualPower);
extern Drive chassis;
void score_auto(int intakePower, int scorePower, int timeMs);

inline pros::MotorGroup intake({-10, 3});
inline pros::MotorGroup score({8});
extern pros::Optical optical;

inline ez::Piston descore('A');
inline ez::Piston dex('C');
inline ez::Piston middle('D');
