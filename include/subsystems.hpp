#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

inline pros::MotorGroup intake({-10, 3});
inline pros::MotorGroup score({8});
extern pros::Optical optical;

inline ez::Piston descore('A');
inline ez::Piston dex('C');
inline ez::Piston middle('D');
