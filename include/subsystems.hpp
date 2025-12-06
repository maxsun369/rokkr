#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

inline pros::MotorGroup intake({-1, 3});
inline pros::MotorGroup score({-2, 10});
inline ez::Piston descore('A');
inline ez::Piston dex('B');
inline ez::Piston middle('D');
