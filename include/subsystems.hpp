#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

inline pros::Motor intake(2);
inline pros::Motor score(3);
inline ez::Piston descore('A');
inline ez::Piston dex('B');
inline ez::Piston middle('D');
// inline pros::adi::DigitalIn limit_switch('A');