#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"


extern Drive chassis;

inline pros::Distance distFront(12);
inline pros::Distance distRight(20);
inline pros::Distance distBack(19);
inline pros::Distance distLeft(11);

inline pros::MotorGroup intake({7, -5});

inline ez::Piston goal('C');
inline ez::Piston dex('H');
inline ez::Piston hood('A');
inline ez::Piston wing('D');