// Cartesian kinematics stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_coord


struct tripteron_stepper {
    struct stepper_kinematics sk;
    double arm2, tower_x, tower_y;
};


// this function needs to set the position of the stepper motors
// which is a double

static double tripteron_stepper_calc_position(struct stepper_kinematics *sk, struct move *m, double move_time)
{

}


// this one just allocates the memory and uses the above function
struct stepper_kinematics * __visible tripteron_stepper_alloc(double arm2, double tower_x, double tower_y)
{

}