#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "compiler.h"
#include "itersolve.h"
#include "trapq.h"

struct tripteron_stepper {
    struct stepper_kinematics sk;
    double rail_positions[3];
    double radius;
    double arm_lengths[3];
    double angles[3];
};

struct tripteron_kinematics {
    struct stepper_kinematics sk;
    struct tripteron_stepper *ts;
};

static double tripteron_stepper_calc_position(struct stepper_kinematics *sk, struct move *m, double move_time) {
    struct tripteron_kinematics *tk = (struct tripteron_kinematics *) sk;
    struct tripteron_stepper *ts = tk->ts;
    struct coord c = move_get_coord(m, move_time);
    
    // Calculate the position based on tripteron kinematics
    double dx = ts->rail_positions[0] - c.x;
    double dy = ts->rail_positions[1] - c.y;
    double dz = ts->rail_positions[2] - c.z;
    
    return sqrt(dx * dx + dy * dy + dz * dz);
}

struct stepper_kinematics * __visible tripteron_stepper_alloc(double *rail_positions, double radius, double *arm_lengths, double *angles) {
    struct tripteron_kinematics *tk = malloc(sizeof(*tk));
    struct tripteron_stepper *ts = malloc(sizeof(*ts));
    memset(tk, 0, sizeof(*tk));
    memset(ts, 0, sizeof(*ts));
    
    memcpy(ts->rail_positions, rail_positions, sizeof(ts->rail_positions));
    ts->radius = radius;
    memcpy(ts->arm_lengths, arm_lengths, sizeof(ts->arm_lengths));
    memcpy(ts->angles, angles, sizeof(ts->angles));

    tk->ts = ts;
    
    tk->sk.calc_position_cb = tripteron_stepper_calc_position;
    tk->sk.active_flags = AF_X | AF_Y | AF_Z;
    
    return &tk->sk;
}

