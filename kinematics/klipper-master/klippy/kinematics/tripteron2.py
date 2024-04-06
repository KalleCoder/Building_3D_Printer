import math
import logging
import stepper

class ColinearTripteronKinematics:
    def __init__(self, toolhead, config):
        # Initialize the ColinearTripteronSolution with default parameters
        self.kinematics = ColinearTripteronSolution()
        # Setup tower rails
        stepper_configs = [config.getsection('stepper_' + a) for a in 'abc']
        self.rails = [stepper.LookupMultiRail(config, need_position_minmax=False) for config in stepper_configs]
        self.printer = config.get_printer()
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat('max_z_velocity', self.max_velocity, above=0., maxval=self.max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', self.max_accel, above=0., maxval=self.max_accel)

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_position(self, stepper_positions):
        actuator_mm = [stepper_positions[rail.get_name()] for rail in self.rails]
        return self.kinematics.actuator_to_cartesian(actuator_mm)

    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)

    def home(self, homing_state):
        homing_state.set_axes([0, 1, 2])
        homing_state.home_rails(self.rails, forcepos, self.home_position)

    def _motor_off(self, print_time):
        self.limit_xy2 = -1.
        self.need_home = True

    def check_move(self, move):
        end_pos = move.end_pos
        end_xy2 = end_pos[0]**2 + end_pos[1]**2
        if end_xy2 <= self.limit_xy2 and not move.axes_d[2]:
            return
        if self.need_home:
            raise move.move_error("Must home first")
        end_z = end_pos[2]
        limit_xy2 = self.max_xy2
        if end_z > self.limit_z:
            above_z_limit = end_z - self.limit_z
            allowed_radius = self.radius - math.sqrt(
                self.min_arm2 - (self.min_arm_length - above_z_limit)**2
            )
            limit_xy2 = min(limit_xy2, allowed_radius**2)
        if end_xy2 > limit_xy2 or end_z > self.max_z or end_z < self.min_z:
            if (end_pos[:2] != self.home_position[:2]
                or end_z < self.min_z or end_z > self.home_position[2]):
                raise move.move_error()
            limit_xy2 = -1.
        if move.axes_d[2]:
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(self.max_z_velocity * z_ratio,
                             self.max_z_accel * z_ratio)
            limit_xy2 = -1.
        extreme_xy2 = max(end_xy2, move.start_pos[0]**2 + move.start_pos[1]**2)
        if extreme_xy2 > self.slow_xy2:
            r = 0.5
            if extreme_xy2 > self.very_slow_xy2:
                r = 0.25
            move.limit_speed(self.max_velocity * r, self.max_accel * r)
            limit_xy2 = -1.
        self.limit_xy2 = min(limit_xy2, self.slow_xy2)

    def get_status(self, eventtime):
        return {
            'homed_axes': '' if self.need_home else 'xyz',
            'axis_minimum': self.axes_min,
            'axis_maximum': self.axes_max,
            'cone_start_z': self.limit_z,
        }

def load_kinematics(toolhead, config):
    return ColinearTripteronKinematics(toolhead, config)
