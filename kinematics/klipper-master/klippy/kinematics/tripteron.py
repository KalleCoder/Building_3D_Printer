import math, logging
import stepper, mathutil

# Constants
PIOVER180 = 0.01745329251994329576923690768489
SLOW_RATIO = 3.

class ColinearTripteronKinematics:
    def __init__(self, toolhead, config):
        # Setup tower rails
        stepper_configs = [config.getsection('stepper_' + a) for a in 'abc']
        rail_a = stepper.LookupMultiRail(
            stepper_configs[0], need_position_minmax=False)
        a_endstop = rail_a.get_homing_info().position_endstop
        rail_b = stepper.LookupMultiRail(
            stepper_configs[1], need_position_minmax=False,
            default_position_endstop=a_endstop)
        rail_c = stepper.LookupMultiRail(
            stepper_configs[2], need_position_minmax=False,
            default_position_endstop=a_endstop)
        self.rails = [rail_a, rail_b, rail_c]
        config.get_printer().register_event_handler("stepper_enable:motor_off",
                                                    self._motor_off)

        # Setup max velocity
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', self.max_velocity,
            above=0., maxval=self.max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', self.max_accel,
                                          above=0., maxval=self.max_accel)

        # Read radius and arm lengths
        self.radius = radius = config.getfloat('delta_radius', above=0.)
        arm_length_a = stepper_configs[0].getfloat('arm_length', above=radius)
        self.arm_lengths = arm_lengths = [
            sconfig.getfloat('arm_length', arm_length_a, above=radius)
            for sconfig in stepper_configs]
        self.arm2 = [arm**2 for arm in arm_lengths]

        # Setup boundary checks
        self.max_z = min([rail.get_homing_info().position_endstop
                          for rail in self.rails])
        self.min_z = config.getfloat('minimum_z_position', 0, maxval=self.max_z)

        # Setup movement limitations
        self.slow_xy2 = SLOW_RATIO * math.sqrt(min(arm_lengths)**2 - radius**2)
        self.max_xy2 = (min(arm_lengths) - radius)**2

        logging.info(
            "Delta max build height %.2fmm"
            % (self.max_z))

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def _actuator_to_cartesian(self, spos):
        sphere_coords = [(spos[i] * math.cos(math.radians(120 * i)),
                          spos[i] * math.sin(math.radians(120 * i)),
                          self.rails[i].get_homing_info().position_endstop)
                         for i in range(3)]
        return mathutil.trilateration(sphere_coords, self.arm2)

    def calc_position(self, stepper_positions):
        spos = [stepper_positions[rail.get_name()] for rail in self.rails]
        return self._actuator_to_cartesian(spos)

    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)

    def _motor_off(self, print_time):
        pass

    def check_move(self, move):
        end_pos = move.end_pos
        end_xy2 = end_pos[0]**2 + end_pos[1]**2
        if end_xy2 > self.max_xy2 or end_pos[2] > self.max_z or end_pos[2] < self.min_z:
            raise move.move_error("Move out of range")

        if move.axes_d[2]:
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(self.max_z_velocity * z_ratio,
                             self.max_z_accel * z_ratio)

        if end_xy2 > self.slow_xy2:
            move.limit_speed(self.max_velocity / SLOW_RATIO, self.max_accel / SLOW_RATIO)

    def get_status(self, eventtime):
        return {
            'homed_axes': 'xyz',
            'axis_minimum': (-self.radius, -self.radius, self.min_z, 0.),
            'axis_maximum': (self.radius, self.radius, self.max_z, 0.),
            'cone_start_z': self.min_z,
        }

def load_kinematics(toolhead, config):
    return ColinearTripteronKinematics(toolhead, config)


# Example usage:
config = {
    "arm_angle": 30.0,
    "alpha_tower_rotation": 0.0,
    "beta_tower_rotation": 120.0,
    "gamma_tower_rotation": 240.0
}

# Initialize the kinematics object
solution = ColinearTripteronKinematics(config)

# Example conversion from Cartesian to actuator coordinates
cartesian = [1.0, 2.0, 3.0]
actuator = solution.cartesian_to_actuator(cartesian)
print("Actuator coordinates:", actuator)

# Example conversion from actuator to Cartesian coordinates
actuator_converted = [10.0, 20.0, 30.0]
cartesian_converted = solution.actuator_to_cartesian(actuator_converted)
print("Converted Cartesian coordinates:", cartesian_converted)

