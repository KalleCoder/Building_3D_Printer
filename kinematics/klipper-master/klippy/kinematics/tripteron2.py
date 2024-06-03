import math, logging
import stepper

# Dummy "none" kinematics support (for developer testing)
#
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

class TripteronKinematics:
    def __init__(self, toolhead, config):


         ## KINEMATICS

        self.arm_angle = config.getfloat('arm_angle', above=0)
        self.a_a = math.radians(self.arm_angle)
        self.a_t = math.tan(self.a_a)

        self.a_r = math.radians(0)
        self.b_r = math.radians(120)
        self.g_r = math.radians(240)

        self.a_x = math.sin(self.a_r) * self.a_t
        self.a_y = math.cos(self.a_r) * self.a_t
        self.b_x = math.sin(self.b_r) * self.a_t
        self.b_y = math.cos(self.b_r) * self.a_t
        self.g_x = math.sin(self.g_r) * self.a_t
        self.g_y = math.cos(self.g_r) * self.a_t


        self.d = self.a_y*self.b_x - self.g_y*self.b_x - self.a_x*self.b_y - self.a_y*self.g_x + self.b_y*self.g_x + self.a_x*self.g_y


        # Setup tower rails
        stepper_configs = [config.getsection('stepper_' + a) for a in 'abc']
        rail_a = stepper.PrinterRail(stepper_configs[0], need_position_minmax=False, units_in_radians=True)
        a_endstop = rail_a.get_homing_info().position_endstop
        rail_b = stepper.PrinterRail(stepper_configs[1], need_position_minmax=False, default_position_endstop=a_endstop, units_in_radians=True)
        rail_c = stepper.PrinterRail(stepper_configs[2], need_position_minmax=False, default_position_endstop=a_endstop, units_in_radians=True)
        self.rails = [rail_a, rail_b, rail_c]
        config.get_printer().register_event_handler("stepper_enable:motor_off", self._motor_off)
        
         # Setup max velocity
        self.max_velocity, self.max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat('max_z_velocity', self.max_velocity, above=0., maxval=self.max_velocity)
        self.max_z_accel = config.getfloat('max_z_accel', self.max_accel, above=0., maxval=self.max_accel)
    

         # Getting radious of plate and arms
        self.radius = radius = config.getfloat('delta_radius', above=0.) # should be the same as arm length
        print_radius = config.getfloat('print_radius', radius, above=0.) # defined by build plate

        arm_length_a = stepper_configs[0].getfloat('arm_length', above=radius)
        self.arm_lengths = arm_lengths = [sconfig.getfloat('arm_length', arm_length_a, above=radius) for sconfig in stepper_configs]
        self.arm2 = [arm**2 for arm in arm_lengths]
        self.abs_endstops = [(rail.get_homing_info().position_endstop + math.sqrt(arm2 - radius**2)) for rail, arm2 in zip(self.rails, self.arm2)]

         # Setup boundary checks
        self.need_home = True
        self.limit_xy2 = -1.
        self.home_position = tuple(self._actuator_to_cartesian(self.abs_endstops))
        self.max_z = min([rail.get_homing_info().position_endstop for rail in self.rails])
    
        self.min_z = config.getfloat('minimum_z_position', above=0.)

        self.limit_z = min([ep - arm for ep, arm in zip(self.abs_endstops, self.arm_lengths)])
        self.min_arm_length = min_arm_length = min(self.arm_lengths)
        self.min_arm2 = min_arm_length**2
        logging.info( "Max build height %.2fmm (radius tapered above %.2fmm)" % (self.max_z, self.limit_z))


        # Determine tower locations in cartesian space
        self.angles = [sconfig.getfloat('angle', angle) for sconfig, angle in zip(stepper_configs, [0., 120., 240.])] ## tried to change to my own values here
        self.towers = [(math.cos(math.radians(angle)) * radius, math.sin(math.radians(angle)) * radius) for angle in self.angles]

        for r, a, t in zip(self.rails, self.arm2, self.towers):
            r.setup_itersolve('delta_stepper_alloc', a, t[0], t[1])

        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)

        ## Build plate max and min
        self.max_xy2 = min(print_radius, radius) ## 
        max_xy = math.sqrt(self.max_xy2)
        self.axes_min = toolhead.Coord(-max_xy, -max_xy, self.min_z, 0.)
        self.axes_max = toolhead.Coord(max_xy, max_xy, self.max_z, 0.)
        self.set_position([0., 0., 0.], ())
 

   
    
    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]


    def _actuator_to_cartesian(self, actuator_mm):
        x = (actuator_mm[0]*(self.g_y-self.b_y) + actuator_mm[1]*(self.a_y-self.g_y) + actuator_mm[2]*(self.b_y-self.a_y)) / self.d
        y = (actuator_mm[0]*(self.g_x-self.b_x) + actuator_mm[1]*(self.a_x-self.g_x) + actuator_mm[2]*(self.b_x-self.a_x)) / self.d
        z = (actuator_mm[0]*(self.b_y*self.g_x-self.b_x*self.g_y) + actuator_mm[1]*(self.a_x*self.g_y-self.a_y*self.g_x) + actuator_mm[2]*(self.a_y*self.b_x-self.a_x*self.b_y)) / self.d

        return [x, y, z]


    def calc_position(self, stepper_positions):
        spos = [stepper_positions[rail.get_name()] for rail in self.rails]
        return self._actuator_to_cartesian(self, spos)


    def set_position(self, newpos, homing_axes):
        for rail in self.rails:
            rail.set_position(newpos)
        self.limit_xy2 = -1.
        if tuple(homing_axes) == (0, 1, 2):
            self.need_home = False


    def home(self, homing_state):
        # All axes are homed simultaneously
        homing_state.set_axes([0, 1, 2])
        forcepos = list(self.home_position)
        forcepos[2] = -1.5 * math.sqrt(max(self.arm2)-self.max_xy2)
        homing_state.home_rails(self.rails, forcepos, self.home_position)


    def check_move(self, move):
        end_pos = move.end_pos
        end_xy2 = end_pos[0]**2 + end_pos[1]**2
        if end_xy2 <= self.limit_xy2 and not move.axes_d[2]:
            # Normal XY move
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
            # Move out of range - verify not a homing move
            if (end_pos[:2] != self.home_position[:2]
                or end_z < self.min_z or end_z > self.home_position[2]):
                raise move.move_error()
            limit_xy2 = -1.
        if move.axes_d[2]:
            z_ratio = move.move_d / abs(move.axes_d[2])
            move.limit_speed(self.max_z_velocity * z_ratio,
                             self.max_z_accel * z_ratio)
            limit_xy2 = -1.
        # Limit the speed/accel of this move if is is at the extreme
        # end of the build envelope
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
    
    ## ADDITIONAL STUFF THAT CAN BE GOOD ####
    def _motor_off(self, print_time):
        self.limit_xy2 = -1.
        self.need_home = True

# Define the load_kinematics function outside the TripteronKinematics class
def load_kinematics(toolhead, config):
    return TripteronKinematics(toolhead, config)
