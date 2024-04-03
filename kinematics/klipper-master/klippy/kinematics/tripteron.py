import math, logging
import stepper, mathutil

# Constants
PIOVER180 = 0.01745329251994329576923690768489 ## if PI is over 180 (something with converting radians to degrees)
SLOW_RATIO = 3.

# Dummy "none" kinematics support (for developer testing)
#
# Copyright (C) 2018-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

class NoneKinematics:
    def __init__(self, toolhead, config):
        self.axes_minmax = toolhead.Coord(0., 0., 0., 0.)

        # Setup tower rails
        stepper_configs = [config.getsection('stepper_' + a) for a in 'abc']
        rail_a = stepper.PrinterRail(
            stepper_configs[0], need_position_minmax=False,
            units_in_radians=True)
        a_endstop = rail_a.get_homing_info().position_endstop
        rail_b = stepper.PrinterRail(
            stepper_configs[1], need_position_minmax=False,
            default_position_endstop=a_endstop, units_in_radians=True)
        rail_c = stepper.PrinterRail(
            stepper_configs[2], need_position_minmax=False,
            default_position_endstop=a_endstop, units_in_radians=True)
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
        

        ## Here I need to setup arm angle
        self.arm_a = PIOVER180 
        


    def get_steppers(self):
        return []


    def calc_position(self, stepper_positions):
        return [0, 0, 0]


    def set_position(self, newpos, homing_axes):
        pass


    def home(self, homing_state):
        pass


    def check_move(self, move):
        pass


    def get_status(self, eventtime):
        return {
            'homed_axes': '',
            'axis_minimum': self.axes_minmax,
            'axis_maximum': self.axes_minmax,
        }


def load_kinematics(toolhead, config):
    return NoneKinematics(toolhead, config)

