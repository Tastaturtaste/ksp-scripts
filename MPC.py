import numpy as np
import matplotlib.pyplot as plt
import time
import math
from gekko import GEKKO

class MPC_Whiplash_Thrust:
    def __init__(self,spoolup_time,ctrl_horizon,ctrl_frequency,pred_horizon,pred_frequency):
        self.m = GEKKO(remote=False)
        self.m.options.IMODE = 6
        self.m.options.CSV_READ = 0
        self.m.options.CTRL_HOR = ctrl_horizon
        self.m.options.CTRL_TIME = 1.0 / ctrl_frequency
        self.m.options.PRED_HOR = pred_horizon
        self.m.options.PRED_TIME = 1.0 / pred_frequency
        self.m.options.WEB = 0
        self.m.options.MEAS_CHK = 0
        self.m.options.CV_TYPE = 2
        self.reference = self.m.Param(value=0,name='reference')
        self.spoolup_time = self.m.Const(value=spoolup_time,name='spoolup_time')
        self.available_thrust = self.m.Param(name='available_thrust')
        self.throttle = self.m.MV(value=0,lb=0,ub=100,name='throttle')
        self.throttle.STATUS = 1
        self.thrust = self.m.CV(value=0,lb=0,name='thrust')
        self.thrust.FSTATUS = 0
        self.thrust.STATUS = 0
        self.m.Equation(self.spoolup_time * self.thrust.dt() + self.thrust == self.available_thrust*self.throttle / 100)

    # def set_reference(self,thrust,trajectory_mode=0,tau=1):
    #     self.reference.value = thrust
    #     self.thrust.SP = thrust
    #     self.thrust.TR_INIT = trajectory_mode
    #     self.thrust.TAU = tau

    # def cycle(self,available_thrust,cur_throttle,cur_thrust):
    #     self.available_thrust.value = available_thrust
    #     self.throttle.value = cur_throttle
    #     self.thrust.value = cur_thrust
    #     self.m.solve(disp=False)
    #     return (self.throttle.VALUE[1],self.thrust.VALUE[1])

class MPC_HoverTestVehicle(MPC_Whiplash_Thrust):
    def __init__(self,initial_mass,spoolup_time,thrust_specific_fuel_consumption,g,altitude,ctrl_horizon,ctrl_frequency,pred_horizon,pred_frequency):
        MPC_Whiplash_Thrust.__init__(self,spoolup_time,ctrl_horizon,ctrl_frequency,pred_horizon,pred_frequency)
        self.thrust.STATUS = 0
        self.m.options.CV_TYPE = 1
        self.g = self.m.Param(value=g,name='g')
        self.throttle.DCOST = 1
        #self.cd = self.m.Const(value=cd,name='cd')
        #self.Area = self.m.Const(value=Projected_Area,name='frontal_area')
        self.thrust_specific_fuel_consumption = self.m.Const(value=thrust_specific_fuel_consumption,name='thrust_specific_fuel_consumption') # kg per Newton second
        #self.air_density = self.m.Param(name='air_density')
        self.mass = self.m.CV(value=initial_mass,name='mass')
        self.mass.FSTATUS = 1
        self.mass.STATUS = 0
        self.alt = self.m.CV(value=altitude,name='altitude')
        self.alt.FSTATUS = 1
        self.alt.STATUS = 1
        self.alt.WSPHI = 10
        self.alt.WSPLO = 10
        self.vert_v = self.m.CV(name='vertical_velocity')
        self.vert_v.STATUS = 0
        self.vert_v.FSTATUS = 1
        self.alt_error = self.m.Intermediate(self.alt - self.reference,name='alt_error')
        self.sqr_speed_error = self.m.CV(value=-2*self.alt_error*self.g,name='sqr_speed_error')
        self.sqr_speed_error.FSTATUS = 0
        self.sqr_speed_error.STATUS = 1
        self.sqr_speed_error.SP = 0
        self.sqr_speed_error.SPHI = 1
        self.sqr_speed_error.SPLO = -1
        self.sqr_speed_error.WSPHI = 1000
        self.sqr_speed_error.WSPLO = 1000
        #self.drag_force = self.m.Intermediate(0.5 * self.vert_v * self.m.abs(self.vert_v) * self.air_density * self.cd,name='drag_force')
        #self.speed_limit = self.m.Intermediate(self.m.sqrt(self.g * self.m.abs(self.alt - self.reference)),name='speed_limit')
        #self.sqr_target_speed = self.m.Intermediate(-2*self.alt_error*self.g,name='sqr_target_speed')
        self.sqr_target_speed = self.m.SV(value=-2*self.alt_error*self.g,name='sqared_target_speed')

        self.m.Equations([
            self.alt.dt() == self.vert_v,
            #self.vert_v.dt() == ((self.thrust - 0.5 * self.vert_v * self.m.abs(self.vert_v) * self.air_density * self.cd) / self.mass) - self.g,
            self.mass * self.vert_v.dt() == self.thrust - self.g * self.mass, # without drag
            self.mass.dt() == -self.thrust_specific_fuel_consumption * self.thrust,
            self.sqr_speed_error == self.vert_v * self.m.abs(self.vert_v) - self.sqr_target_speed,
            self.sqr_target_speed == -2*self.alt_error*self.g
        ])

    def set_reference(self,alt,trajectory_mode=0,tau=1):
        self.reference.value = alt
        if self.m.options.CV_TYPE > 1:
            self.alt.SP = alt
        else:
            self.alt.SPHI = alt
            self.alt.SPLO = alt
        self.sqr_speed_error.TR_INIT = trajectory_mode
        self.sqr_speed_error.TAU = tau

    def update_state(self,mass,alt,vert_v):
        # update State Variables
        self.mass.MEAS = mass
        self.alt.MEAS = alt
        self.vert_v.MEAS = vert_v

    def update_parameters(self,available_thrust,g,air_density=1):
        # update Parameters
        #self.air_density.value = air_density
        self.available_thrust.value = available_thrust
        self.g.value = g

    def cycle(self,display=False,GUI=False,time_cycle=False):
        start = time.time()
        self.m.solve(disp=display,GUI=GUI)
        if time_cycle:
            print('meas_solve_time: ',time.time() - start)
        if self.m.options.APPINFO != 0:
            print("Controller Error: Errorcode %d with solver %d" % (self.m.options.APPINFO %self.m.options.SOLVER))
            return 0
        else:
            return self.throttle.NEWVAL
