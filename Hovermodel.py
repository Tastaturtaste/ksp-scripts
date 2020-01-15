from gekko import GEKKO

class Model:
    def __init__(self,vessel,refframe):
        self.vessel = vessel
        self.refframe = refframe
        self.m = GEKKO()
        self.height = self.m.SV(vessel.flight().mean_altitude)
        self.vert_speed = self.m.SV()
        self.vert_acce = self.m.SV()
        self.thrust = self.m.SV()
        self.engine_spool_time = self.m.Const(2.0)
        self.max_thrust = self.m.Param(vessel.available_thrust)
        self.mass = self.m.SV(vessel.mass)
        self.throttle = self.m.MV()
        self.throttle.STATUS = 0
        self.m.Equations([
            self.vert_speed == self.height.dt,
            self.vert_acce == self.vert_speed.dt,
        ])