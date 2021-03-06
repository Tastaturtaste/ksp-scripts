import krpc
import time
import math
from MPC_casadi import MPC_Casadi as HTV

# Enable Auto-accept new clients in the advanced server settings, otherwise the connection will time out
conn = krpc.connect(name='connection')
ves = conn.space_center.active_vessel
ves.auto_pilot.engage()
# ves.auto_pilot.target_pitch_and_heading(90, 90)
ves.auto_pilot.target_pitch = 90
ves.auto_pilot.target_roll = 0
ut = conn.add_stream(getattr, conn.space_center, 'ut')
altitude = conn.add_stream(getattr, ves.flight(), 'mean_altitude')
surf_altitude = conn.add_stream(getattr, ves.flight(), 'surface_altitude')
vert_speed = conn.add_stream(getattr, ves.flight(
    ves.orbit.body.reference_frame), 'vertical_speed')
mass = conn.add_stream(getattr, ves, 'mass')
air_density = conn.add_stream(getattr, ves.flight(), 'atmosphere_density')
available_thrust = conn.add_stream(getattr, ves, 'available_thrust')
thrust = conn.add_stream(getattr, ves, 'thrust')
G = ves.orbit.body.gravitational_parameter
planet_radius = ves.orbit.body.equatorial_radius
r = conn.add_stream(getattr, ves.orbit, 'radius')


def g(height, planet_radius):
    return G / (height + planet_radius)**2


def dm():
    before = mass()
    time.sleep(1)
    return (before - mass()) / available_thrust()


def speed_from_error(error, max_speed, error_at_max_speed):
    expo = math.log(max_speed - 1) / error_at_max_speed
    if error >= 0:
        return -math.exp(error*expo) + 1
    else:
        return math.exp(-error*expo) - 1


Tpred = 5
intervalsize = 0.2
Topt = 3*intervalsize

mass_loss = 53.98  # kg/s for Aerospike
# mass_loss = 5.5*5/70000
spoolup_time = 0.1

platform_heigth = altitude()
# Offset to account for compressed landing legs
landing_target_heigth = platform_heigth + 0.5
hover_time_start = 0.0
landing = False
hover_duration = 8.0
target_altitude = 500
ves.control.activate_next_stage()
mpc = HTV(Tpred, intervalsize, T_opt=Topt)
mpc.set_parameters(spoolup_time, available_thrust(),
                   mass_loss, g(altitude(), planet_radius))
mpc.set_altitude(target_altitude)
throttle_pct = mpc.warmup([altitude(), vert_speed(), mass(), thrust()])
ves.control.throttle = throttle_pct
time.sleep(spoolup_time)
ves.control.activate_next_stage()
ves.control.rcs = True
while True:
    start = time.time()
    v = vert_speed()
    alt = altitude()
    m = mass()
    thr = thrust()

    if (alt + 1) > target_altitude:
        if hover_time_start == 0.0:
            hover_time_start = time.time()
        elif (time.time() - hover_time_start) > hover_duration:
            mpc.set_altitude(landing_target_heigth)
            landing = True

    if landing and (alt-0.5) < landing_target_heigth:
        ves.control.throttle = 0.0
        break
    mpc.set_parameters(spoolup_time, available_thrust(),
                       mass_loss, g(alt, planet_radius))
    throttle_pct = mpc.cycle([alt, v, m, thrust()])
    time_control_end = time.time()
    spare_time = intervalsize - (time_control_end - start)
    if spare_time < 0:
        print('mpc took {} to long!'.format(spare_time))
    time.sleep(max(spare_time, 0))
    ves.control.throttle = throttle_pct / 100
