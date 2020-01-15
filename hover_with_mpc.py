import krpc
import time
import math
from MPC import MPC_HoverTestVehicle as HTV

conn = krpc.connect(name='connection')
ves = conn.space_center.active_vessel
ves.auto_pilot.engage()
ves.auto_pilot.target_pitch_and_heading(90, 90)
ut = conn.add_stream(getattr, conn.space_center, 'ut')
altitude = conn.add_stream(getattr, ves.flight(), 'mean_altitude')
vert_speed = conn.add_stream(getattr,ves.flight(ves.orbit.body.reference_frame), 'vertical_speed')
mass = conn.add_stream(getattr,ves,'mass')
air_density = conn.add_stream(getattr,ves.flight(),'atmosphere_density')
available_thrust = conn.add_stream(getattr,ves,'available_thrust')
thrust = conn.add_stream(getattr,ves,'thrust')
G = ves.orbit.body.gravitational_parameter
r = conn.add_stream(getattr,ves.orbit,'radius')

def g():
    return G / r()**2

def dm():
    before = mass()
    time.sleep(1)
    return (before - mass()) / available_thrust()

def speed_from_error(error,max_speed,error_at_max_speed):
    expo = math.log(max_speed - 1) / error_at_max_speed
    if error >= 0:
        return -math.exp(error*expo) + 1
    else:
        return math.exp(-error*expo) - 1

target_altitude = 500
ctrl_rate = 4
ves.control.activate_next_stage()
mpc = HTV(mass(),0.1, 5.5 * 5 / 70000,g(),altitude(),2,ctrl_rate,8,1)
mpc.m.options.MEAS_CHK = 1 # set to 0 as soon as bug is fixed
#mpc.alt.VDVL = 10**5
#mpc.alt.VLHI = 10**5
#mpc.alt.VLLO = 10**5
#mpc.alt.VLACTION = 0
mpc.update_state(mass(),altitude(),vert_speed())
mpc.update_parameters(available_thrust(),g())
mpc.set_reference(500)
print('solver: ',mpc.m.options.SOLVER)
#try:
mpc.m.open_folder()
ves.control.throttle = mpc.cycle(display=False,GUI=False) * 0.01
ves.control.activate_next_stage()
# while altitude() < (target_altitude*0.9):
#     pass
while True:
    start = time.time()
    v = vert_speed()
    alt = altitude()
    mpc.update_state(mass(),altitude(),vert_speed())
    mpc.update_parameters(available_thrust(),g())
    throttle_percentage = mpc.cycle(display=False,GUI=False)
    ves.control.throttle = throttle_percentage / 100
    #print(throttle_percentage)
    #print('Objective Function: ', mpc.m.options.OBJFCNVAL)
    #print('Time to solve: ', mpc.m.options.SOLVETIME)
    #print('model alt: ', mpc.alt.VALUE)
    print('model alt bias: ',mpc.alt.BIAS)
    #print('solver time: ',mpc.m.options.SOLVETIME)
    #print('measured speed: ',v)
    #print('measured altitude: ',alt)
    time.sleep(max(1.0/ctrl_rate - (time.time() - start), 0))

# except:
#     print('APPINFO: ', mpc.m.options.APPINFO)