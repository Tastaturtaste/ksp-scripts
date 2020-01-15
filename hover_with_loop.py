import krpc
import time
import math

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
#g = conn.add_stream(getattr,ves.orbit.body,'surface_gravity')
G = ves.orbit.body.gravitational_parameter
r = conn.add_stream(getattr,ves.orbit,'radius')

def g(G,r):
    return G / r**2

def speed_from_error(error,g):
    # Error should be positive while being above the reference, otherwise negative
    speed = math.sqrt(2*abs(error)*g)
    if error >= 0:
        return -speed
    else:
        return speed

def thrust_from_speed_error(mass,g,speed_error,K):
    ss_acc = g * mass
    return ss_acc * (1-speed_error*K)

def throttle_from_thrust(available_thrust,required_thrust):
    #returns value from 0 to 1
    return min(max(required_thrust,0) / available_thrust,1)

target_altitude = 500
ves.control.activate_next_stage()
while True:
    target_speed = speed_from_error(altitude() - target_altitude,g(G,r()))
    required_thrust = thrust_from_speed_error(mass(),g(G,r()),vert_speed()-target_speed,0.01)
    throttle = throttle_from_thrust(available_thrust(),required_thrust)
    #print('target speed: ',target_speed)
    print(g(G,r()))
    #print('throttle: ',throttle)
    ves.control.throttle = throttle