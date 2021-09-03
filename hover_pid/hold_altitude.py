import krpc
import time
import math
from PID import pid_controller

conn = krpc.connect(name='PID Hover Control')
ves = conn.space_center.active_vessel
ves.auto_pilot.engage()
ves.auto_pilot.target_pitch_and_heading(90, 90)
ut = conn.add_stream(getattr, conn.space_center, 'ut')
altitude = conn.add_stream(getattr, ves.flight(), 'surface_altitude')
vert_speed = conn.add_stream(getattr,ves.flight(ves.orbit.body.reference_frame), 'vertical_speed')
g = ves.orbit.body.surface_gravity

def speed_from_error(error,g):
    # Error should be positive while being above the reference, otherwise negative
    speed = math.sqrt(2*abs(error)*g)
    if error >= 0:
        return -speed
    else:
        return speed

target_altitude = 1000
ves.control.activate_next_stage()
p_gain = 0.1
i_gain = 0.01
i_windup_limit = 10
d_gain = 0.005
sample_rate = 10
pid = pid_controller(p_gain,i_gain,i_windup_limit,d_gain,sample_rate)
ves.control.throttle = 0.4
# while altitude() < (target_altitude*0.9):
#     pass
while True:
    target_speed = speed_from_error(altitude() - target_altitude,g)
    throttle = - pid.update(vert_speed(),target_speed)
    print('target speed: ',target_speed)
    print('throttle: ',throttle)
    if throttle < 0.0:
        throttle = 0.0
    elif throttle > 1.0:
        throttle = 1.0
    ves.control.throttle = throttle
    time.sleep(1.0 / sample_rate)