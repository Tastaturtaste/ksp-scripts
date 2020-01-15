import krpc
import numpy as np
import matplotlib.pyplot as plt

def check_vel_formula(approx_v0):
    conn = krpc.connect(name='PID Hover Control')
    ves = conn.space_center.active_vessel
    ves.auto_pilot.engage()
    ves.auto_pilot.target_pitch_and_heading(90, 90)
    altitude = conn.add_stream(getattr, ves.flight(), 'surface_altitude')
    vert_speed = conn.add_stream(getattr,ves.flight(ves.orbit.body.reference_frame), 'vertical_speed')
    thrust = conn.add_stream(getattr,ves,'thrust')
    g = ves.orbit.body.surface_gravity

    ves.control.activate_next_stage()
    ves.control.throttle = 1
    while vert_speed() < approx_v0:
        print('vert_speed: ',vert_speed())
    ves.control.throttle = 0
    while thrust() > 0:
        print('thrust: ',thrust())
    v0 = vert_speed()
    x0 = altitude()
    estimated_distance = v0**2 / (g * 2)
    print("dx should be ", estimated_distance)
    while vert_speed() > 0:
        pass
    true_distance = altitude() - x0
    print("dx was ", true_distance)
