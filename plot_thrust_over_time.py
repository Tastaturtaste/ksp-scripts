import krpc
import time
import math
import numpy as np
import matplotlib.pyplot as plt

def get_thrust_over_time(duration,resolution):
    conn = krpc.connect(name='connection')
    ves = conn.space_center.active_vessel
    ves.auto_pilot.engage()
    ves.auto_pilot.target_pitch_and_heading(90, 90)
    ut = conn.add_stream(getattr, conn.space_center, 'ut')
    thrust = conn.add_stream(getattr, ves, 'thrust')
    ves.control.activate_next_stage()
    time.sleep(1)
    t0 = ut()
    ves.control.throttle = 1.0
    data = np.array([[ut()-t0],[thrust()]])
    while ut() - t0 < duration:
        time.sleep(resolution)
        data = np.append(data,np.array([[ut()-t0],[thrust()]]),axis=1)

    return data

def simulate_thrust_over_time(duration,resolution):
    time = [i for i in np.arange(0,duration,resolution)]
    T = 2
    K = 79651.25
    data = np.array([[time[0]],[0.0]])
    for t in time:
	    data = np.append(data,np.array([[t],[K*(1-math.exp(-t/T))]]),axis=1)

    return data

measured = get_thrust_over_time(8,0.01)
simulated = simulate_thrust_over_time(8,0.01)
plt.plot(measured[0],measured[1],'r-')
plt.plot(simulated[0],simulated[1],'g-')
plt.show()