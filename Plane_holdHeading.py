import krpc
import time
conn = krpc.connect(name='flyingStraight')
vessel = conn.space_center.active_vessel
flight_info = vessel.flight(vessel.surface_reference_frame)
ap = vessel.auto_pilot
ap.reference_frame = vessel.surface_reference_frame
ap.target_pitch = 2.0
ap.target_heading = flight_info.heading
ap.target_roll = 0.0
time.sleep(2)
#ap.attenuation_angle = [2,2,6]
#ap.time_to_peak = [1,1,1]
#ap.auto_tune = False
#ap.overshoot = [0,0,0]
ap.engage()
print(ap.target_pitch, ap.pitch_error)
print(ap.target_roll, ap.roll_error)
print(ap.target_heading, ap.heading_error)
print(ap.pitch_pid_gains)
print(ap.roll_pid_gains)
print(ap.yaw_pid_gains)
while 1:
    #if ap.pitch_error > 4:
        #print(ap.pitch_error)
    print(ap.pitch_pid_gains)
    print(ap.roll_pid_gains)
    print(ap.yaw_pid_gains)
    continue
