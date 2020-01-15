from BasicSetup import setup
from Calculate import *
from Maneuvers import *
import krpc
import math
import time
import sys

turn_start_altitude = 200
turn_end_altitude = 45000
target_altitude = 90000

#def do_high_atmossphere_stuff():
#    high_atmossphere_maneuvers(vessel)
def launch_to_orbit(booster_decouple_stage=-1, initial_thrust=0.0):
    ConVes = setup()
    conn = ConVes[0]
    vessel = ConVes[1]

    #setting up streams
    ut = conn.add_stream(getattr, conn.space_center, 'ut')
    altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
    """
    def alti_callback(x):
        if x > 60000:
            high_atmossphere_maneuvers(vessel)

    altitude.add_callback(alti_callback)
    """
    apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
    if booster_decouple_stage == -1:
        stage_3_resources = vessel.resources_in_decouple_stage(stage=0, cumulative=True)
    else:
        stage_3_resources = vessel.resources_in_decouple_stage(stage=booster_decouple_stage, cumulative=False)
    srb_fuel = conn.add_stream(stage_3_resources.amount, 'SolidFuel')
    available_thrust = conn.add_stream(getattr, vessel, 'available_thrust')
    available_thrust.start()

    # Pre-Launch BasicSetup
    vessel.control.sas = False
    vessel.control.rcs = False
    print("Setting initial thrust to {}".format(initial_thrust))
    vessel.control.throttle = initial_thrust

    # Countdown...
    print('3...')
    time.sleep(1)
    print('2...')
    time.sleep(1)
    print('1...')
    time.sleep(1)
    print('Launch!')

    # Activate the first stage
    vessel.control.activate_next_stage()
    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_pitch_and_heading(90, 90)

    # Main ascent loop
    srbs_separated = True if booster_decouple_stage == -1 else False
    turn_angle = 0
    available_thrust_at_launch = available_thrust()
    while True:

        # Gravity turn
        if altitude() > turn_start_altitude and altitude() < turn_end_altitude:
            frac = ((altitude() - turn_start_altitude) /
                    (turn_end_altitude - turn_start_altitude))
            new_turn_angle = frac * 90
            if abs(new_turn_angle - turn_angle) > 0.5:
                turn_angle = new_turn_angle
                vessel.auto_pilot.target_pitch_and_heading(90-turn_angle, 90)

        # Separate SRBs when finished
        if not srbs_separated:
            if srb_fuel() < 0.1 or available_thrust_at_launch*0.9 > available_thrust():
                vessel.control.throttle = 0.0
                vessel.control.activate_next_stage()
                vessel.control.throttle = 1.0
                srbs_separated = True
                print('SRBs separated')

        # Decrease throttle when approaching target apoapsis
        if apoapsis() > target_altitude*0.9:
            print('Approaching target apoapsis')
            break

    # Disable engines when target apoapsis is reached
    vessel.control.throttle = 0.25
    while apoapsis() < target_altitude:
        pass
    print('Target apoapsis reached')
    vessel.control.throttle = 0.0

    # Wait until out of atmosphere
    print('Coasting out of atmosphere')
    while altitude() < 70500:
        pass
    high_atmossphere_maneuvers(vessel)
    time.sleep(1)

    # Plan circularization burn (using vis-viva equation)
    print('Planning circularization burn')
    delta_v = circularization_delta_v(vessel)
    node = vessel.control.add_node(ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

    # Calculate burn time (using rocket equation)
    burn_time = circularization_burn_time(vessel)

    # Orientate ship
    print('Orientating ship for circularization burn')
    vessel.auto_pilot.reference_frame = node.reference_frame
    vessel.auto_pilot.target_direction = (0, 1, 0)
    vessel.auto_pilot.wait()

    # Wait until burn
    print('Waiting until circularization burn')
    burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burn_time/2.)
    lead_time = 5
    conn.space_center.warp_to(burn_ut - lead_time)

    # Execute burn
    print('Ready to execute burn')
    time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
    while time_to_apoapsis() - (burn_time/2.) > 0:
        pass
    print('Executing burn')
    vessel.control.throttle = 1.0
    time.sleep(burn_time - 0.1)
    print('Fine tuning')
    vessel.control.throttle = 0.05
    remaining_burn = conn.add_stream(node.remaining_burn_vector, node.reference_frame)
    prev_remaining_burn = remaining_burn()
    while not remaining_burn()[1] > prev_remaining_burn[1]:
        prev_remaining_burn = remaining_burn()
        print(remaining_burn()[1])
        pass
    vessel.control.throttle = 0.0
    node.remove()

    print('Launch complete')


if __name__ == "__main__":
    stage = -1;
    initial_thrust = 0.0
    """
    if sys.argv[1]:
        stage = int(sys.argv[1])
    if sys.argv[2]:
        initial_thrust = float(sys.argv[2])
    """
    stage = int(input("Enter the Number of the Stage, in which your SRB's decouple. If you have no SRB's enter '-1': "))
    initial_thrust = float(input("Enter your initial thrust setting between 0 and 1: "))
    launch_to_orbit(stage, initial_thrust)
