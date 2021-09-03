import krpc
import time
import math

def circularization_delta_v(vessel_):
    mu = vessel_.orbit.body.gravitational_parameter
    r = vessel_.orbit.apoapsis
    a1 = vessel_.orbit.semi_major_axis
    a2 = r
    v1 = math.sqrt(mu*((2./r)-(1./a1)))
    v2 = math.sqrt(mu*((2./r)-(1./a2)))
    return v2 - v1

def circularization_burn_time(vessel_):
    F = vessel_.available_thrust
    Isp = vessel_.specific_impulse * 9.82
    m0 = vessel_.mass
    m1 = m0 / math.exp(circularization_delta_v(vessel_) / Isp)
    flow_rate = F / Isp
    return (m0 - m1) / flow_rate

def high_atmossphere_maneuvers(vessel_):
    for fairing in vessel_.parts.fairings:
        for module in fairing.part.modules:
            if module.name == 'ModuleProceduralFairing':
                module.trigger_event('Deploy')

    for panel in vessel_.parts.solar_panels:
        if panel.deployable and not panel.deployed:
            panel.deployed = True

    for radiator in vessel_.parts.radiators:
        if radiator.deployable and not radiator.deployed:
            radiator.deployed = True

def circularization_node(conn_, vessel_):
    ut = conn_.add_stream(getattr, conn_.space_center, 'ut')
    delta_v = circularization_delta_v(vessel_)
    node = vessel_.control.add_node(ut() + vessel_.orbit.time_to_apoapsis, prograde=delta_v)
    return node

def circularize_orbit(conn_, vessel_):
    ut = conn_.add_stream(getattr, conn_.space_center, 'ut')
    burn_time = circularization_burn_time(vessel_)
    vessel_.control.remove_nodes()
    node = circularization_node(conn_, vessel_)

    # Orientate ship
    print('Orientating ship for circularization burn')
    vessel_.control.throttle = 0.0
    vessel_.auto_pilot.engage()
    time.sleep(1)
    vessel_.auto_pilot.reference_frame = node.reference_frame
    vessel_.auto_pilot.target_direction = (0, 1, 0)
    vessel_.auto_pilot.wait()

    # Wait until burn
    print('Waiting until circularization burn')
    burn_ut = ut() + vessel_.orbit.time_to_apoapsis - (burn_time/2.)
    lead_time = 5
    conn_.space_center.warp_to(burn_ut - lead_time)

    # Execute burn
    print('Ready to execute burn')
    time_to_apoapsis = conn_.add_stream(getattr, vessel_.orbit, 'time_to_apoapsis')
    while time_to_apoapsis() - (burn_time/2.) > 0:
        pass
    print('Executing burn')
    vessel_.control.throttle = 1.0
    time.sleep(burn_time - 0.1)
    print('Fine tuning')
    vessel_.control.throttle = 0.05
    remaining_burn = conn_.add_stream(node.remaining_burn_vector, node.reference_frame)
    prev_remaining_burn = remaining_burn()
    while not remaining_burn()[1] > prev_remaining_burn[1]:
        prev_remaining_burn = remaining_burn()
        print(remaining_burn()[1])
        pass
    vessel_.control.throttle = 0.0
    node.remove()
    vessel_.auto_pilot.disengage()
    print('Orbit circularized')

def circularize_orbit_active_vessel():
    conn = krpc.connect()
    vessel = conn.space_center.active_vessel
    circularize_orbit(conn, vessel)
