import krpc
import math

def circularization_delta_v(vessel):
    mu = vessel.orbit.body.gravitational_parameter
    r = vessel.orbit.apoapsis
    a1 = vessel.orbit.semi_major_axis
    a2 = r
    v1 = math.sqrt(mu*((2./r)-(1./a1)))
    v2 = math.sqrt(mu*((2./r)-(1./a2)))
    return v2 - v1

def circularization_burn_time(vessel):
    F = vessel.available_thrust
    Isp = vessel.specific_impulse * 9.82
    m0 = vessel.mass
    m1 = m0 / math.exp(circularization_delta_v(vessel)/Isp)
    flow_rate = F / Isp
    return (m0 - m1) / flow_rate
