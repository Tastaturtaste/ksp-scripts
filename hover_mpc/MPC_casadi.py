from casadi import *
import time
import numpy as np


class MPC_Casadi:
    def __init__(self, T_pred, dt, T_opt=-1):
        self.N_pred = int(T_pred / dt)
        if T_opt < 0:
            self.N_opt = self.N_pred
        else:
            self.N_opt = int(T_opt / dt)
        self.ns = 4  # number of states
        self.nu = 1  # number of inputs
        self.np = 4  # number of parameters
        self.last_throttle_output = 0
        self.dt = dt
        h = SX.sym('h')
        v = SX.sym('v')
        m = SX.sym('m')
        thrust = SX.sym('thrust')
        x = vertcat(h, v, m, thrust)
        spoolup_time = SX.sym('spoolup_time')
        max_thrust = SX.sym('max_thrust')
        mass_loss = SX.sym('mass_loss')
        g = SX.sym('g')
        p = vertcat(spoolup_time, max_thrust,
                    mass_loss, g)
        u = SX.sym('throttle')  # between 0 and 100
        htarget = SX.sym('htarget')
        # h_delta = h - htarget
        # v_target_sq = -2*g*h_delta
        # j = (sign(v)*v*v-v_target_sq)**2 + h_delta**2 + u**2
        j = (h-htarget)**2
        hdot = v
        vdot = (thrust / m) - g
        mdot = -thrust/max_thrust * mass_loss
        thrustdot = (max_thrust * (u/100) - thrust) / spoolup_time
        xdot = vertcat(hdot, vdot, mdot, thrustdot)
        dae = {'x': x, 'p': vertcat(htarget, u, p), 'ode': xdot, 'quad': j}
        X = MX.sym('X', x.sparsity())
        P = MX.sym('P', p.sparsity())
        U = MX.sym('U', u.sparsity())
        TP = MX.sym('TP')
        intg = integrator('intg', 'rk', dae, {
                          'number_of_finite_elements': 4, 'expand': True, 'tf': self.dt})
        F = intg(x0=X, p=vertcat(TP, U, P))
        xf = F['xf']
        qf = F['qf']
        f = Function('f', [X, TP, U, P], [xf, qf], [
                     'x0', 'TP', 'U', 'P'], ['xf', 'qf'])
        g = []
        self.lbg = []
        self.ubg = []
        J = 0
        lbx = [0, -inf, 0, 0]
        ubx = [inf, inf, inf, inf]
        lbu = [0]
        ubu = [100]
        X = MX.sym('X', self.ns, self.N_pred + 1)
        U_opt = MX.sym('U_opt', self.nu, self.N_opt)
        U_hold = MX.sym('U_hold', self.nu, self.N_pred-self.N_opt)
        U = horzcat(U_opt, U_hold)
        U0 = MX.sym('U0', self.nu, 1)
        X0 = MX.sym('X0', self.ns)
        P = MX.sym('P', self.np)
        TP = MX.sym('TP', self.N_pred)
        Xk = X[:, 0]
        g += [Xk - X0]
        self.lbg += [DM.zeros(Xk.sparsity())]
        self.ubg += [DM.zeros(Xk.sparsity())]
        g += [U_opt[:, 0]-U0]
        self.lbg += [DM.zeros(self.nu, 1)]
        self.ubg += [DM.zeros(self.nu, 1)]
        # Setup prediction horizon >= opt horizon
        g += [reshape(U_hold, U_hold.numel(), 1) -
              reshape(U_opt[:, -1], self.nu, 1)]
        self.lbg += [DM.zeros(U_hold.numel())]
        self.ubg += [DM.zeros(U_hold.numel())]

        for k in range(self.N_pred):
            # New NLP variable for the control
            Uk = U[:, k]
            # Target Point for this timestep
            TPk = TP[k]
            # Integrate till the end of the interval
            Fk = f(x0=Xk, TP=TPk, U=Uk, P=P)
            Xk_end = Fk['xf']
            J = J+Fk['qf']
            # New NLP variable for state at end of interval
            Xk = X[:, k+1]
            # Add equality constraint
            g += [Xk_end-Xk]
            self.lbg += [DM.zeros(Xk.sparsity())]
            self.ubg += [DM.zeros(Xk.sparsity())]

        self.lbg = vertcat(*self.lbg)
        self.ubg = vertcat(*self.ubg)
        opt_var = vertcat(
            reshape(X, X.numel(), 1),
            reshape(U, U.numel(), 1))
        prob = {'f': J, 'x': opt_var, 'g': vertcat(
            *g), 'p': vertcat(X0, U0, P, TP)}

        if True:

            solver_opts = {'ipopt.print_level': 2, 'print_time': 0,
                           'ipopt.warm_start_init_point': 'yes', 'ipopt.max_iter': 30}
            self.solver = nlpsol('solver', 'ipopt', prob, solver_opts)
        else:
            qpoases_opts = {'printLevel': 'low'}
            self.solver = qpsol('solver', 'qpoases', prob, qpoases_opts)

        self.lbw = vertcat(
            repmat(lbx, X.size2(), 1),
            repmat(lbu, U.size2(), 1))
        self.ubw = vertcat(
            repmat(ubx, X.size2(), 1),
            repmat(ubu, U.size2(), 1))

    def set_altitude(self, target_alt):
        self.target_alt = target_alt

    def set_parameters(self, spoolup_time, available_thrust, mass_loss, g):
        self.params = DM([spoolup_time, available_thrust, mass_loss, g])

    def warmup(self, x0):
        # x0: h,v,m,thrust
        g = self.params[3]
        ref_trajectory = generate_reference_trajectory(
            x0[0], self.target_alt, g, self.dt, self.N_pred)
        self.w0 = vertcat(
            reshape(DM.ones(self.ns, self.N_pred+1)
                    * x0, self.ns*(self.N_pred+1), 1),
            reshape(DM.ones(self.nu, self.N_pred)*0, self.nu*self.N_pred, 1))
        self.sol = self.solver(x0=self.w0, lbx=self.lbw, ubx=self.ubw, lbg=self.lbg,
                               ubg=self.ubg, p=vertcat(x0, self.last_throttle_output, self.params, ref_trajectory))
        self.last_throttle_output = self.sol['x'].full().flatten()[
            (self.N_pred+1)*self.ns + 1]
        return self.last_throttle_output

    def cycle(self, x0, time_cycle=False):
        # x0: h,v,m,thrust
        start = time.time()
        g = self.params[3]
        ref_trajectory = generate_reference_trajectory(
            x0[0], self.target_alt, g, self.dt, self.N_pred)
        self.w0 = vertcat(x0, self.sol['x'][self.ns:])
        self.sol = self.solver(x0=self.w0, lbx=self.lbw, ubx=self.ubw, lbg=self.lbg, ubg=self.ubg, p=vertcat(x0, self.last_throttle_output, self.params, ref_trajectory),
                               lam_x0=self.sol['lam_x'], lam_g0=self.sol['lam_g'])
        if time_cycle:
            print('meas_solve_time: ', time.time() - start)
        self.last_throttle_output = self.sol['x'].full().flatten()[
            (self.N_pred+1)*self.ns + 1]
        return self.last_throttle_output


def generate_reference_trajectory(alt_current, alt_target, g, dt, N_pred):
    # PT1-Trajektorie mit K = 1 und T = alt_delta/v_max.
    # v_max ist die Geschwindigkeit, welche gerade durch die Gravitation abgebaut ist wenn die Sollhöhe erreicht ist.
    # Wenn die Höhenabweichung während des Prädiktionshorizonts vollständig mit der Erdbeschleunigung abgebaut werden kann, wird nur die Sollhöhe als Referenz ausgegeben.
    alt_delta = alt_target - alt_current
    v_max = sign(alt_delta)*sqrt(2*g*abs(alt_delta))
    if abs(alt_delta) > 10:
        traj = DM([alt_current + alt_delta*(1-exp(-i*dt * v_max/alt_delta))
                   for i in range(0, N_pred)])
    else:
        traj = DM.ones(N_pred, 1) * alt_target
    return traj
