from casadi import *
import time

class MPC_Casadi:
    def __init__(self,spoolup,available_thrust,g,thrust_specific_fuel_consumption):
        self.opti = Opti()
        self.opti.solver('ipopt')
        self.N = 10
        self.T = 5
        self.dt = self.T / self.N
        # Parameters
        self.spoolup_time = self.opti.parameter()
        self.available_thrust = self.opti.parameter()
        self.thrust_specific_fuel_consumption = self.opti.parameter()
        self.g = self.opti.parameter()
        self.alt_target = self.opti.parameter()
        # Variables
        self.X = self.opti.variable(4,self.N+1) # state trajectory
        self.pos = self.X[0]
        self.vel = self.X[1]
        self.thrust = self.X[2]
        self.mass = self.X[3]
        self.U = self.opti.variable(1,self.N)
        # Set initial guess
        self.opti.set_initial(self.U,50)
        # Set Bounds
        self.opti.subject_to(self.opti.bounded(0,self.U,100))
        # Costfunction
        self.opti.minimize( (self.pos-self.alt_target)**2 )
        # Close Gaps in State
        for k in range(0,self.N):
            k1 = self.dynamic(self.X[:,k],self.U[:,k])
            k2 = self.dynamic(self.X[:,k] + k1 * self.dt/2, self.U[:,k])
            k3 = self.dynamic(self.X[:,k] + k2 * self.dt/2, self.U[:,k])
            k4 = self.dynamic(self.X[:,k] + k3 * self.dt, self.U[:,k])
            X_next = self.X[:,k] + self.dt/6 * (k1 + 2*k2 + 2*k3 + k4)
            self.opti.subject_to(self.X[:,k+1] == X_next)
        

    def dynamic(self,X,U):
        dxdt = MX(4,1)
        dxdt[0] = X[1]
        dxdt[1] = (X[2] / X[3]) - self.g
        dxdt[2] = (self.available_thrust * (U[0] / 100) - X[2]) / self.spoolup_time
        dxdt[3] = -self.thrust_specific_fuel_consumption * X[2]
        return dxdt

    def set_reference(self,alt_target):
        self.opti.set_value(self.alt_target,alt_target)

    def set_parameters(self,spoolup_time,available_thrust,thrust_specific_fuel_consumption,g):
        self.opti.set_value(self.spoolup_time,spoolup_time)
        self.opti.set_value(self.available_thrust,available_thrust)
        self.opti.set_value(self.thrust_specific_fuel_consumption,thrust_specific_fuel_consumption)
        self.opti.set_value(self.g,g)

    def update_state(self,pos,vel,thrust,mass):
        self.opti.subject_to(self.pos[0]==pos)
        self.opti.subject_to(self.vel[0]==vel)
        self.opti.subject_to(self.thrust[0]==thrust)
        self.opti.subject_to(self.mass[0]==mass)

    def cycle(self,time_cycle=False):
        start = time.time()
        sol = self.opti.solve()
        if time_cycle:
            print('meas_solve_time: ',time.time() - start)
        return sol.value(self.U)

    def cycle_(self,time_cycle=False):
        start = time.time()
        sol = self.opti.solve()
        if time_cycle:
            print('meas_solve_time: ',time.time() - start)
        return sol