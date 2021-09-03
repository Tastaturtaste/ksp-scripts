class pid_controller:
    def __init__(self,p_gain,i_gain,i_windup_limit,d_gain,sample_rate):
        self.p_last = 0
        self.i = 0
        self.p_gain = p_gain
        self.i_gain = i_gain
        self.i_windup_limit = i_windup_limit
        self.d_gain = d_gain
        self.sample_rate = sample_rate

    def update(self,current,target):
        p = current - target
        self.i += (self.p_last + p) / 2 / self.sample_rate
        if self.i > self.i_windup_limit:
            self.i = self.i_windup_limit
        elif self.i < -self.i_windup_limit:
            self.i = -self.i_windup_limit
        d = (p - self.p_last) * self.sample_rate
        self.p_last = p
        return p * self.p_gain + self.i * self.i_gain + d * self.d_gain