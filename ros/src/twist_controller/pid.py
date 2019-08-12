
MIN_NUM = float('-inf')
MAX_NUM = float('inf')

VAL_INIT = 9999999999.0  # just abnormal value

class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_error = 0.

        self.saturated = False
        self.last_val = VAL_INIT

    def reset(self):
        self.int_val = 0.0

    def step(self, error, sample_time):
        # integral on? or off?
        #set False 
        #if error is the same sign with last_val and it was saturated last time
        integral_on = True

        integral = self.int_val + error * sample_time;
        derivative = (error - self.last_error) / sample_time;

        if self.last_val != VAL_INIT:
            if (error > 0 and self.last_val > 0 ) or (error < 0 and self.last_val < 0 ): #same sign?
                if self.saturated == True:                
                    integral_on = False

        if integral_on == True:
            val = self.kp * error + self.ki * integral + self.kd * derivative;
        else:
            val = self.kp * error + self.kd * derivative;
            
        self.last_val = val

        if val > self.max:
            val = self.max
            self.saturated = True
        elif val < self.min:
            val = self.min
            self.saturated = True
        else:
            self.int_val = integral
            self.saturated = False
        self.last_error = error

        return val
