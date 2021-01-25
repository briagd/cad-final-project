
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID():
    def __init__(self, k_p, k_i, k_d, min=MIN_NUM, max=MAX_NUM):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.min = min
        self.max = max

        self.int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0

    def step(self, error, sampling_time):

        integral = self.int_val + error * sampling_time
        derivative = (error - self.last_error) / sampling_time

        val = self.k_p * error + self.k_i * integral + self.k_d * derivative

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral

        self.last_error = error

        return val