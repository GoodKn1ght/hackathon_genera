import time

class PIDController:
    def __init__(self, kp: float, ki: float, kd: float, max_output: float, min_output: float) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.clear()

    def clear(self) -> None:
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def update(self, current_value: float, target_value: float) -> float:
        current_time = time.time()
        if self.last_time is None:
            self.last_time = current_time
            return 0.0

        dt = current_time - self.last_time
        if dt <= 0.0:
            return 0.0

        error = target_value - current_value
        p_term = self.kp * error

        self.integral += error * dt
        i_term = self.ki * self.integral
        i_term = max(self.min_output, min(self.max_output, i_term))

        derivative = (error - self.last_error) / dt
        d_term = self.kd * derivative

        output = p_term + i_term + d_term
        self.last_error = error
        self.last_time = current_time

        return max(self.min_output, min(self.max_output, output))