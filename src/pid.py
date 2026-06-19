from typing import List, Tuple
import math

class PID:
    def __init__(self, kp: float, ki : float, kd : float, setpoint: float = 0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        self._last_error = 0.0
        self._integral = 0.0
        self._derivative = 0.0

    def update(self, measurement: float, dt: float) -> float:
        error = self.setpoint - measurement
        self._integral += error * dt
        self._derivative = (error - self._last_error) / dt if dt > 0 else 0.0

        output = (self.kp * error) + (self.ki * self._integral) + (self.kd * self._derivative)

        self._last_error = error

        return output
    
    def reset(self):
        self._last_error = 0.0
        self._integral = 0.0
        self._derivative = 0.0