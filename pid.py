class PIDController:
    def __init__(self, kp=2.0, ki=0.1, kd=0.05,
                 output_min=-1000.0, output_max=1000.0,
                 integral_max=100.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.integral_max = integral_max

        self._integral = 0.0
        self._prev_error = 0.0
        self._first = True

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._first = True

    def update(self, setpoint: float, measured: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        error = setpoint - measured

        # Reset integral when error changes sign (motor crossed setpoint).
        # Prevents wound-up integral from holding the motor in an overshot position.
        if not self._first and self._prev_error * error < 0:
            self._integral = 0.0

        # Integral with anti-windup (clamping)
        self._integral += error * dt
        self._integral = max(-self.integral_max,
                             min(self.integral_max, self._integral))

        # Derivative (on measurement change to avoid derivative kick)
        if self._first:
            derivative = 0.0
            self._first = False
        else:
            derivative = (error - self._prev_error) / dt

        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(self.output_min, min(self.output_max, output))
