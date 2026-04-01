class SCurveProfile:
    """
    Trapezoidal velocity profile (symmetric acceleration / deceleration).
    Handles the triangular case when the distance is too short to reach max_vel.
    When disabled, immediately returns the raw target position.
    """

    def __init__(self, max_vel=120.0, max_acc=240.0, enabled=True):
        self.max_vel = max_vel      # deg/s
        self.max_acc = max_acc      # deg/s²
        self.enabled = enabled

        # Profile state
        self._start_pos = 0.0
        self._target_pos = 0.0
        self._direction = 1.0       # +1 or -1
        self._t_acc = 0.0           # duration of acceleration phase
        self._t_cruise = 0.0        # duration of cruise phase
        self._t_dec = 0.0           # duration of deceleration phase
        self._peak_vel = 0.0        # actual peak velocity (may be < max_vel)
        self._elapsed = 0.0
        self._distance = 0.0
        self._done = True

        # Current profiled position (updated by update())
        self._current_pos = 0.0

    @property
    def done(self) -> bool:
        return self._done

    def set_target(self, current_pos: float, current_vel: float,
                   target_pos: float) -> None:
        """Compute a new trapezoidal profile from (current_pos) to target_pos."""
        self._current_pos = current_pos
        self._start_pos = current_pos
        self._target_pos = target_pos
        self._elapsed = 0.0

        delta = target_pos - current_pos
        if abs(delta) < 1e-6:
            self._done = True
            return

        self._done = False
        self._direction = 1.0 if delta >= 0 else -1.0
        self._distance = abs(delta)

        # Minimum distance required to reach max_vel
        d_min = self.max_vel ** 2 / self.max_acc

        if self._distance >= d_min:
            # Full trapezoidal: acc + cruise + dec
            self._peak_vel = self.max_vel
            self._t_acc = self.max_vel / self.max_acc
            self._t_dec = self._t_acc
            d_acc = 0.5 * self.max_acc * self._t_acc ** 2
            d_cruise = self._distance - 2 * d_acc
            self._t_cruise = d_cruise / self.max_vel
        else:
            # Triangular: no cruise phase
            self._peak_vel = (self._distance * self.max_acc) ** 0.5
            self._t_acc = self._peak_vel / self.max_acc
            self._t_dec = self._t_acc
            self._t_cruise = 0.0

    def update(self, dt: float) -> float:
        """Advance the profile by dt seconds; return the desired position."""
        if not self.enabled:
            self._current_pos = self._target_pos
            self._done = True
            return self._target_pos

        if self._done:
            return self._current_pos

        self._elapsed += dt
        t = self._elapsed
        t1 = self._t_acc
        t2 = t1 + self._t_cruise
        t3 = t2 + self._t_dec

        if t <= t1:
            # Acceleration phase
            s = 0.5 * self.max_acc * t ** 2
        elif t <= t2:
            # Cruise phase
            d_acc = 0.5 * self.max_acc * t1 ** 2
            s = d_acc + self._peak_vel * (t - t1)
        elif t < t3:
            # Deceleration phase
            d_acc = 0.5 * self.max_acc * t1 ** 2
            d_cruise = self._peak_vel * self._t_cruise
            dt_dec = t - t2
            s = d_acc + d_cruise + self._peak_vel * dt_dec - 0.5 * self.max_acc * dt_dec ** 2
        else:
            # Done
            s = self._distance
            self._done = True

        self._current_pos = self._start_pos + self._direction * min(s, self._distance)
        return self._current_pos
