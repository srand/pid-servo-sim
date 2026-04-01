class KalmanFilter:
    """
    Motor-model-aware 2-state Kalman filter (position + velocity).

    Uses the actual motor plant equation in the prediction step:
        pos_new = pos + vel * dt
        vel_new = vel * (1 - damping*dt/inertia) + torque*dt/inertia

    This eliminates the systematic prediction error that occurs with a
    naive constant-velocity model during acceleration phases.

    Process noise Q models unmodeled disturbances (random acceleration):
        Q = q * [[dt³/3,  dt²/2],
                 [dt²/2,  dt   ]]

    Tune q vs r:
      High q / low r  → trust measurements more, less lag
      Low  q / high r → trust model more, heavier noise rejection
      A good starting point: r = noise_std²  (e.g. noise=5° → r=25)
    """

    def __init__(self, q: float = 1.0, r: float = 25.0,
                 inertia: float = 1.0, damping: float = 0.0):
        self.q = q
        self.r = r
        self.inertia = inertia
        self.damping = damping

        self._pos = 0.0
        self._vel = 0.0
        # 2×2 error covariance stored as four scalars
        self._p00 = 1.0
        self._p01 = 0.0
        self._p10 = 0.0
        self._p11 = 1.0

    def reset(self, position: float = 0.0, velocity: float = 0.0) -> None:
        self._pos = position
        self._vel = velocity
        self._p00 = 1.0
        self._p01 = 0.0
        self._p10 = 0.0
        self._p11 = 1.0

    def update(self, measurement: float, dt: float, torque: float = 0.0) -> float:
        """Ingest one noisy position measurement; return filtered position estimate."""
        if dt <= 0.0:
            return self._pos

        # Motor state-transition coefficients
        inv_i = 1.0 / max(self.inertia, 1e-6)
        a11 = 1.0 - self.damping * dt * inv_i   # velocity decay factor per step

        # ---- Predict ----------------------------------------
        # A = [[1, dt], [0, a11]]
        pos_pred = self._pos + self._vel * dt
        vel_pred = self._vel * a11 + torque * dt * inv_i

        # P_pred = A·P·Aᵀ + Q  (kinematic process noise for random-acceleration model)
        dt2 = dt * dt
        p00_pred = (self._p00 + dt * (self._p01 + self._p10) + dt2 * self._p11
                    + self.q * dt2 * dt / 3.0)
        p01_pred = a11 * (self._p01 + dt * self._p11) + self.q * dt2 / 2.0
        p10_pred = a11 * (self._p10 + dt * self._p11) + self.q * dt2 / 2.0
        p11_pred = a11 * a11 * self._p11 + self.q * dt

        # ---- Update -----------------------------------------
        # H = [1, 0]  →  S = P_pred[0,0] + R
        s = p00_pred + self.r
        k0 = p00_pred / s
        k1 = p10_pred / s

        innov = measurement - pos_pred

        self._pos = pos_pred + k0 * innov
        self._vel = vel_pred + k1 * innov

        # P = (I − K·H)·P_pred
        self._p00 = (1.0 - k0) * p00_pred
        self._p01 = (1.0 - k0) * p01_pred
        self._p10 = p10_pred - k1 * p00_pred
        self._p11 = p11_pred - k1 * p01_pred

        return self._pos
