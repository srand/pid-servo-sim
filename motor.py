class MotorSim:
    """
    Second-order simulated servo plant.
    Dynamics:  accel = (torque - damping * vel) / inertia
    Integrates accel → vel → position using forward Euler.
    Position is unbounded (no mod 360) so the PID stays consistent.
    """

    def __init__(self, inertia=0.5, damping=0.3):
        self.inertia = inertia   # kg·m² equivalent
        self.damping = damping   # viscous friction coefficient
        self.noise = 0.0         # std dev of Gaussian sensor noise (degrees)

        self.position = 0.0     # degrees (unbounded)
        self.velocity = 0.0     # degrees/second

    def reset(self, position=0.0, velocity=0.0):
        self.position = position
        self.velocity = velocity

    def apply(self, torque: float, dt: float) -> None:
        """Integrate one time step."""
        if dt <= 0.0:
            return
        accel = (torque - self.damping * self.velocity) / self.inertia
        self.velocity += accel * dt
        self.position += self.velocity * dt
