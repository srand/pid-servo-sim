import time
import random

from PySide6.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QGroupBox, QLabel, QDoubleSpinBox, QCheckBox, QSizePolicy
)
from PySide6.QtCore import Qt, QTimer

from pid import PIDController
from scurve import SCurveProfile
from motor import MotorSim
from kalman import KalmanFilter
from clock_widget import ClockWidget


class MainWindow(QMainWindow):
    SIM_INTERVAL_MS = 20    # 50 Hz

    def __init__(self):
        super().__init__()
        self.setWindowTitle("PID + S-Curve Servo Controller")
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        # --- Simulation objects ---
        self._pid = PIDController(kp=2.0, ki=0.1, kd=0.05)
        self._scurve = SCurveProfile(max_vel=120.0, max_acc=240.0, enabled=False)
        self._motor = MotorSim(inertia=0.5, damping=0.3)
        self._kalman = KalmanFilter(q=1.0, r=25.0, inertia=0.5, damping=0.3)
        self._target_pos = 0.0      # unbounded target
        self._last_torque = 0.0
        self._last_time = time.monotonic()

        # --- Build UI ---
        self._clock = ClockWidget()
        panel = self._build_panel()

        central = QWidget()
        layout = QHBoxLayout(central)
        layout.addWidget(self._clock, stretch=3)
        layout.addWidget(panel, stretch=1)
        self.setCentralWidget(central)

        # --- Simulation timer ---
        self._timer = QTimer(self)
        self._timer.setInterval(self.SIM_INTERVAL_MS)
        self._timer.timeout.connect(self._step)
        self._timer.start()

    # ------------------------------------------------------------------
    # Panel construction
    # ------------------------------------------------------------------

    def _build_panel(self) -> QWidget:
        panel = QWidget()
        panel.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        vbox = QVBoxLayout(panel)
        vbox.setAlignment(Qt.AlignmentFlag.AlignTop)

        # --- S-curve group ---
        sc_group = QGroupBox("S-Curve Profile")
        sc_layout = QVBoxLayout(sc_group)

        self._scurve_enable = QCheckBox("Enable S-curve")
        self._scurve_enable.setChecked(False)
        self._scurve_enable.toggled.connect(self._on_scurve_enable)
        sc_layout.addWidget(self._scurve_enable)

        self._max_vel_box = self._make_spinbox(
            "Max Velocity (°/s)", sc_layout,
            value=120.0, min_=1.0, max_=1080.0, step=10.0,
            callback=self._on_max_vel
        )
        self._max_acc_box = self._make_spinbox(
            "Max Accel (°/s²)", sc_layout,
            value=240.0, min_=1.0, max_=4320.0, step=20.0,
            callback=self._on_max_acc
        )
        vbox.addWidget(sc_group)

        # --- PID group ---
        pid_group = QGroupBox("PID Parameters")
        pid_layout = QVBoxLayout(pid_group)

        self._kp_box = self._make_spinbox(
            "Kp", pid_layout, value=2.0, min_=0.0, max_=100.0, step=0.1,
            callback=lambda v: setattr(self._pid, 'kp', v)
        )
        self._ki_box = self._make_spinbox(
            "Ki", pid_layout, value=0.1, min_=0.0, max_=100.0, step=0.01,
            callback=lambda v: setattr(self._pid, 'ki', v)
        )
        self._kd_box = self._make_spinbox(
            "Kd", pid_layout, value=0.05, min_=0.0, max_=100.0, step=0.01,
            callback=lambda v: setattr(self._pid, 'kd', v)
        )
        vbox.addWidget(pid_group)

        # --- Motor group ---
        motor_group = QGroupBox("Motor Plant")
        motor_layout = QVBoxLayout(motor_group)

        self._inertia_box = self._make_spinbox(
            "Inertia", motor_layout, value=0.5, min_=0.01, max_=10.0, step=0.05,
            callback=lambda v: (setattr(self._motor, 'inertia', v),
                                setattr(self._kalman, 'inertia', v))
        )
        self._damping_box = self._make_spinbox(
            "Damping", motor_layout, value=0.3, min_=0.0, max_=10.0, step=0.05,
            callback=lambda v: (setattr(self._motor, 'damping', v),
                                setattr(self._kalman, 'damping', v))
        )
        self._noise_box = self._make_spinbox(
            "Noise (°)", motor_layout, value=0.0, min_=0.0, max_=50.0, step=0.1,
            callback=lambda v: setattr(self._motor, 'noise', v)
        )
        vbox.addWidget(motor_group)

        # --- Kalman filter group ---
        kalman_group = QGroupBox("Kalman Filter")
        kalman_layout = QVBoxLayout(kalman_group)

        self._kalman_enable = QCheckBox("Enable Kalman filter")
        self._kalman_enable.setChecked(False)
        self._kalman_enable.toggled.connect(self._on_kalman_enable)
        kalman_layout.addWidget(self._kalman_enable)

        self._kalman_q_box = self._make_spinbox(
            "Process noise (Q)", kalman_layout,
            value=1.0, min_=0.001, max_=1000.0, step=0.1,
            callback=lambda v: setattr(self._kalman, 'q', v)
        )
        self._kalman_r_box = self._make_spinbox(
            "Meas. noise (R)", kalman_layout,
            value=25.0, min_=0.001, max_=10000.0, step=1.0,
            callback=lambda v: setattr(self._kalman, 'r', v)
        )
        vbox.addWidget(kalman_group)

        # --- Status labels ---
        info_group = QGroupBox("Status")
        info_layout = QVBoxLayout(info_group)
        self._lbl_target = QLabel("Target:  0.0°")
        self._lbl_current = QLabel("Current: 0.0°")
        self._lbl_hint = QLabel("Press 0–9 to set target")
        self._lbl_hint.setStyleSheet("color: gray; font-size: 10px;")
        for lbl in (self._lbl_target, self._lbl_current, self._lbl_hint):
            info_layout.addWidget(lbl)
        vbox.addWidget(info_group)

        vbox.addStretch(1)
        return panel

    @staticmethod
    def _make_spinbox(label: str, layout: QVBoxLayout,
                      value: float, min_: float, max_: float,
                      step: float, callback) -> QDoubleSpinBox:
        row = QWidget()
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        lbl = QLabel(label)
        lbl.setMinimumWidth(100)
        spin = QDoubleSpinBox()
        spin.setRange(min_, max_)
        spin.setSingleStep(step)
        spin.setDecimals(3)
        spin.setValue(value)
        spin.valueChanged.connect(callback)
        row_layout.addWidget(lbl)
        row_layout.addWidget(spin)
        layout.addWidget(row)
        return spin

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_scurve_enable(self, checked: bool):
        self._scurve.enabled = checked
        if not checked:
            # When disabling S-curve mid-movement, snap desired to target
            self._scurve.set_target(self._motor.position,
                                    self._motor.velocity,
                                    self._target_pos)

    def _on_max_vel(self, value: float):
        self._scurve.max_vel = value

    def _on_max_acc(self, value: float):
        self._scurve.max_acc = value

    def _on_kalman_enable(self, checked: bool):
        if checked:
            self._kalman.reset(self._motor.position, self._motor.velocity)

    # ------------------------------------------------------------------
    # Simulation step
    # ------------------------------------------------------------------

    def _step(self):
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now
        if dt > 0.1:        # cap dt on first tick or after stall
            dt = 0.1

        desired = self._scurve.update(dt)
        if self._motor.noise > 0.0:
            measured = self._motor.position + random.gauss(0.0, self._motor.noise)
        else:
            measured = self._motor.position
        if self._kalman_enable.isChecked():
            measured = self._kalman.update(measured, dt, self._last_torque)
        torque = self._pid.update(desired, measured, dt)
        self._motor.apply(torque, dt)
        self._last_torque = torque

        current_display = self._motor.position % 360.0
        target_display = self._target_pos % 360.0

        self._clock.set_angles(current_display, target_display)
        self._lbl_current.setText(f"Current: {current_display:.1f}°")
        self._lbl_target.setText(f"Target:  {target_display:.1f}°")

    # ------------------------------------------------------------------
    # Key handling — 0–9 set target = n/10 * 360°
    # ------------------------------------------------------------------

    def keyPressEvent(self, event):
        key = event.key()
        if Qt.Key.Key_0 <= key <= Qt.Key.Key_9:
            n = key - Qt.Key.Key_0
            new_angle = n / 10.0 * 360.0
            current_angle = self._motor.position % 360.0
            delta = (new_angle - current_angle + 180.0) % 360.0 - 180.0
            self._target_pos = self._motor.position + delta
            self._pid.reset()
            self._scurve.set_target(self._motor.position,
                                    self._motor.velocity,
                                    self._target_pos)
        else:
            super().keyPressEvent(event)
