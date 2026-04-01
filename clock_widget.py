import math
from PySide6.QtWidgets import QWidget
from PySide6.QtCore import Qt, QPointF
from PySide6.QtGui import QPainter, QPen, QColor, QFont


class ClockWidget(QWidget):
    """
    Draws a clock face with:
    - A solid red needle: current motor position
    - A dashed grey needle: target position

    0° = 12 o'clock, clockwise.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._current_angle = 0.0   # degrees, display angle (mod 360)
        self._target_angle = 0.0    # degrees, display angle (mod 360)
        self.setMinimumSize(300, 300)

    def set_angles(self, current: float, target: float) -> None:
        self._current_angle = current % 360.0
        self._target_angle = target % 360.0
        self.update()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _angle_to_point(self, center: QPointF, angle_deg: float,
                        length: float) -> QPointF:
        """Convert clock angle (0=12 o'clock, CW) to a QPointF."""
        rad = math.radians(angle_deg - 90.0)   # shift so 0° points up
        return QPointF(center.x() + length * math.cos(rad),
                       center.y() + length * math.sin(rad))

    # ------------------------------------------------------------------
    # Paint
    # ------------------------------------------------------------------

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        size = min(self.width(), self.height())
        margin = size * 0.06
        radius = size / 2.0 - margin
        center = QPointF(self.width() / 2.0, self.height() / 2.0)

        # --- Background circle ---
        painter.setPen(QPen(QColor("#333"), 3))
        painter.setBrush(QColor("#1a1a2e"))
        painter.drawEllipse(center, radius, radius)

        # --- Tick marks (12 major, no text) ---
        for i in range(12):
            angle = i * 30.0
            outer = self._angle_to_point(center, angle, radius * 0.95)
            inner = self._angle_to_point(center, angle,
                                         radius * (0.80 if i % 3 == 0 else 0.87))
            tick_width = 3 if i % 3 == 0 else 1
            painter.setPen(QPen(QColor("#aaaaaa"), tick_width))
            painter.drawLine(outer, inner)

        # --- Degree labels at 0 / 90 / 180 / 270 ---
        font = QFont("monospace", max(8, int(size * 0.035)))
        painter.setFont(font)
        painter.setPen(QColor("#888888"))
        for deg, label in ((0, "0°"), (90, "90°"), (180, "180°"), (270, "270°")):
            pt = self._angle_to_point(center, deg, radius * 0.68)
            painter.drawText(
                int(pt.x()) - 35, int(pt.y()) - 15, 70, 30,
                Qt.AlignmentFlag.AlignCenter, label
            )

        # --- Target needle (dashed grey, drawn first so current overlays it) ---
        target_tip = self._angle_to_point(center, self._target_angle, radius * 0.82)
        pen = QPen(QColor("#888888"), 2, Qt.PenStyle.DashLine)
        pen.setDashPattern([6, 4])
        painter.setPen(pen)
        painter.drawLine(center, target_tip)

        # Target arrowhead (small circle)
        painter.setBrush(QColor("#888888"))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(target_tip, 4, 4)

        # --- Current needle (solid red) ---
        current_tip = self._angle_to_point(center, self._current_angle, radius * 0.78)
        painter.setPen(QPen(QColor("#e63946"), 3))
        painter.setBrush(QColor("#e63946"))
        painter.drawLine(center, current_tip)

        # Needle base cap
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(center, 6, 6)

        # Needle tip cap
        painter.drawEllipse(current_tip, 4, 4)

        painter.end()
