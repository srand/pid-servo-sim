import signal
import sys
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from main_window import MainWindow


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    signal.signal(signal.SIGINT, lambda *_: app.quit())

    # Let Python process signals every 200ms so Ctrl-C is handled promptly
    timer = QTimer()
    timer.start(200)
    timer.timeout.connect(lambda: None)

    window = MainWindow()
    window.resize(800, 520)
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
