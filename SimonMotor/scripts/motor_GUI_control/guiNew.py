"""Entry point for the motor-control GUI.

The application is split into focused modules:
  - config.py      constants: parameters, plot channels, serial/timing defaults
  - protocol.py    MIT/CAN wire protocol: mode bytes, pack/decode, framing
  - scaling.py     MotorScaling: command/reply ranges read from the motor
  - motor_link.py  MotorLink: serial IO + protocol + reader thread
  - plot_panel.py  PlotPanel: rolling-window live plots
  - panels.py      ControlPanel / ModePanel / ParamPanel input widgets
  - main_window.py MotorControlWindow: wires panels, link, and timers together
"""

import sys

from PyQt5.QtWidgets import QApplication

from main_window import MotorControlWindow


def main():
    app = QApplication(sys.argv)
    window = MotorControlWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
