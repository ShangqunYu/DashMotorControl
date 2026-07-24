"""Top-level window: composes the input panels and plot panel, owns the
MotorLink, and wires signals/timers between them."""

from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QGroupBox,
)
from PyQt5.QtCore import QTimer

import protocol
from config import (
    DEFAULT_PORT, DEFAULT_BAUDRATE, PLOT_REFRESH_MS, AUTO_SEND_MS,
)
from motor_link import MotorLink
from plot_panel import PlotPanel
from panels import ControlPanel, ModePanel, ParamPanel
from scaling import MotorScaling


class MotorControlWindow(QMainWindow):
    def __init__(self, port=DEFAULT_PORT, baudrate=DEFAULT_BAUDRATE):
        super().__init__()
        self.setWindowTitle("Motor Control GUI")
        self.setGeometry(100, 100, 1600, 900)

        self.link = MotorLink(port, baudrate)
        self.link.data_received.connect(self._on_data_received)
        self.link.status_changed.connect(self._set_status)

        self._build_ui()
        self._build_timers()

        # Bring the link up and start streaming.
        self.link.connect()
        self.link.start()

    # ── Construction ──────────────────────────────────────────────────────────
    def _build_ui(self):
        self.control_panel = ControlPanel()
        self.mode_panel = ModePanel()
        self.param_panel = ParamPanel(self.link)
        self.plot_panel = PlotPanel()

        self.mode_panel.mit_requested.connect(self._enable_mit_mode)
        self.mode_panel.calibration_requested.connect(self._enable_calibration_mode)
        self.mode_panel.menu_requested.connect(self._enable_menu_mode)
        # Commands are mis-scaled until the motor's real parameters are known,
        # so wait for a successful Read All before enabling them.
        self.param_panel.params_read.connect(self._on_params_read)

        self.send_btn = QPushButton("SEND COMMAND")
        self.send_btn.setStyleSheet(
            "background-color: #4CAF50; color: white; font-size: 14px; padding: 10px;"
        )
        self.send_btn.setEnabled(False)
        self.send_btn.clicked.connect(self._send_command)

        self.status_label = QLabel("Disconnected")
        status_group = QGroupBox("Status")
        status_layout = QVBoxLayout()
        status_layout.addWidget(self.status_label)
        status_group.setLayout(status_layout)

        left_panel = QVBoxLayout()
        left_panel.addWidget(self.control_panel)
        left_panel.addWidget(self.mode_panel)
        left_panel.addWidget(self.param_panel)
        left_panel.addWidget(self.send_btn)
        left_panel.addWidget(status_group)
        left_panel.addStretch()

        right_panel = QVBoxLayout()
        right_panel.addWidget(self.plot_panel)

        main_layout = QHBoxLayout()
        main_layout.addLayout(left_panel, 1)
        main_layout.addLayout(right_panel, 2)

        central = QWidget()
        central.setLayout(main_layout)
        self.setCentralWidget(central)

    def _build_timers(self):
        # Redraw the plots at a fixed rate, decoupled from the serial rate.
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.plot_panel.refresh)
        self.plot_timer.start(PLOT_REFRESH_MS)

        # Periodic command sending (poll / control loop); started in MIT mode.
        self.auto_send_timer = QTimer()
        self.auto_send_timer.timeout.connect(self._auto_send_command)
        self.auto_send_timer.setInterval(AUTO_SEND_MS)

    # ── Signal handlers ───────────────────────────────────────────────────────
    def _set_status(self, message):
        self.status_label.setText(message)

    def _on_data_received(self, data):
        self.plot_panel.add_sample(data)

    def _on_params_read(self, values):
        """Once the scaling-relevant parameters are known, adopt them for
        command/decode scaling and unlock command sending."""
        if not MotorScaling.has_required(values):
            self._set_status("Read all scaling params before sending commands")
            return
        scaling = MotorScaling.from_params(values)
        self.link.scaling = scaling
        self.control_panel.apply_scaling(scaling)
        self.control_panel.setEnabled(True)
        self.send_btn.setEnabled(True)
        self.mode_panel.set_mit_enabled(True)
        self._set_status("Parameters loaded — commands enabled")

    def _send_command(self):
        try:
            cmd = self.control_panel.command()
            self.link.send_command(self.control_panel.can_id(), **cmd)
            self._set_status(
                f"Command sent: p={cmd['p']:.2f}, kp={cmd['kp']:.1f}, kd={cmd['kd']:.2f}"
            )
        except Exception as e:
            self._set_status(f"Error: {e}")

    def _auto_send_command(self):
        """Send the current command periodically to poll the motor / run the
        control loop, eliciting regular status replies for plotting."""
        try:
            self.link.send_command(self.control_panel.can_id(), **self.control_panel.command())
        except Exception:
            pass

    # ── Mode control ──────────────────────────────────────────────────────────
    def _enable_mit_mode(self):
        try:
            self.link.set_mode(protocol.MIT_MODE, self.control_panel.can_id())
            self._set_status("MIT Mode enabled")
            # Start periodic auto-send so we get regular replies for plotting.
            self.auto_send_timer.start()
        except Exception as e:
            self._set_status(f"Error: {e}")

    def _enable_calibration_mode(self):
        try:
            self.link.set_mode(protocol.CALIBRATION_MODE, self.control_panel.can_id())
            self._set_status("Calibration Mode enabled")
        except Exception as e:
            self._set_status(f"Error: {e}")

    def _enable_menu_mode(self):
        try:
            self.link.set_mode(protocol.MENU_MODE, self.control_panel.can_id())
            self._set_status("MENU Mode enabled (motor disabled)")
            # Stop periodic auto-send when motor is disabled.
            self.auto_send_timer.stop()
        except Exception as e:
            self._set_status(f"Error: {e}")

    # ── Lifecycle ─────────────────────────────────────────────────────────────
    def closeEvent(self, event):
        self.link.close()
        event.accept()
