"""Left-hand input panels: motor control inputs, mode buttons, and the config
parameter tuner. Each is a self-contained QGroupBox so the main window just
composes them and connects a few signals."""

from PyQt5.QtWidgets import (
    QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton, QDoubleSpinBox,
    QGroupBox, QFormLayout, QTableWidget, QTableWidgetItem, QAbstractItemView,
    QHeaderView,
)
from PyQt5.QtCore import Qt, pyqtSignal

from config import CONFIG_PARAMS, CONFIG_DEFAULTS, DEFAULT_COMMAND_CAN_ID, DEFAULT_CONFIG_CAN_ID

# Placeholder half-range for the spinboxes before real limits are read. Wide
# enough not to clamp the initial values; replaced by apply_scaling().
_PLACEHOLDER_LIMIT = 1e6


def _make_spinbox(minimum, maximum, value, step):
    box = QDoubleSpinBox()
    box.setRange(minimum, maximum)
    box.setValue(value)
    box.setSingleStep(step)
    return box


class ControlPanel(QGroupBox):
    """MIT command inputs (position/velocity/gains/torque) plus the CAN ID.

    Disabled until the motor's scaling parameters are read; apply_scaling()
    installs the real input limits and the window re-enables the panel."""

    def __init__(self):
        super().__init__("Motor Control")
        layout = QFormLayout()

        self.can_id_input = QLineEdit(DEFAULT_COMMAND_CAN_ID)
        layout.addRow("CAN ID (8 hex):", self.can_id_input)

        self.position_input = _make_spinbox(-_PLACEHOLDER_LIMIT, _PLACEHOLDER_LIMIT, 0.0, 0.1)
        layout.addRow("Position (rad):", self.position_input)

        self.velocity_input = _make_spinbox(-_PLACEHOLDER_LIMIT, _PLACEHOLDER_LIMIT, 0.0, 1.0)
        layout.addRow("Velocity (rad/s):", self.velocity_input)

        self.kp_input = _make_spinbox(0, _PLACEHOLDER_LIMIT, 20.0, 1.0)
        layout.addRow("KP (N-m/rad):", self.kp_input)

        self.kd_input = _make_spinbox(0, _PLACEHOLDER_LIMIT, 0.5, 0.1)
        layout.addRow("KD (N-m·s/rad):", self.kd_input)

        self.torque_input = _make_spinbox(-_PLACEHOLDER_LIMIT, _PLACEHOLDER_LIMIT, 0.0, 0.5)
        layout.addRow("Torque FF (N-m):", self.torque_input)

        self.setLayout(layout)
        self.setEnabled(False)  # unlocked once parameters are read

    def apply_scaling(self, scaling):
        """Clamp the input ranges to the motor's actual limits so a value can't
        be entered that the firmware would saturate."""
        self.position_input.setRange(scaling.p_min, scaling.p_max)
        self.velocity_input.setRange(scaling.v_min, scaling.v_max)
        self.kp_input.setRange(0, scaling.kp_max)
        self.kd_input.setRange(0, scaling.kd_max)
        self.torque_input.setRange(scaling.t_min, scaling.t_max)

    def can_id(self):
        return self.can_id_input.text()

    def command(self):
        """Current control inputs as kwargs for MotorLink.send_command()."""
        return {
            "p": self.position_input.value(),
            "v": self.velocity_input.value(),
            "kp": self.kp_input.value(),
            "kd": self.kd_input.value(),
            "t_ff": self.torque_input.value(),
        }


class ModePanel(QGroupBox):
    """Mode selection buttons. Emits a signal per mode; the window decides what
    each mode does (e.g. starting/stopping the auto-send timer)."""

    mit_requested = pyqtSignal()
    calibration_requested = pyqtSignal()
    menu_requested = pyqtSignal()

    def __init__(self):
        super().__init__("Mode")
        layout = QVBoxLayout()

        # MIT mode commands the motor with scaled torque, so it stays disabled
        # until parameters have been read.
        self.mit_btn = QPushButton("Enable MIT Mode")
        self.mit_btn.setEnabled(False)
        self.mit_btn.clicked.connect(self.mit_requested)
        layout.addWidget(self.mit_btn)

        calibration_btn = QPushButton("Enable Calibration Mode")
        calibration_btn.clicked.connect(self.calibration_requested)
        layout.addWidget(calibration_btn)

        menu_btn = QPushButton("Disable (MENU Mode)")
        menu_btn.clicked.connect(self.menu_requested)
        layout.addWidget(menu_btn)

        self.setLayout(layout)

    def set_mit_enabled(self, enabled):
        self.mit_btn.setEnabled(enabled)


class ParamPanel(QGroupBox):
    """Config parameter tuner: shows every parameter in a table, reads them all
    off the motor at once, and writes/reset over the given MotorLink."""

    # Emitted after a successful Read All with {param_index: value}.
    params_read = pyqtSignal(dict)

    VALUE_COL = 1

    def __init__(self, link):
        super().__init__("Parameter Tuning")
        self.link = link
        layout = QVBoxLayout()

        form = QFormLayout()
        self.can_id_input = QLineEdit(DEFAULT_CONFIG_CAN_ID)
        self.can_id_input.setPlaceholderText("8-digit hex")
        form.addRow("Config CAN ID:", self.can_id_input)
        layout.addLayout(form)

        self.table = self._build_table()
        layout.addWidget(self.table)

        buttons = QHBoxLayout()
        read_btn = QPushButton("Read All")
        read_btn.clicked.connect(self.read_all)
        buttons.addWidget(read_btn)

        write_btn = QPushButton("Write Selected")
        write_btn.clicked.connect(self.write_selected)
        buttons.addWidget(write_btn)

        reset_btn = QPushButton("Reset Defaults")
        reset_btn.clicked.connect(self.reset_parameters)
        buttons.addWidget(reset_btn)
        layout.addLayout(buttons)

        self.status_label = QLabel("Read parameters before sending commands")
        layout.addWidget(self.status_label)

        self.setLayout(layout)

    # ── Table construction ────────────────────────────────────────────────────
    def _build_table(self):
        table = QTableWidget(len(CONFIG_PARAMS), 2)
        table.setHorizontalHeaderLabels(["Parameter", "Value"])
        table.verticalHeader().setVisible(False)
        table.setSelectionBehavior(QAbstractItemView.SelectRows)
        table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)

        # Row order maps 1:1 to CONFIG_PARAMS; keep the index alongside.
        self._index_by_row = []
        for row, (index, name, unit, default) in enumerate(CONFIG_PARAMS):
            label = f"{name} ({unit})" if unit else name
            name_item = QTableWidgetItem(label)
            name_item.setFlags(name_item.flags() & ~Qt.ItemIsEditable)
            table.setItem(row, 0, name_item)
            table.setItem(row, self.VALUE_COL, QTableWidgetItem(self._format_value(index, default)))
            self._index_by_row.append(index)
        return table

    # ── Helpers ───────────────────────────────────────────────────────────────
    @staticmethod
    def _format_value(index, value):
        """Integer params (bit 0x80) render as ints; floats trim trailing zeros."""
        if index & 0x80:
            return str(int(value))
        return f"{value:.6f}".rstrip("0").rstrip(".")

    def _param_name(self, row):
        return self.table.item(row, 0).text().split(" (")[0]

    def _set_row_value(self, row, index, value):
        self.table.item(row, self.VALUE_COL).setText(self._format_value(index, value))

    # ── Actions ───────────────────────────────────────────────────────────────
    def read_all(self):
        try:
            values = self.link.read_all_params(self.can_id_input.text(), list(self._index_by_row))
        except Exception as e:
            self.status_label.setText(f"Config read error: {e}")
            return

        for row, index in enumerate(self._index_by_row):
            value = values.get(index)
            if value is not None:
                self._set_row_value(row, index, value)

        present = {index: v for index, v in values.items() if v is not None}
        missing = [index for index, v in values.items() if v is None]
        if missing:
            self.status_label.setText(
                f"Read {len(present)}/{len(values)} params; no reply for indices {missing}"
            )
        else:
            self.status_label.setText(f"Read all {len(values)} parameters")
        self.params_read.emit(present)

    def write_selected(self):
        row = self.table.currentRow()
        if row < 0:
            self.status_label.setText("Select a parameter row to write")
            return
        try:
            index = self._index_by_row[row]
            value = float(self.table.item(row, self.VALUE_COL).text())
            written = self.link.write_param(self.can_id_input.text(), index, value)
            if written is None:
                self.status_label.setText("No reply for parameter write")
                return
            self._set_row_value(row, index, written)
            self.status_label.setText(f"Wrote {self._param_name(row)} = {written}")
        except Exception as e:
            self.status_label.setText(f"Config write error: {e}")

    def reset_parameters(self):
        try:
            confirmed = self.link.reset_params(self.can_id_input.text())
        except Exception as e:
            self.status_label.setText(f"Reset error: {e}")
            return
        if confirmed:
            self.status_label.setText("Parameters reset to defaults; re-reading…")
            self.read_all()
        else:
            self.status_label.setText("Reset command sent, no confirmation reply")
