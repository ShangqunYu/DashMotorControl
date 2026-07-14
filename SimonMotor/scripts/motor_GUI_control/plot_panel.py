"""Live plotting widget: a grid of pyqtgraph plots backed by rolling
time-window buffers. Feed it decoded status dicts with `add_sample()` and call
`refresh()` on a timer to redraw."""

import os
from collections import deque

# Force pyqtgraph onto the same Qt binding as the rest of the app (PyQt5).
# The base env also has PySide6, which pyqtgraph would otherwise prefer,
# producing widgets whose QWidget base class doesn't match PyQt5's layouts.
os.environ.setdefault("PYQTGRAPH_QT_LIB", "PyQt5")

import pyqtgraph as pg

from config import PLOT_CHANNELS, PLOT_WINDOW_SECONDS


class PlotPanel(pg.GraphicsLayoutWidget):
    def __init__(self, window_seconds=PLOT_WINDOW_SECONDS):
        pg.setConfigOption('background', '#1a1a1a')
        pg.setConfigOption('foreground', '#ffffff')
        super().__init__()

        self.window_seconds = window_seconds
        # Shared timestamps + one value buffer per channel, trimmed to the window.
        self._timestamps = deque()
        self._buffers = {ch["key"]: deque() for ch in PLOT_CHANNELS}

        self._plots = []
        self._curves = {}
        for ch in PLOT_CHANNELS:
            plot = self.addPlot(
                title=ch["title"], row=ch["row"], col=ch["col"], colspan=ch["colspan"]
            )
            plot.setLabel('bottom', 'Time', units='s')
            plot.setLabel('left', ch["label"], units=ch["units"])
            if not ch["si_prefix"]:
                plot.getAxis('left').enableAutoSIPrefix(False)
            self._curves[ch["key"]] = plot.plot(pen=pg.mkPen(ch["color"], width=2))
            self._plots.append(plot)

    def add_sample(self, data):
        """Append one decoded status sample and drop anything older than the
        rolling window so memory and the visible range stay bounded."""
        self._timestamps.append(data['time'])
        for key, buffer in self._buffers.items():
            buffer.append(data[key])

        cutoff = data['time'] - self.window_seconds
        while self._timestamps and self._timestamps[0] < cutoff:
            self._timestamps.popleft()
            for buffer in self._buffers.values():
                buffer.popleft()

    def refresh(self):
        """Redraw the curves and scroll the x-axis to the last window."""
        if not self._timestamps:
            return
        t = list(self._timestamps)
        for key, curve in self._curves.items():
            curve.setData(t, list(self._buffers[key]))
        # Lock the x-axis to the last window so the trace scrolls instead of
        # the view zooming out over time.
        for plot in self._plots:
            plot.setXRange(t[-1] - self.window_seconds, t[-1], padding=0)
