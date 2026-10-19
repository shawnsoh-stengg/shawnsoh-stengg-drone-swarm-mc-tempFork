"""Monitor tab: MC_STATUS / MC_HEALTH / MC_DETECTION live view (TEST-007)."""

from __future__ import annotations

from datetime import datetime

import pyqtgraph as pg
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QProgressBar,
    QPushButton,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from ..core import defs
from ..core.telemetry import Telemetry, local_xy
from .widgets import GREEN, GREY, RED, Tile


class MonitorTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.telemetry: Telemetry | None = None
        self.origin: tuple[float, float] | None = None
        self.plan: dict | None = None
        self._last_det_t = 0.0

        split = QSplitter(Qt.Horizontal, self)
        outer = QHBoxLayout(self)
        outer.setContentsMargins(4, 4, 4, 4)
        outer.addWidget(split)

        # --- left column: status tiles, faults, health ---
        left = QWidget()
        lv = QVBoxLayout(left)
        lv.setContentsMargins(0, 0, 0, 0)

        status_box = QGroupBox("MC_STATUS")
        grid = QGridLayout(status_box)
        self.t_phase = Tile("Mission phase", big=True)
        self.t_pos = Tile("Lat / Lon (deg)")
        self.t_alt = Tile("Alt MSL (m)")
        self.t_hdg = Tile("Heading (deg)")
        self.t_gs = Tile("Groundspeed (m/s)")
        self.t_climb = Tile("Climb (m/s)")
        self.t_batt = Tile("Battery (%)")
        self.t_loc = Tile("Localisation")
        self.t_peers = Tile("Peers / ISR")
        self.t_status_rate = Tile(f"MC_STATUS rate (≥{defs.MC_STATUS_MIN_HZ:.0f} Hz)")
        self.t_age = Tile("Last MC_STATUS")
        grid.addWidget(self.t_phase, 0, 0, 1, 2)
        grid.addWidget(self.t_status_rate, 0, 2)
        for i, tile in enumerate(
            [self.t_pos, self.t_alt, self.t_hdg, self.t_gs, self.t_climb, self.t_batt, self.t_loc, self.t_peers, self.t_age]
        ):
            grid.addWidget(tile, 1 + i // 3, i % 3)
        lv.addWidget(status_box)

        fault_box = QGroupBox("Active faults (fault_mask, ICD §10.8)")
        fgrid = QGridLayout(fault_box)
        fgrid.setSpacing(2)
        self.fault_labels: dict[int, QLabel] = {}
        for i, (bit, name) in enumerate(defs.FAULT_BITS.items()):
            label = QLabel(name)
            label.setAlignment(Qt.AlignCenter)
            self.fault_labels[bit] = label
            fgrid.addWidget(label, i // 3, i % 3)
        lv.addWidget(fault_box)

        health_box = QGroupBox("MC_HEALTH")
        hgrid = QGridLayout(health_box)
        self.bars = {}
        for row, key in enumerate(("CPU", "Memory", "Disk")):
            hgrid.addWidget(QLabel(key), row, 0)
            bar = QProgressBar()
            bar.setRange(0, 100)
            bar.setFormat("%v %")
            self.bars[key] = bar
            hgrid.addWidget(bar, row, 1, 1, 2)
        self.t_temps = Tile("CPU / GPU temp (°C)")
        self.t_uptime = Tile("MC uptime")
        self.t_health_rate = Tile(f"MC_HEALTH rate (≥{defs.MC_HEALTH_MIN_HZ:.0f} Hz)")
        hgrid.addWidget(self.t_temps, 3, 0)
        hgrid.addWidget(self.t_uptime, 3, 1)
        hgrid.addWidget(self.t_health_rate, 3, 2)
        nodes = QGroupBox("Node liveness (node_alive_mask, ICD §10.9)")
        ngrid = QGridLayout(nodes)
        ngrid.setSpacing(2)
        self.node_labels: dict[int, QLabel] = {}
        for i, (bit, name) in enumerate(defs.NODE_ALIVE_BITS.items()):
            label = QLabel(name)
            label.setAlignment(Qt.AlignCenter)
            self.node_labels[bit] = label
            ngrid.addWidget(label, i // 3, i % 3)
        hgrid.addWidget(nodes, 4, 0, 1, 3)
        lv.addWidget(health_box)
        lv.addStretch(1)
        split.addWidget(left)

        # --- right column: plots / map, detections ---
        right = QSplitter(Qt.Vertical)
        tabs = QTabWidget()
        pg.setConfigOptions(antialias=True, background="w", foreground="k")
        plots = pg.GraphicsLayoutWidget()
        self.p_alt = plots.addPlot(row=0, col=0, title="Altitude MSL (m)")
        self.p_batt = plots.addPlot(row=1, col=0, title="Battery (%)")
        self.p_phase = plots.addPlot(row=2, col=0, title="Mission phase")
        for p in (self.p_alt, self.p_batt, self.p_phase):
            p.showGrid(x=True, y=True, alpha=0.3)
            p.setLabel("bottom", "s ago")
        self.p_batt.setYRange(0, 100)
        # Label a few landmark phases; the rest are unlabelled minor ticks
        # (11 labels don't fit the plot height). Exact phase is in the tile.
        landmarks = {0, 3, 7}
        self.p_phase.getAxis("left").setTicks([
            [(v, defs.MISSION_PHASES[v]) for v in sorted(landmarks)] + [(10, "FAULT")],
            [(v, "") for v in defs.MISSION_PHASES if v not in landmarks and v != 255],
        ])
        self.p_phase.setYRange(-0.5, 10.5)
        self.c_alt = self.p_alt.plot(pen=pg.mkPen("#1f6feb", width=2))
        self.c_batt = self.p_batt.plot(pen=pg.mkPen("#2e9e44", width=2))
        self.c_phase = self.p_phase.plot(pen=pg.mkPen("#8250df", width=2), stepMode=None)
        tabs.addTab(plots, "Plots")

        map_widget = QWidget()
        mv = QVBoxLayout(map_widget)
        mv.setContentsMargins(0, 0, 0, 0)
        self.map = pg.PlotWidget(title="Local map — east/north metres from origin (no tiles needed)")
        self.map.setAspectLocked(True)
        self.map.showGrid(x=True, y=True, alpha=0.3)
        self.map.addLegend(offset=(10, 10))
        self.m_plan_route = self.map.plot(pen=pg.mkPen("#d18f00", width=2, style=Qt.DashLine), symbol="t", symbolSize=8, name="plan ingress")
        self.m_plan_area = self.map.plot(pen=pg.mkPen("#8250df", width=2), name="plan search area")
        self.m_track = self.map.plot(pen=pg.mkPen("#1f6feb", width=2), name="track")
        self.m_drone = self.map.plot(pen=None, symbol="o", symbolSize=12, symbolBrush="#1f6feb", name="drone")
        self.m_det = self.map.plot(pen=None, symbol="x", symbolSize=12, symbolPen=pg.mkPen("#d13438", width=2), name="detections")
        bar = QHBoxLayout()
        self.origin_label = QLabel("origin: —")
        reset = QPushButton("Reset origin / clear track")
        reset.clicked.connect(self.reset_map)
        bar.addWidget(self.origin_label)
        bar.addStretch(1)
        bar.addWidget(reset)
        mv.addWidget(self.map)
        mv.addLayout(bar)
        tabs.addTab(map_widget, "Map")
        right.addWidget(tabs)

        det_box = QGroupBox("MC_DETECTION")
        dv = QVBoxLayout(det_box)
        self.det_table = QTableWidget(0, 8)
        self.det_table.setHorizontalHeaderLabels(["Time", "ID", "Class", "Conf", "Lat", "Lon", "Alt (m)", "±m"])
        self.det_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.det_table.verticalHeader().hide()
        self.det_table.setEditTriggers(QTableWidget.NoEditTriggers)
        dv.addWidget(self.det_table)
        right.addWidget(det_box)
        right.setSizes([520, 200])
        split.addWidget(right)
        split.setSizes([560, 700])

    # --- data binding ---

    def bind(self, telemetry: Telemetry) -> None:
        self.telemetry = telemetry
        self.det_table.setRowCount(0)
        self._last_det_t = 0.0
        self.reset_map()

    def set_plan(self, plan: dict | None) -> None:
        self.plan = plan
        if plan and self.origin is None and plan.get("ingress_waypoints"):
            wp = plan["ingress_waypoints"][0]
            self._set_origin(wp["lat"], wp["lon"])
        self._draw_plan()

    def reset_map(self) -> None:
        self.origin = None
        self.origin_label.setText("origin: —")
        if self.telemetry is not None:
            self.telemetry.track.clear()
        for curve in (self.m_track, self.m_drone, self.m_det):
            curve.setData([], [])
        self._draw_plan()

    def _set_origin(self, lat: float, lon: float) -> None:
        self.origin = (lat, lon)
        self.origin_label.setText(f"origin: {lat:.6f}, {lon:.6f}")

    def _xy(self, points):
        lat0, lon0 = self.origin
        xs, ys = [], []
        for lat, lon in points:
            e, n = local_xy(lat, lon, lat0, lon0)
            xs.append(e)
            ys.append(n)
        return xs, ys

    def _draw_plan(self) -> None:
        if not self.plan or self.origin is None:
            self.m_plan_route.setData([], [])
            self.m_plan_area.setData([], [])
            return
        route = [(w["lat"], w["lon"]) for w in self.plan.get("ingress_waypoints", []) if "lat" in w and "lon" in w]
        area = [(v["lat"], v["lon"]) for v in self.plan.get("search_polygon", []) if "lat" in v and "lon" in v]
        if area:
            area.append(area[0])
        self.m_plan_route.setData(*self._xy(route))
        self.m_plan_area.setData(*self._xy(area))

    @staticmethod
    def _paint(label: QLabel, colour: str) -> None:
        text = "white" if colour in (RED, GREEN) else "black"
        label.setStyleSheet(f"background:{colour}; color:{text}; border-radius:3px; padding:2px 4px;")

    @staticmethod
    def _paint_fault(label: QLabel, state: bool | None) -> None:
        # Inactive faults stay quiet so the one that's set stands out.
        if state:
            style = f"background:{RED}; color:white; font-weight:600;"
        elif state is None:
            style = f"background:transparent; color:{GREY}; border:1px solid {GREY};"
        else:
            style = "background:transparent; color:palette(text); border:1px solid #c8c8c8;"
        label.setStyleSheet(style + " border-radius:3px; padding:2px 4px;")

    def refresh(self, now: float, link_up: bool) -> None:
        tele = self.telemetry
        if tele is None:
            return
        rates = tele.check_rates(now, link_up)

        s = tele.status
        r, req, ok = rates["MC_STATUS"]
        self.t_status_rate.set(f"{r:.2f} Hz", alarm=s is not None and not ok)
        if s is not None:
            phase = defs.phase_name(s.mission_phase)
            self.t_phase.set(phase, alarm=s.mission_phase == 255)
            self.t_pos.set(f"{s.lat / 1e7:.7f}\n{s.lon / 1e7:.7f}", warn=s.lat == 0 and s.lon == 0)
            self.t_alt.set(f"{s.alt_msl / 1000:.2f}")
            self.t_hdg.set(f"{s.heading / 100:.1f}")
            self.t_gs.set(f"{s.groundspeed / 100:.2f}")
            self.t_climb.set(f"{s.climb_rate / 100:+.2f}")
            self.t_batt.set(f"{s.battery_remaining}", warn=s.battery_remaining < 30, alarm=s.battery_remaining < 20)
            self.t_loc.set(defs.LOCALISATION_MODES.get(s.localisation_mode, str(s.localisation_mode)))
            self.t_peers.set(f"{s.peer_count} / {'ISR' if s.isr_role else 'non-ISR'}")
            age = now - tele.status_time
            self.t_age.set(f"{age:.1f} s ago", alarm=age > 2.0)
            fault = s.fault_mask | (tele.health.fault_mask if tele.health is not None else 0)
        else:
            fault = tele.health.fault_mask if tele.health is not None else None
        for bit, label in self.fault_labels.items():
            self._paint_fault(label, None if fault is None else bool(fault & (1 << bit)))

        h = tele.health
        r, req, ok = rates["MC_HEALTH"]
        self.t_health_rate.set(f"{r:.2f} Hz", alarm=h is not None and not ok)
        if h is not None:
            for key, value in (("CPU", h.cpu_load), ("Memory", h.mem_used), ("Disk", h.disk_used)):
                self.bars[key].setValue(value)
            self.t_temps.set(f"{h.cpu_temp} / {h.gpu_temp}", warn=h.gpu_temp >= 75, alarm=h.gpu_temp > 85)
            up = h.uptime_s
            self.t_uptime.set(f"{up // 3600}h {up // 60 % 60:02d}m {up % 60:02d}s")
            for bit, label in self.node_labels.items():
                self._paint(label, GREEN if h.node_alive_mask & (1 << bit) else RED)
        else:
            for label in self.node_labels.values():
                self._paint(label, GREY)

        self._refresh_plots(now)
        self._refresh_map()
        self._refresh_detections()

    def _refresh_plots(self, now: float) -> None:
        series = list(self.telemetry.series)
        if not series:
            for c in (self.c_alt, self.c_batt, self.c_phase):
                c.setData([], [])
            return
        series = [p for p in series if now - p[0] <= 600]
        xs = [p[0] - now for p in series]
        self.c_alt.setData(xs, [p[1] for p in series])
        self.c_batt.setData(xs, [p[2] for p in series])
        self.c_phase.setData(xs, [10 if p[3] == 255 else p[3] for p in series])

    def _refresh_map(self) -> None:
        track = list(self.telemetry.track)
        if track and self.origin is None:
            self._set_origin(track[0][1], track[0][2])
            self._draw_plan()
        if self.origin is None:
            return
        if track:
            xs, ys = self._xy([(p[1], p[2]) for p in track])
            self.m_track.setData(xs, ys)
            self.m_drone.setData(xs[-1:], ys[-1:])
        dets = list(self.telemetry.detections)
        if dets:
            self.m_det.setData(*self._xy([(d.lat, d.lon) for d in dets]))

    def _refresh_detections(self) -> None:
        new = [d for d in self.telemetry.detections if d.t > self._last_det_t]
        for d in new:
            self.det_table.insertRow(0)
            values = [
                datetime.fromtimestamp(d.t).strftime("%H:%M:%S"), str(d.detection_id), d.class_name,
                f"{d.confidence:.2f}", f"{d.lat:.7f}", f"{d.lon:.7f}", f"{d.alt_m:.1f}", f"{d.pos_uncertainty_m:.2f}",
            ]
            for col, v in enumerate(values):
                self.det_table.setItem(0, col, QTableWidgetItem(v))
        if new:
            self._last_det_t = new[-1].t
        while self.det_table.rowCount() > 500:
            self.det_table.removeRow(self.det_table.rowCount() - 1)
