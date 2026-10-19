"""gcs-test-panel entry point (TEST-007)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .core import defs
from .core.session import DEFAULT_LOG_ROOT
from .core.shared import load_dialect


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="gcs-test-panel",
        description="GUI stand-in for the ground Mission Orchestrator app on MAVLink Channels B and C "
        "(TEST-007). Channel A stays with QGroundControl.",
    )
    p.add_argument("--mc-host", default=defs.DEFAULT_MC_HOST, help=f"MC's LAN IP (default {defs.DEFAULT_MC_HOST})")
    p.add_argument("--drone-id", type=int, default=defs.DEFAULT_DRONE_ID, help="MC DRONE_ID / system_id (default 1)")
    p.add_argument("--bind", default="0.0.0.0", help="local address to bind the listen ports on (default 0.0.0.0)")
    p.add_argument("--b-recv-port", type=int, default=defs.CH_B_RECV_PORT)
    p.add_argument("--b-send-port", type=int, default=defs.CH_B_SEND_PORT)
    p.add_argument("--c-recv-port", type=int, default=defs.CH_C_RECV_PORT)
    p.add_argument("--c-send-port", type=int, default=defs.CH_C_SEND_PORT)
    p.add_argument("--log-dir", default=str(DEFAULT_LOG_ROOT), help=f"session log root (default {DEFAULT_LOG_ROOT})")
    p.add_argument("--test-case", default="", help="initial test-case ID, e.g. 'FT-001 AC2'")
    p.add_argument("--connect", action="store_true", help="connect on startup")
    p.add_argument("--regen-dialect", action="store_true", help="force-regenerate the mc_mavlink_defs dialect")
    p.add_argument("--screenshot", metavar="PNG", help=argparse.SUPPRESS)  # docs/verification: grab window, quit
    p.add_argument("--screenshot-tab", type=int, default=0, help=argparse.SUPPRESS)
    p.add_argument("--screenshot-after", type=float, default=8.0, help=argparse.SUPPRESS)
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    load_dialect(force_regen=args.regen_dialect)

    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import QApplication

    from .ui.main_window import MainWindow

    app = QApplication(sys.argv[:1])
    app.setApplicationName("gcs-test-panel")
    app.setStyle("Fusion")
    win = MainWindow(args)
    win.show()

    if args.screenshot:
        def grab():
            win.tabs.setCurrentIndex(args.screenshot_tab)
            QTimer.singleShot(500, lambda: (win.grab().save(args.screenshot), win.close()))

        QTimer.singleShot(int(args.screenshot_after * 1000), grab)
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
