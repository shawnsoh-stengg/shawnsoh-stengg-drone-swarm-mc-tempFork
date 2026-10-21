"""Test session: log directory, JSONL event log, export bundle (TEST-007).

Every session writes, under <log root>/<YYYYmmdd-HHMMSS>/:

- channel_b.tlog / channel_c.tlog -- every frame, both directions;
- events.jsonl -- one JSON object per operator action, result or alarm,
  each stamped with the current test-case ID (e.g. "FT-001 AC2");
- session.json -- connection settings and tool/dialect versions.

`export()` zips the lot for attaching under test-results/.
"""

from __future__ import annotations

import json
import os
import platform
import subprocess
import threading
import time
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Optional

from .tlog import TlogWriter

DEFAULT_LOG_ROOT = Path(os.environ.get("GCS_TEST_PANEL_LOG_DIR", Path.home() / "gcs-test-panel-logs"))
_REPO_ROOT = Path(__file__).resolve().parents[4]


def _git_describe() -> str:
    try:
        out = subprocess.run(
            ["git", "-C", str(_REPO_ROOT), "describe", "--always", "--dirty"],
            capture_output=True, text=True, timeout=5,
        )
        return out.stdout.strip() or "unknown"
    except (OSError, subprocess.SubprocessError):
        return "unknown"


def _iso(t: float) -> str:
    return datetime.fromtimestamp(t, timezone.utc).isoformat(timespec="milliseconds")


class Session:
    def __init__(self, root: Path = DEFAULT_LOG_ROOT, name: Optional[str] = None):
        self.started = time.time()
        stamp = datetime.fromtimestamp(self.started).strftime("%Y%m%d-%H%M%S")
        self.dir = Path(root) / (name or stamp)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.test_case = ""
        self.tester = os.environ.get("USER", "")
        self._lock = threading.Lock()
        self._events = open(self.dir / "events.jsonl", "a", encoding="utf-8")
        self._listeners: list[Callable[[dict], None]] = []
        self._tlogs: dict[str, TlogWriter] = {}
        self.meta: dict = {
            "started": _iso(self.started),
            "tool": "gcs-test-panel (TEST-007)",
            "repo_version": _git_describe(),
            "host": platform.node(),
            "python": platform.python_version(),
        }
        self.write_meta()

    def add_listener(self, listener: Callable[[dict], None]) -> None:
        self._listeners.append(listener)

    def tlog(self, channel: str) -> TlogWriter:
        if channel not in self._tlogs:
            self._tlogs[channel] = TlogWriter(self.dir / f"{channel}.tlog")
        return self._tlogs[channel]

    def write_meta(self, **extra) -> None:
        self.meta.update(extra)
        (self.dir / "session.json").write_text(json.dumps(self.meta, indent=2) + "\n", encoding="utf-8")

    def event(self, kind: str, summary: str, passed: Optional[bool] = None, **data) -> dict:
        t = time.time()
        record = {
            "time": _iso(t),
            "t": round(t, 3),
            "test_case": self.test_case,
            "kind": kind,
            "summary": summary,
        }
        if passed is not None:
            record["pass"] = passed
        if data:
            record["data"] = data
        with self._lock:
            if not self._events.closed:
                self._events.write(json.dumps(record, default=str) + "\n")
                self._events.flush()
        for listener in list(self._listeners):
            try:
                listener(record)
            except Exception:
                pass
        return record

    def set_test_case(self, test_case: str) -> None:
        if test_case != self.test_case:
            self.test_case = test_case
            self.event("test_case", f"test case set to '{test_case}'" if test_case else "test case cleared")

    def note(self, text: str) -> None:
        self.event("note", text)

    def export(self, dest: Optional[Path] = None) -> Path:
        dest = Path(dest) if dest else self.dir.with_suffix(".zip")
        for tlog in self._tlogs.values():
            with tlog._lock:
                if not tlog._f.closed:
                    tlog._f.flush()
        with self._lock:
            self._events.flush()
        self.write_meta(exported=_iso(time.time()))
        with zipfile.ZipFile(dest, "w", zipfile.ZIP_DEFLATED) as zf:
            for path in sorted(self.dir.iterdir()):
                if path.is_file():
                    zf.write(path, f"{self.dir.name}/{path.name}")
        self.event("export", f"session exported to {dest}")
        return dest

    def close(self) -> None:
        self.event("session_end", "session closed")
        for tlog in self._tlogs.values():
            tlog.close()
        with self._lock:
            self._events.close()
