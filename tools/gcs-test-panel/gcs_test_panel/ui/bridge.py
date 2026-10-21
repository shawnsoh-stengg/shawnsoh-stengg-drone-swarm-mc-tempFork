"""Thread -> Qt plumbing (TEST-007).

Link reader threads and the command/MAVFTP worker pool never touch widgets.
They emit signals on QObjects that live in the GUI thread; Qt's
AutoConnection then queues the slot onto the GUI thread. Slots are always
bound methods of GUI-thread QObjects -- a plain lambda connected to a signal
emitted from a worker thread would run on that worker thread instead.
"""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from typing import Callable, Optional

from PySide6.QtCore import QObject, Signal, Slot


class Bridge(QObject):
    # (channel, msg, direction, timestamp)
    message = Signal(str, object, str, float)
    # one events.jsonl record
    event = Signal(dict)
    # Channel B upload progress: (seq, count, text)
    upload_progress = Signal(int, int, str)


class _Task(QObject):
    _done = Signal(object, object)

    def __init__(self, runner: "Runner", on_done: Callable, on_error: Optional[Callable]):
        super().__init__()
        self._runner = runner
        self._on_done = on_done
        self._on_error = on_error
        self._done.connect(self._deliver)

    @Slot(object, object)
    def _deliver(self, result, error):
        self._runner._tasks.discard(self)
        if error is None:
            self._on_done(result)
        elif self._on_error is not None:
            self._on_error(error)


class Runner:
    """Run blocking request/reply calls off the GUI thread; deliver the
    result (or exception) back on it."""

    def __init__(self, max_workers: int = 4):
        self._pool = ThreadPoolExecutor(max_workers=max_workers, thread_name_prefix="panel-worker")
        self._tasks: set[_Task] = set()

    def run(self, fn: Callable[[], object], on_done: Callable[[object], None],
            on_error: Optional[Callable[[BaseException], None]] = None) -> None:
        task = _Task(self, on_done, on_error)
        self._tasks.add(task)

        def work():
            try:
                task._done.emit(fn(), None)
            except Exception as e:  # delivered to on_error on the GUI thread
                task._done.emit(None, e)

        self._pool.submit(work)

    def shutdown(self) -> None:
        self._pool.shutdown(wait=False, cancel_futures=True)
