"""Channel C client -- Mission Orchestrator <-> mission_orchestrator_node (TEST-007).

Reuses TEST-004's ChannelCClient (MAVFTP plan CRUD + MC_MISSION_PLAN_ACTIVATE)
unchanged, except that replies arrive through this panel's reader-thread
Subscription instead of the client reading the socket itself -- via the
`_recv_match` seam that client exposes for exactly this.

mission_orchestrator_node never sends anything unsolicited (no heartbeat), so
Channel C "link up" here means "the last request got a reply", not a lamp
driven by a heartbeat.
"""

from __future__ import annotations

import functools
import itertools
import threading
import time
from typing import Optional

from . import defs
from .link import LinkWorker, SplitPortLink
from .shared import ChannelCClient, DirectoryEntry, MavftpError
from .tlog import TlogWriter


def _operation(fn):
    """Serialise top-level operations and drop stale replies from a previous,
    timed-out one before starting -- a late FTP Ack with the same req_opcode
    would otherwise be taken as this request's reply."""

    @functools.wraps(fn)
    def wrapper(self, *args, **kwargs):
        with self._op_lock:
            self._sub.drain()
            try:
                result = fn(self, *args, **kwargs)
            except MavftpError as e:
                # A Nak is still a reply -- only a timeout means the link is down.
                self.last_reply_ok = "timed out" not in str(e)
                if self.last_reply_ok:
                    self.last_reply_time = time.time()
                raise
            self.last_reply_ok = True
            self.last_reply_time = time.time()
            return result

    return wrapper


class _QueuedClient(ChannelCClient):
    def __init__(self, link, mc_system_id, timeout, sub):
        super().__init__(link, mc_system_id, timeout)
        self._sub = sub

    def _recv_match(self, types, timeout):
        msg = self._sub.get(timeout)
        if msg is not None and msg.get_type() not in types:
            return None
        return msg


class ChannelC:
    def __init__(
        self,
        mc_host: str,
        drone_id: int,
        recv_port: int = defs.CH_C_RECV_PORT,
        send_port: int = defs.CH_C_SEND_PORT,
        tlog: Optional[TlogWriter] = None,
        timeout: float = 5.0,
        bind_host: str = "0.0.0.0",
    ):
        self.drone_id = drone_id
        self.link = SplitPortLink(
            recv_port, mc_host, send_port, defs.GCS_SYSTEM_ID, defs.ORCHESTRATOR_COMPONENT_ID, bind_host
        )
        self.worker = LinkWorker("channel_c", self.link, tlog)
        self._sub = self.worker.subscribe(["FILE_TRANSFER_PROTOCOL", "MC_MISSION_PLAN_ACK"])
        self.client = _QueuedClient(self.link, drone_id, timeout, self._sub)
        self._op_lock = threading.Lock()
        self._request_ids = itertools.count(int(time.time()) % 100000 * 10)
        self.last_reply_time: Optional[float] = None
        self.last_reply_ok: Optional[bool] = None

    def start(self) -> None:
        self.worker.start()

    def stop(self) -> None:
        self.worker.stop()

    def link_up(self) -> Optional[bool]:
        return self.last_reply_ok

    @staticmethod
    def plan_filename(name: str) -> str:
        return name if name.endswith(".json") else f"{name}.json"

    @_operation
    def list_plans(self) -> tuple[list[DirectoryEntry], str]:
        entries = self.client.list_directory()
        active = self.client.read_active_filename()
        return entries, active

    @_operation
    def read_plan(self, name: str) -> bytes:
        return self.client.read_plan(self.plan_filename(name))

    @_operation
    def create_plan(self, name: str, data: bytes) -> None:
        self.client.create_or_update(self.plan_filename(name), data, update=False)

    @_operation
    def update_plan(self, name: str, data: bytes) -> None:
        self.client.create_or_update(self.plan_filename(name), data, update=True)

    @_operation
    def delete_plan(self, name: str) -> None:
        self.client.remove_plan(self.plan_filename(name))

    @_operation
    def activate(self, name: str):
        """Returns (client_request_id, MC_MISSION_PLAN_ACK)."""
        request_id = next(self._request_ids) & 0xFFFFFFFF
        return request_id, self.client.activate(self.plan_filename(name), request_id)
