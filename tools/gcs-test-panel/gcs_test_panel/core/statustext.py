"""MAVLink v2 STATUSTEXT chunk reassembly.

gcs_mavlink_node splits a /mc/status_text longer than STATUSTEXT's 50 chars
into chunks sharing one nonzero `id`, numbered by `chunk_seq`; the first
chunk shorter than 50 chars (pymavlink strips the NUL) ends the text, so a
text filling its last chunk exactly is followed by an empty chunk. `id` 0 is
a complete single-frame text.
"""

from __future__ import annotations

from typing import Optional

CHUNK_LEN = 50


def statustext_text(msg) -> str:
    text = msg.text if isinstance(msg.text, str) else msg.text.decode("ascii", "replace")
    return text.rstrip("\x00")


class StatustextAssembler:
    def __init__(self, timeout_s: float = 2.0):
        self.timeout_s = timeout_s
        # (sysid, compid, id) -> (first chunk time, {chunk_seq: text})
        self._pending: dict[tuple[int, int, int], tuple[float, dict[int, str]]] = {}

    def feed(self, msg, t: float) -> Optional[str]:
        """Returns the whole text once its last chunk arrives, else None."""
        text = statustext_text(msg)
        chunk_id = getattr(msg, "id", 0)
        if not chunk_id:
            return text
        self._pending = {k: v for k, v in self._pending.items() if t - v[0] <= self.timeout_s}
        key = (msg.get_srcSystem(), msg.get_srcComponent(), chunk_id)
        _, parts = self._pending.setdefault(key, (t, {}))
        parts[msg.chunk_seq] = text
        if len(text) >= CHUNK_LEN:
            return None
        del self._pending[key]
        # A lost middle chunk shows as a gap rather than silently joining.
        return "".join(parts.get(i, "…") for i in range(max(parts) + 1))
