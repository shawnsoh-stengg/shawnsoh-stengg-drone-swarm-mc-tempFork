"""A minimal fake MC for tests and offline demos (TEST-007) -- NOT a reference.

Plays the MC side of both channels on loopback, closely enough to exercise the
panel's own code paths without a Jetson, containers or ROS 2:

- Channel B: MC heartbeat (only once a GCS heartbeat has been seen, like
  gcs_mavlink_node), COMMAND_ACK for every COMMAND_LONG, the mission_type=3
  upload handshake with upload_id in MISSION_ACK.opaque_id, MC_STATUS at 2 Hz
  (a slow circle, phase driven by MC_CMD_PHASE_TRANSITION), MC_HEALTH at 1 Hz,
  an MC_DETECTION every few seconds while in SEARCH, and GCS_LINK_LOST after
  10 s without a GCS heartbeat.
- Channel C: an in-memory MAVFTP plan library + MC_MISSION_PLAN_ACTIVATE.

The real gcs_mavlink_node/mission_orchestrator_node are the reference -- the
panel's acceptance run is against them (tools/gcs-test-panel/README.md).

    python3 -m gcs_test_panel.fake_mc [--drone-id 1] [--drop-every N]
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import threading
import time

from .core import defs
from .core.link import LinkWorker, SplitPortLink
from .core.shared import load_dialect

FTP_ACK, FTP_NAK = 128, 129
OP_TERMINATE, OP_RESET, OP_LIST, OP_OPEN_RO, OP_READ, OP_CREATE, OP_WRITE, OP_REMOVE = 1, 2, 3, 4, 5, 6, 7, 8
OP_OPEN_WO, OP_BURST_READ = 11, 15
ERR_EOF, ERR_EXISTS, ERR_PROTECTED, ERR_NOT_FOUND = 6, 8, 9, 10


class FakeMC:
    def __init__(self, drone_id: int = 1, host: str = "127.0.0.1", ack_delay_s: float = 0.0,
                 b_ports=(defs.CH_B_SEND_PORT, defs.CH_B_RECV_PORT),
                 c_ports=(defs.CH_C_SEND_PORT, defs.CH_C_RECV_PORT)):
        load_dialect()
        self.drone_id = drone_id
        self.ack_delay_s = ack_delay_s
        self.drop_mission_items = 0  # drop the first N MISSION_ITEM_INTs, to exercise re-requests
        self.b = LinkWorker("fake_b", SplitPortLink(b_ports[0], host, b_ports[1], drone_id, defs.MC_COMPONENT_ID, host))
        self.c = LinkWorker("fake_c", SplitPortLink(c_ports[0], host, c_ports[1], drone_id, defs.MC_COMPONENT_ID, host))
        self.b.add_listener(self._on_b)
        self.c.add_listener(self._on_c)
        self.phase = 0
        # When set, MC_CMD_PHASE_TRANSITION is DENIED with this STATUSTEXT
        # reason, like mission_manager_node's guard rejections.
        self.phase_reject_reason: Optional[str] = None
        self.fault_mask = 0
        self.last_gcs_hb = None
        self.next_upload_id = 1
        self.received_commands: list[tuple[int, tuple]] = []
        self.uploads: list[list] = []
        self._upload = None
        self.plans: dict[str, bytes] = {}
        self.active = ""
        self._ftp_session = None
        self._stop = threading.Event()
        self._threads: list[threading.Thread] = []
        self._t0 = time.time()
        self._det_id = 0

    # --- lifecycle ---

    def start(self):
        self.b.start()
        self.c.start()
        for target in (self._loop_1hz, self._loop_2hz):
            th = threading.Thread(target=target, daemon=True)
            th.start()
            self._threads.append(th)
        return self

    def stop(self):
        self._stop.set()
        for th in self._threads:
            th.join(timeout=2)
        self.b.stop()
        self.c.stop()

    # --- Channel B ---

    def _on_b(self, msg, direction, t):
        if direction != "rx":
            return
        mtype = msg.get_type()
        mav = self.b.link.mav
        if mtype == "HEARTBEAT" and msg.type == defs.MAV_TYPE_GCS and msg.autopilot == defs.MAV_AUTOPILOT_INVALID:
            self.last_gcs_hb = t
        elif mtype == "COMMAND_LONG" and msg.target_system == self.drone_id and msg.target_component == defs.MC_COMPONENT_ID:
            params = (msg.param1, msg.param2, msg.param3, msg.param4, msg.param5, msg.param6, msg.param7)
            self.received_commands.append((msg.command, params))
            if self.ack_delay_s:
                time.sleep(self.ack_delay_s)
            if msg.command == defs.MC_CMD_PHASE_TRANSITION:
                # gcs_mavlink_node: IN_PROGRESS on receipt, then the verdict.
                mav.command_ack_send(msg.command, defs.MAV_RESULT_IN_PROGRESS, 255, 0,
                                     msg.get_srcSystem(), msg.get_srcComponent())
                if self.phase_reject_reason:
                    mav.command_ack_send(msg.command, 2, 255, 0, msg.get_srcSystem(), msg.get_srcComponent())
                    self._send_statustext(4, self.phase_reject_reason)
                    return
            mav.command_ack_send(msg.command, 0, 255, 0, msg.get_srcSystem(), msg.get_srcComponent())
            self._apply_command(msg.command, params)
        elif mtype == "MISSION_COUNT" and msg.mission_type == defs.MAV_MISSION_TYPE_MC:
            self._upload = {"count": msg.count, "items": [], "id": self.next_upload_id}
            self.next_upload_id += 1
            if msg.count == 0:
                self._finish_upload()
            else:
                mav.mission_request_int_send(255, 0, 0, defs.MAV_MISSION_TYPE_MC)
        elif mtype == "MISSION_ITEM_INT" and self._upload and msg.mission_type == defs.MAV_MISSION_TYPE_MC:
            if self.drop_mission_items > 0:
                self.drop_mission_items -= 1
                threading.Timer(0.3, lambda s=len(self._upload["items"]): mav.mission_request_int_send(
                    255, 0, s, defs.MAV_MISSION_TYPE_MC)).start()
                return
            if msg.seq != len(self._upload["items"]):
                return
            self._upload["items"].append(msg)
            if len(self._upload["items"]) == self._upload["count"]:
                self._finish_upload()
            else:
                mav.mission_request_int_send(255, 0, len(self._upload["items"]), defs.MAV_MISSION_TYPE_MC)

    def _finish_upload(self):
        up = self._upload
        self.uploads.append(up["items"])
        # pymavlink 2.4.49 can't pack opaque_id -- append it to the payload by hand.
        msg = self.b.link.mav.mission_ack_encode(255, 0, 0, defs.MAV_MISSION_TYPE_MC)
        buf = bytearray(msg.pack(self.b.link.mav))
        plen = buf[1]
        payload = bytes(buf[10:10 + plen]).ljust(4, b"\x00") + struct.pack("<I", up["id"])
        self._send_raw(self.b, msg.get_msgId(), payload, msg.crc_extra)
        self._upload = None

    def _send_raw(self, worker, msgid, payload, crc_extra):
        from pymavlink.generator.mavcrc import x25crc
        mav = worker.link.mav
        hdr = struct.pack("<BBBBBBBHB", 0xFD, len(payload), 0, 0, mav.seq, mav.srcSystem, mav.srcComponent,
                          msgid & 0xFFFF, msgid >> 16)
        mav.seq = (mav.seq + 1) % 256
        crc = x25crc(hdr[1:] + payload)
        crc.accumulate(struct.pack("B", crc_extra))
        worker.link.write(hdr + payload + struct.pack("<H", crc.crc))

    def _send_statustext(self, severity: int, text: str) -> None:
        """Same splitting as gcs_mavlink_node::send_statustext(): v2 chunks
        sharing one nonzero id when over 50 chars, empty terminator when the
        text fills its last chunk exactly."""
        data = text.encode()[:200]
        if len(data) <= 50:
            self.b.link.mav.statustext_send(severity, data)
            return
        self._statustext_id = getattr(self, "_statustext_id", 0) % 65535 + 1
        seq = 0
        while True:
            chunk = data[seq * 50:(seq + 1) * 50]
            self.b.link.mav.statustext_send(severity, chunk, self._statustext_id, seq)
            if len(chunk) < 50:
                return
            seq += 1

    def _apply_command(self, command, params):
        if command == defs.MC_CMD_PHASE_TRANSITION:
            self.phase = int(params[0])
        elif command == defs.MC_CMD_END_MISSION:
            self.phase = 8
        elif command == defs.MC_CMD_INDOOR_GNSS_SIM_ANCHOR:
            # gnss_sim_node's own confirmation (>50 chars, so chunked).
            self._send_statustext(
                6, f"gnss_sim_node: anchored false origin lat={params[0]:.7f} "
                   f"lon={params[1]:.7f} alt_msl={params[2]:.2f}")
        elif command in (defs.MAV_CMD_NAV_RETURN_TO_LAUNCH, defs.MAV_CMD_COMPONENT_ARM_DISARM, defs.MAV_CMD_DO_SET_MODE):
            self.phase = 255

    def _loop_1hz(self):
        while not self._stop.wait(1.0):
            gcs_alive = self.last_gcs_hb is not None and time.time() - self.last_gcs_hb < defs.GCS_LINK_TIMEOUT_S
            if self.last_gcs_hb is not None:
                if gcs_alive:
                    self.fault_mask &= ~(1 << 1)
                else:
                    self.fault_mask |= 1 << 1
            if self.last_gcs_hb is None:
                continue  # gcs_mavlink_node has no endpoint to send to yet
            mav = self.b.link.mav
            mav.heartbeat_send(defs.MAV_TYPE_ONBOARD_CONTROLLER, defs.MAV_AUTOPILOT_INVALID, 0, 0, 4)
            up = int(time.time() - self._t0)
            mav.mc_health_send(int(time.time() * 1e6), 20 + up % 15, 41, 37, 48, 52,
                               self.fault_mask, 0b001000110011, up)

    def _loop_2hz(self):
        while not self._stop.wait(0.5):
            if self.last_gcs_hb is None:
                continue
            t = time.time() - self._t0
            lat = 1.3521 + 0.0004 * math.sin(t / 30)
            lon = 103.8198 + 0.0004 * math.cos(t / 30)
            alt = 15.0 + (15.0 if self.phase not in (0, 1, 9) else 0.0)
            batt = max(0, 100 - int(t / 20))
            self.b.link.mav.mc_status_send(
                int(time.time() * 1e6), int(lat * 1e7), int(lon * 1e7), int(alt * 1000),
                int((t * 6) % 360 * 100), 500, 0, batt, self.phase, 0, self.fault_mask, 1, 0,
            )
            if self.phase == 7 and int(t * 2) % 8 == 0:
                self._det_id += 1
                self.b.link.mav.mc_detection_send(
                    int(time.time() * 1e6), self._det_id, 0, 8700, int((lat + 0.0001) * 1e7),
                    int(lon * 1e7), 0, 250, self.drone_id, b"person",
                )

    # --- Channel C ---

    def _ftp_reply(self, req, opcode, data=b"", offset=None, burst_complete=0, dest=(255, 190)):
        seq, session, req_op, size, _, _, _, off = req
        hdr = struct.pack("<HBBBBBBI", (seq + 1) & 0xFFFF, session, opcode, len(data), req_op,
                          burst_complete, 0, off if offset is None else offset)
        payload = (hdr + data).ljust(251, b"\x00")
        self.c.link.mav.file_transfer_protocol_send(0, dest[0], dest[1], payload)

    def _on_c(self, msg, direction, t):
        if direction != "rx":
            return
        if msg.get_type() == "MC_MISSION_PLAN_ACTIVATE":
            name = msg.filename if isinstance(msg.filename, str) else msg.filename.decode()
            name = name.rstrip("\x00")
            if name not in self.plans:
                result, err = 2, "not found"
            elif self.phase != 0:
                result, err = 3, "not in PREFLIGHT"
            else:
                try:
                    json.loads(self.plans[name])
                    result, err = 0, ""
                    self.active = name
                except ValueError:
                    result, err = 1, "invalid json"
            self.c.link.mav.mc_mission_plan_ack_send(msg.client_request_id, name.encode(), result, err.encode())
            return
        if msg.get_type() != "FILE_TRANSFER_PROTOCOL":
            return
        raw = bytes(msg.payload)
        req = struct.unpack("<HBBBBBBI", raw[:12])
        opcode, size, offset = req[2], req[3], req[7]
        data = raw[12:12 + size]
        name = data.decode("ascii", "replace").lstrip("/")
        nak = lambda code: self._ftp_reply(req, FTP_NAK, bytes([code]))  # noqa: E731
        if opcode == OP_LIST:
            entries = [f"F{n}\t{len(b)}".encode() + b"\x00" for n, b in sorted(self.plans.items())][offset:]
            if not entries:
                return nak(ERR_EOF)
            return self._ftp_reply(req, FTP_ACK, b"".join(entries[:5]))
        if opcode == OP_CREATE:
            if name in self.plans:
                return nak(ERR_EXISTS)
            self._ftp_session = {"name": name, "buf": bytearray(), "write": True}
            return self._ftp_reply(req, FTP_ACK)
        if opcode == OP_OPEN_WO:
            if name not in self.plans:
                return nak(ERR_NOT_FOUND)
            self._ftp_session = {"name": name, "buf": bytearray(), "write": True}
            return self._ftp_reply(req, FTP_ACK)
        if opcode == OP_WRITE and self._ftp_session:
            buf = self._ftp_session["buf"]
            if len(buf) < offset + len(data):
                buf.extend(b"\x00" * (offset + len(data) - len(buf)))
            buf[offset:offset + len(data)] = data
            return self._ftp_reply(req, FTP_ACK)
        if opcode == OP_OPEN_RO:
            content = self.active.encode() if name == "_active" else self.plans.get(name)
            if content is None:
                return nak(ERR_NOT_FOUND)
            self._ftp_session = {"name": name, "buf": bytearray(content), "write": False}
            return self._ftp_reply(req, FTP_ACK, struct.pack("<I", len(content)))
        if opcode == OP_BURST_READ and self._ftp_session:
            content = bytes(self._ftp_session["buf"])
            if not content:
                return nak(ERR_EOF)
            for off in range(0, len(content), 239):
                chunk = content[off:off + 239]
                self._ftp_reply(req, FTP_ACK, chunk, offset=off, burst_complete=int(off + 239 >= len(content)))
            return None
        if opcode == OP_REMOVE:
            if name == self.active:
                return nak(ERR_PROTECTED)
            if self.plans.pop(name, None) is None:
                return nak(ERR_NOT_FOUND)
            return self._ftp_reply(req, FTP_ACK)
        if opcode in (OP_TERMINATE, OP_RESET):
            s, self._ftp_session = self._ftp_session, None
            if s and s["write"]:
                self.plans[s["name"]] = bytes(s["buf"])
            return self._ftp_reply(req, FTP_ACK)
        return nak(1)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Fake MC for gcs-test-panel tests/demos (loopback only)")
    parser.add_argument("--drone-id", type=int, default=1)
    args = parser.parse_args(argv)
    mc = FakeMC(args.drone_id).start()
    print(f"fake MC sysid={args.drone_id} on 127.0.0.1 -- Channel B :{defs.CH_B_SEND_PORT}, "
          f"Channel C :{defs.CH_C_SEND_PORT}; Ctrl-C to stop")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    mc.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
