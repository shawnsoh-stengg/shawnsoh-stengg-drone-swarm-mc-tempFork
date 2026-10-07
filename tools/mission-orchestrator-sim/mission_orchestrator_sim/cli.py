"""mission-orchestrator-sim CLI (TEST-004).

One subcommand per plan-library CRUD action, mirroring the Mission
Orchestrator app's own actions over Channel C
(MISSION-ORCHESTRATOR-MAVLINK-INTERFACE-SPEC.md Sec4-Sec7):

    mission-orchestrator-sim create   <name> <plan.json> --mc-host <ip>
    mission-orchestrator-sim list                        --mc-host <ip>
    mission-orchestrator-sim read     <name>              --mc-host <ip>
    mission-orchestrator-sim update   <name> <plan.json> --mc-host <ip>
    mission-orchestrator-sim delete   <name>              --mc-host <ip>
    mission-orchestrator-sim activate <name>              --mc-host <ip>

See this directory's README.md for the full option reference and example
session.
"""

from __future__ import annotations

import argparse
import sys

from . import dialect
from .transport import ChannelCClient, ChannelCLink, MavftpError

# config/mavlink-router-channel-c.conf / ICD-MC-001.md Sec11.4's
# ORCHESTRATOR_MAV_PORT_RECV/ORCHESTRATOR_MAV_PORT_SEND defaults.
DEFAULT_RECV_PORT = 14571
DEFAULT_SEND_PORT = 14572

_ACTIVATE_RESULTS = {
    0: "ACCEPTED",
    1: "REJECTED_INVALID_JSON",
    2: "REJECTED_NOT_FOUND",
    3: "REJECTED_PHASE",
}


def _plan_filename(name: str) -> str:
    return name if name.endswith(".json") else f"{name}.json"


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="mission-orchestrator-sim",
        description="Simulates the Mission Orchestrator app's mission-plan CRUD actions "
        "over MAVFTP on Channel C (TEST-004).",
    )
    parser.add_argument(
        "--mc-host",
        required=True,
        help="MC's Channel C address (the Jetson's LAN IP, e.g. 192.168.100.11; "
        "use 127.0.0.1 against a loopback-rewritten mavlink-router-channel-c.conf for local testing)",
    )
    parser.add_argument(
        "--drone-id",
        type=int,
        default=1,
        help="Expected system_id of the drone at --mc-host (its DRONE_ID, default: 1) -- "
        "verifies replies actually came from it; wire-level addressing is by --mc-host "
        "plus MAVLink broadcast, see transport.py's ChannelCClient docstring",
    )
    parser.add_argument(
        "--recv-port",
        type=int,
        default=DEFAULT_RECV_PORT,
        help=f"Local UDP port this tool listens on (ORCHESTRATOR_MAV_PORT_RECV, default: {DEFAULT_RECV_PORT})",
    )
    parser.add_argument(
        "--send-port",
        type=int,
        default=DEFAULT_SEND_PORT,
        help=f"MC's Channel C UDP port this tool sends to (ORCHESTRATOR_MAV_PORT_SEND, default: {DEFAULT_SEND_PORT})",
    )
    parser.add_argument(
        "--timeout", type=float, default=5.0, help="Per-request reply timeout in seconds (default: 5.0)"
    )
    parser.add_argument(
        "--regen-dialect",
        action="store_true",
        help="Force-regenerate the mc_mavlink_defs dialect module instead of using the cached one",
    )

    sub = parser.add_subparsers(dest="command", required=True)

    p_create = sub.add_parser("create", help="Upload a new plan (Nak(FileExists) if <name>.json already exists)")
    p_create.add_argument("name", help="Plan name, e.g. recon_alpha (identity is the filename, not a separate ID)")
    p_create.add_argument("plan_json", help="Local path to the plan's JSON content")

    sub.add_parser("list", help="List all stored plans")

    p_read = sub.add_parser("read", help="Download a stored plan's JSON content to stdout")
    p_read.add_argument("name", help="Plan name, or '_active' for the reserved active-plan-filename entry")
    p_read.add_argument("-o", "--output", help="Write to this file instead of stdout")

    p_update = sub.add_parser("update", help="Replace an existing plan (Nak(FileNotFound) if it doesn't exist)")
    p_update.add_argument("name", help="Plan name, e.g. recon_alpha")
    p_update.add_argument("plan_json", help="Local path to the plan's new JSON content")

    p_delete = sub.add_parser("delete", help="Delete a stored plan (Nak(FileProtected) if it's the active plan)")
    p_delete.add_argument("name", help="Plan name, e.g. recon_alpha")

    p_activate = sub.add_parser("activate", help="Activate a stored plan (MC_MISSION_PLAN_ACTIVATE/ACK)")
    p_activate.add_argument("name", help="Plan name, e.g. recon_alpha")
    p_activate.add_argument(
        "--request-id", type=int, default=1, help="client_request_id echoed back in the ack (default: 1)"
    )

    return parser


def _connect(args) -> ChannelCClient:
    module = dialect.load_dialect(force_regen=args.regen_dialect)
    dialect.install_dialect(module)
    link = ChannelCLink(args.recv_port, args.mc_host, args.send_port)
    return ChannelCClient(link, mc_system_id=args.drone_id, timeout=args.timeout)


def _run(args) -> int:
    client = _connect(args)

    if args.command == "create":
        with open(args.plan_json, "rb") as f:
            data = f.read()
        client.create_or_update(_plan_filename(args.name), data, update=False)
        print(f"OK: created {_plan_filename(args.name)} ({len(data)} bytes)")

    elif args.command == "update":
        with open(args.plan_json, "rb") as f:
            data = f.read()
        client.create_or_update(_plan_filename(args.name), data, update=True)
        print(f"OK: updated {_plan_filename(args.name)} ({len(data)} bytes)")

    elif args.command == "list":
        entries = client.list_directory()
        active = client.read_active_filename()
        if not entries:
            print("(no stored plans)")
        for entry in entries:
            marker = " *" if entry.name == active else ""
            print(f"{entry.name}\t{entry.size} bytes{marker}")
        print(f"active plan: {active or '(none)'}")

    elif args.command == "read":
        name = args.name if args.name == "_active" else _plan_filename(args.name)
        data = client.read_plan(name) if name != "_active" else client.read_active_filename().encode("ascii")
        if args.output:
            with open(args.output, "wb") as f:
                f.write(data)
            print(f"OK: wrote {len(data)} bytes to {args.output}")
        else:
            sys.stdout.buffer.write(data)
            if not data.endswith(b"\n"):
                sys.stdout.write("\n")

    elif args.command == "delete":
        client.remove_plan(_plan_filename(args.name))
        print(f"OK: deleted {_plan_filename(args.name)}")

    elif args.command == "activate":
        ack = client.activate(_plan_filename(args.name), args.request_id)
        result_name = _ACTIVATE_RESULTS.get(ack.result, f"UNKNOWN({ack.result})")
        print(
            f"MC_MISSION_PLAN_ACK: client_request_id={ack.client_request_id} "
            f"filename={ack.filename} result={result_name}"
        )
        if ack.error_message:
            print(f"  error_message: {ack.error_message}")
        return 0 if ack.result == 0 else 1

    return 0


def main(argv=None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    try:
        return _run(args)
    except MavftpError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    except FileNotFoundError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
