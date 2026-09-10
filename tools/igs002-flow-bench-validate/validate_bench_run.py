#!/usr/bin/env python3
"""IGS-002 (HIF-008 Holybro H-Flow) bench validation.

Parses a `ros2 bag record /mc/sensors/optical_flow` recording, integrates
the recorded body-frame velocity over the motion window, and compares the
result against a known lateral displacement -- step 3-4 of
`test-results/IGS-002-hif-008-bench-validation.md`'s "To complete this
ticket" procedure.

Requires the workspace's `mc_msgs` Python bindings on PYTHONPATH (source
`install/setup.bash` first) plus ROS 2's `rosbag2_py`/`rclpy` -- no extra
pip installs.

Usage:
    ros2 bag record /mc/sensors/optical_flow -o flow_run_01
    # ... perform the calibrated motion during the recording, then Ctrl-C ...
    ./validate_bench_run.py flow_run_01 --known-x 2.0 --rig slide

Assumptions / caveats (read before trusting the numbers):
  * `vx`/`vy` are body-FRD (forward/right), not rotated to a world frame
    (ICD-MC-001.md Sec 3.3a) -- valid for a rigid bench slide/hand-carry
    test with no yaw during the recording, not for an in-flight run.
  * `fcc_bridge_node` republishes the latest decoded sample on its own
    fixed publish-timer tick (`FccBridgeNode::on_publish_timer`), not once
    per incoming MAVLink OPTICAL_FLOW_RAD frame. The "topic rate" this
    script reports is therefore the MC-side publish rate, not the H-Flow's
    own underlying sample rate -- expected, not a defect, but don't
    conflate the two when filling in the results table.
  * Integration is trapezoidal over consecutive samples' timestamps
    (`header.stamp`, falling back to bag receive time if unset/
    non-monotonic -- see the stderr note this script prints in that case).
"""
import argparse
import pathlib
import sys

import yaml


def detect_storage_id(bag_path: pathlib.Path) -> str:
    metadata_path = bag_path / "metadata.yaml"
    if not metadata_path.is_file():
        return "sqlite3"
    with metadata_path.open() as f:
        metadata = yaml.safe_load(f)
    return metadata["rosbag2_bagfile_information"]["storage_identifier"]


def read_flow_samples(bag_path: pathlib.Path, topic: str):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from mc_msgs.msg import OpticalFlow

    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_path), storage_id=detect_storage_id(bag_path)
    )
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))

    samples = []
    while reader.has_next():
        _, data, bag_t_ns = reader.read_next()
        samples.append((bag_t_ns, deserialize_message(data, OpticalFlow)))
    return samples


def sample_time_s(bag_t_ns: int, msg, use_header_stamp: bool) -> float:
    if use_header_stamp:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    return bag_t_ns * 1e-9


def integrate(samples):
    any_stamped = any((m.header.stamp.sec or m.header.stamp.nanosec) for _, m in samples)
    use_header_stamp = any_stamped
    times = [sample_time_s(t, m, use_header_stamp) for t, m in samples]

    if use_header_stamp and times != sorted(times):
        print(
            "WARNING: header.stamp is non-monotonic across samples -- falling "
            "back to bag-recorded receive time for integration.",
            file=sys.stderr,
        )
        use_header_stamp = False
        times = [sample_time_s(t, m, False) for t, m in samples]

    dx = dy = 0.0
    for i in range(1, len(samples)):
        dt = times[i] - times[i - 1]
        if dt <= 0:
            continue
        vx0, vx1 = samples[i - 1][1].vx, samples[i][1].vx
        vy0, vy1 = samples[i - 1][1].vy, samples[i][1].vy
        dx += 0.5 * (vx0 + vx1) * dt
        dy += 0.5 * (vy0 + vy1) * dt
    return dx, dy, times, use_header_stamp


def pct_error_str(measured: float, known: float) -> str:
    if abs(known) < 1e-6:
        return "n/a (known ~ 0)"
    return f"{abs(measured - known) / abs(known) * 100:.1f}%"


def verdict(measured: float, known: float, tol_pct: float) -> str:
    if abs(known) < 1e-6:
        return "PASS" if abs(measured) < 0.05 else "FAIL"
    err_pct = abs(measured - known) / abs(known) * 100
    return "PASS" if err_pct <= tol_pct else "FAIL"


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("bag_path", type=pathlib.Path, help="path to the ros2 bag directory")
    parser.add_argument("--topic", default="/mc/sensors/optical_flow")
    parser.add_argument(
        "--known-x", type=float, default=0.0,
        help="known displacement along body-forward X, m (signed)",
    )
    parser.add_argument(
        "--known-y", type=float, default=0.0,
        help="known displacement along body-right Y, m (signed)",
    )
    parser.add_argument(
        "--rig", choices=["slide", "manual"], default="manual",
        help="calibrated slide rig (5%% tolerance) vs. hand-measured (15%%), "
        "per the bench-validation doc's proposed tolerances",
    )
    parser.add_argument(
        "--tolerance-pct", type=float, default=None,
        help="override the --rig default tolerance",
    )
    parser.add_argument(
        "--min-quality", type=int, default=1,
        help="flag samples at or below this quality as invalid (0 = no return)",
    )
    parser.add_argument(
        "--out", type=pathlib.Path, default=None,
        help="also write the markdown snippet to this file",
    )
    args = parser.parse_args()

    tol_pct = (
        args.tolerance_pct if args.tolerance_pct is not None
        else (5.0 if args.rig == "slide" else 15.0)
    )

    samples = read_flow_samples(args.bag_path, args.topic)
    if len(samples) < 2:
        sys.exit(
            f"Only {len(samples)} sample(s) on {args.topic} in {args.bag_path} "
            "-- need at least 2 to integrate."
        )

    dx, dy, times, used_header_stamp = integrate(samples)
    duration = times[-1] - times[0]
    rate_hz = (len(samples) - 1) / duration if duration > 0 else float("nan")
    qualities = [m.quality for _, m in samples]
    n_bad_quality = sum(1 for q in qualities if q <= args.min_quality)

    lines = [
        "```",
        f"bag: {args.bag_path}",
        f"topic: {args.topic}",
        f"samples: {len(samples)}  duration: {duration:.2f}s  rate: {rate_hz:.1f} Hz",
        "timestamp source: "
        + ("header.stamp" if used_header_stamp else "bag receive time (header.stamp unset/non-monotonic)"),
        f"quality: min={min(qualities)} mean={sum(qualities) / len(qualities):.1f} "
        f"samples<=min-quality({args.min_quality})={n_bad_quality}/{len(samples)}",
        f"integrated displacement: X(fwd)={dx:+.3f} m  Y(right)={dy:+.3f} m",
        "```",
        "",
        "| Check | Known/measured value | Sensor-derived value | Result |",
        "|---|---|---|---|",
        f"| Lateral displacement (X) | {args.known_x:+.3f} m | "
        f"{dx:+.3f} m ({pct_error_str(dx, args.known_x)}) | "
        f"{verdict(dx, args.known_x, tol_pct)} |",
        f"| Lateral displacement (Y) | {args.known_y:+.3f} m | "
        f"{dy:+.3f} m ({pct_error_str(dy, args.known_y)}) | "
        f"{verdict(dy, args.known_y, tol_pct)} |",
        f"| MC-side relay topic rate | -- | {rate_hz:.1f} Hz | "
        f"{'PASS' if rate_hz > 0 else 'FAIL'} |",
        f"| `quality` during motion | > 0, stable | "
        f"min={min(qualities)}, mean={sum(qualities) / len(qualities):.1f} | "
        f"{'PASS' if n_bad_quality == 0 else f'FAIL ({n_bad_quality} sample(s) <= {args.min_quality})'} |",
    ]
    output = "\n".join(lines)
    print(output)

    if args.out:
        args.out.write_text(output + "\n")
        print(f"\nWritten to {args.out}", file=sys.stderr)

    if not used_header_stamp:
        print(
            "\nNote: integrated using bag receive time, not header.stamp -- "
            "adds ROS 2 scheduling jitter to the per-sample dt. Re-run with a "
            "fresh bag if this run's samples all had a zero/unset stamp.",
            file=sys.stderr,
        )

    print(
        "\nPaste the block above into test-results/IGS-002-hif-008-bench-validation.md "
        "in place of its placeholder code block and table.",
        file=sys.stderr,
    )


if __name__ == "__main__":
    main()
