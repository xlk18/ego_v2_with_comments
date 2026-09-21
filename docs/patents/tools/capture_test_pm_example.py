#!/usr/bin/env python3
"""Capture the latched ``test_pm`` Path message and render publication evidence."""

from __future__ import annotations

import argparse
import atexit
import csv
import json
import math
import os
import re
import shutil
import subprocess
import tempfile
import threading
import time
import xmlrpc.client
from pathlib import Path
from typing import TYPE_CHECKING, Sequence

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager

if TYPE_CHECKING:
    from nav_msgs.msg import Path as RosPath


TOPIC = "/generated_trajectory"
FRAME_ID = "map"
SAMPLE_PERIOD_S = 0.03
NODE_NAME = "pmps_test_node"
ACCELERATION_UPPER_MPS2 = 20.0
ACCELERATION_LOWER_MPS2 = -20.0
CONFIGURED_CIRCLE_POINTS = 7
RADIUS_M = 25.0
HEIGHT_M = 2.0
START_POSITION_M = (0.0, 0.0, 2.0)
START_VELOCITY_MPS = (0.0, 0.0, 0.0)
FIX_REPLAN = False
_TEMPORARY_FONT: Path | None = None
_ANSI_CSI_PATTERN = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
_COMPUTE_TIME_PATTERN = re.compile(
    r"Trajectory calculation finished in\s+"
    r"((?:0|[1-9][0-9]*)(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?)\s+seconds\."
)


def _positive_finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be a positive finite number; got {value!r}")
    return value


def _remaining_seconds(deadline: float) -> float:
    remaining = deadline - time.monotonic()
    if remaining <= 0.0:
        raise TimeoutError("capture deadline expired")
    return remaining


def _wait_for_master(timeout: float, master_uri: str | None = None) -> None:
    """Bound master availability probing without changing global socket settings."""
    timeout = _positive_finite(timeout, "timeout")
    master_uri = master_uri or os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
    deadline = time.monotonic() + timeout
    last_error: Exception | None = None
    while True:
        remaining = _remaining_seconds(deadline)
        finished = threading.Event()
        result: dict[str, object] = {}

        def probe() -> None:
            try:
                result["value"] = xmlrpc.client.ServerProxy(master_uri).getPid("/capture_test_pm_example")
            except Exception as exc:  # XML-RPC errors are retried until the deadline.
                result["error"] = exc
            finally:
                finished.set()

        threading.Thread(target=probe, daemon=True).start()
        if not finished.wait(remaining):
            raise TimeoutError(f"timed out after {timeout:g} s waiting for ROS master at {master_uri!r}")
        if "value" in result:
            return
        last_error = result.get("error")  # type: ignore[assignment]
        try:
            time.sleep(min(0.05, _remaining_seconds(deadline)))
        except TimeoutError as exc:
            raise TimeoutError(
                f"timed out after {timeout:g} s waiting for ROS master at {master_uri!r}"
            ) from last_error


def _initialize_ros_node(rospy: object, timeout: float) -> None:
    """Bound ROS initialization after the master preflight succeeds."""
    timeout = _positive_finite(timeout, "timeout")
    finished = threading.Event()
    result: dict[str, Exception] = {}

    def initialize() -> None:
        try:
            rospy.init_node("capture_test_pm_example", anonymous=True, disable_signals=True)  # type: ignore[attr-defined]
        except Exception as exc:
            result["error"] = exc
        finally:
            finished.set()

    threading.Thread(target=initialize, daemon=True).start()
    if not finished.wait(timeout):
        raise TimeoutError(f"timed out after {timeout:g} s initializing the ROS capture node")
    if "error" in result:
        raise RuntimeError("could not initialize the ROS capture node") from result["error"]


def wait_for_path(topic: str, timeout: float) -> RosPath:
    """Return one nonempty map-frame Path, or a precise capture error."""
    if not isinstance(topic, str) or not topic.strip():
        raise ValueError("topic must be a nonempty string")
    timeout = _positive_finite(timeout, "timeout")
    deadline = time.monotonic() + timeout

    try:
        import rospy
        from nav_msgs.msg import Path as RosPathMessage
    except ImportError as exc:  # Keep pure helpers usable outside a ROS environment.
        raise RuntimeError("ROS Noetic Python modules are required to capture a Path") from exc

    _wait_for_master(_remaining_seconds(deadline))
    if not rospy.core.is_initialized():
        _initialize_ros_node(rospy, _remaining_seconds(deadline))
    remaining = _remaining_seconds(deadline)

    received: dict[str, object] = {}
    ready = threading.Event()

    def receive(message: RosPathMessage) -> None:
        if not message.poses:
            received["empty"] = True
            return
        if message.header.frame_id != FRAME_ID:
            received["error"] = ValueError(
                f"Path frame_id must be {FRAME_ID!r}; got {message.header.frame_id!r}"
            )
        else:
            received["message"] = message
        ready.set()

    subscriber = rospy.Subscriber(topic, RosPathMessage, receive, queue_size=1)
    try:
        if not ready.wait(remaining):
            if received.get("empty"):
                raise TimeoutError(
                    f"received an empty Path on {topic!r} but no nonempty Path within {timeout:g} s"
                )
            raise TimeoutError(f"timed out after {timeout:g} s waiting for a nonempty Path on {topic!r}")
        if "error" in received:
            raise received["error"]  # type: ignore[misc]
        return received["message"]  # type: ignore[return-value]
    finally:
        subscriber.unregister()


def path_samples(message: RosPath, dt: float) -> np.ndarray:
    """Convert every published pose position into deterministic ``t,x,y,z`` samples."""
    dt = _positive_finite(dt, "dt")
    poses = getattr(message, "poses", None)
    if not poses:
        raise ValueError("Path must contain at least one pose")

    samples = np.empty((len(poses), 4), dtype=float)
    for index, pose_stamped in enumerate(poses):
        position = pose_stamped.pose.position
        values = (float(position.x), float(position.y), float(position.z))
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"Path pose {index} has nonfinite position values: {values!r}")
        samples[index] = (index * dt, *values)
    return samples


def circular_waypoints(count: int, radius: float, height: float) -> np.ndarray:
    """Return the source-defined start point followed by its circular waypoints."""
    if not isinstance(count, int) or isinstance(count, bool) or count <= 0:
        raise ValueError(f"count must be a positive integer; got {count!r}")
    radius = _positive_finite(radius, "radius")
    height = float(height)
    if not math.isfinite(height):
        raise ValueError(f"height must be finite; got {height!r}")

    waypoints = np.empty((count + 1, 3), dtype=float)
    waypoints[0] = (0.0, 0.0, height)
    for index in range(1, count + 1):
        angle = 2.0 * math.pi * index / count
        waypoints[index] = (
            radius * (1.0 - math.cos(angle)),
            radius * math.sin(angle),
            height,
        )
    return waypoints


def parse_compute_time(log_path: Path) -> float:
    """Read the one source-format computation-time line from a node log."""
    log_path = Path(log_path)
    try:
        contents = log_path.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        raise ValueError(f"cannot read node log {log_path}: {exc}") from exc
    matches = _COMPUTE_TIME_PATTERN.findall(_ANSI_CSI_PATTERN.sub("", contents))
    if not matches:
        raise ValueError(
            "node log lacks the computation-time line "
            "'Trajectory calculation finished in <seconds> seconds.'"
        )
    if len(matches) != 1:
        raise ValueError(f"node log has {len(matches)} computation-time lines; expected exactly one")
    compute_time = float(matches[0])
    if not math.isfinite(compute_time) or compute_time <= 0.0:
        raise ValueError(f"computation time must be finite and positive; got {compute_time!r}")
    return compute_time


def _prepare_output_parents(paths: Sequence[Path]) -> None:
    for path in paths:
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            raise ValueError(f"cannot create output parent {path.parent}: {exc}") from exc


def _staged_output_path(destination: Path) -> Path:
    descriptor, name = tempfile.mkstemp(
        dir=str(destination.parent), prefix=f".{destination.name}.", suffix=".tmp"
    )
    os.close(descriptor)
    return Path(name)


def _validate_staged_outputs(csv_path: Path, summary_path: Path, plot_path: Path) -> None:
    with csv_path.open(encoding="utf-8", newline="") as stream:
        if next(csv.reader(stream), None) != ["t_s", "x_m", "y_m", "z_m"]:
            raise ValueError("staged CSV lacks the required header")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    if not isinstance(summary, dict) or "computation_time_s" not in summary:
        raise ValueError("staged JSON lacks required summary metadata")
    rendered = plt.imread(plot_path, format="png")
    if rendered.shape[:2] != (900, 1800):
        raise ValueError(f"staged plot must be 1800x900 pixels; got {rendered.shape[1]}x{rendered.shape[0]}")


def _commit_staged_outputs(staged: Sequence[Path], destinations: Sequence[Path]) -> None:
    """Replace all destinations, rolling back existing files if replacement fails."""
    backups: dict[Path, Path] = {}
    replaced: list[Path] = []
    try:
        for destination in destinations:
            if destination.exists():
                backup = _staged_output_path(destination)
                shutil.copy2(destination, backup)
                backups[destination] = backup
        for temporary, destination in zip(staged, destinations):
            os.replace(temporary, destination)
            replaced.append(destination)
    except Exception:
        for destination in replaced:
            backup = backups.get(destination)
            if backup is None:
                destination.unlink(missing_ok=True)
            else:
                os.replace(backup, destination)
        raise
    finally:
        for temporary in (*staged, *backups.values()):
            temporary.unlink(missing_ok=True)


def _remove_temporary_font() -> None:
    if _TEMPORARY_FONT is not None:
        _TEMPORARY_FONT.unlink(missing_ok=True)


atexit.register(_remove_temporary_font)


def _configure_plot_font() -> None:
    """Register the required font, including its face inside a font collection."""
    try:
        font_manager.findfont("Noto Sans CJK SC", fallback_to_default=False)
    except ValueError:
        result = subprocess.run(
            ["fc-match", "-f", "%{file}\\n%{index}\\n", "Noto Sans CJK SC"],
            check=True,
            capture_output=True,
            text=True,
        )
        location, index_text = result.stdout.splitlines()[:2]
        source = Path(location)
        if not source.is_file():
            raise RuntimeError("Noto Sans CJK SC is required but fontconfig did not locate it")
        index = int(index_text)
        global _TEMPORARY_FONT
        if source.suffix.lower() == ".ttc":
            try:
                from fontTools.ttLib import TTCollection
            except ImportError as exc:
                raise RuntimeError(
                    "Matplotlib cannot register the Noto Sans CJK SC collection; "
                    "install fontTools or a standalone Noto Sans CJK SC font"
                ) from exc
            with tempfile.NamedTemporaryFile(suffix=".ttf", delete=False) as stream:
                _TEMPORARY_FONT = Path(stream.name)
            collection = TTCollection(str(source), lazy=True)
            collection.fonts[index].save(str(_TEMPORARY_FONT))
            source = _TEMPORARY_FONT
        font_manager.fontManager.addfont(str(source))
        try:
            font_manager.findfont("Noto Sans CJK SC", fallback_to_default=False)
        except ValueError as exc:
            _remove_temporary_font()
            _TEMPORARY_FONT = None
            raise RuntimeError("could not register Noto Sans CJK SC with Matplotlib") from exc
        plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC"]
        plt.rcParams["axes.unicode_minus"] = False
        return
    plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC"]
    plt.rcParams["axes.unicode_minus"] = False


def write_outputs(
    samples: np.ndarray,
    waypoints: np.ndarray,
    compute_time: float,
    csv_path: Path,
    summary_path: Path,
    plot_path: Path,
    topic: str = TOPIC,
) -> None:
    """Write deterministic CSV/JSON evidence and the 1800 by 900 pixel figure."""
    samples = np.asarray(samples, dtype=float)
    waypoints = np.asarray(waypoints, dtype=float)
    if samples.ndim != 2 or samples.shape[0] == 0 or samples.shape[1] != 4:
        raise ValueError("samples must be a nonempty N-by-4 array of t,x,y,z values")
    if waypoints.ndim != 2 or waypoints.shape[1] != 3:
        raise ValueError("waypoints must be an N-by-3 array")
    if not np.isfinite(samples).all() or not np.isfinite(waypoints).all():
        raise ValueError("samples and waypoints must contain only finite values")
    compute_time = float(compute_time)
    if not math.isfinite(compute_time) or compute_time <= 0.0:
        raise ValueError("compute_time must be finite and positive")
    if not isinstance(topic, str) or not topic.strip():
        raise ValueError("topic must be a nonempty string")

    destinations = tuple(map(Path, (csv_path, summary_path, plot_path)))
    if len(set(destinations)) != len(destinations):
        raise ValueError("csv, summary, and plot destinations must be distinct")
    _prepare_output_parents(destinations)
    staged: list[Path] = []
    try:
        for destination in destinations:
            staged.append(_staged_output_path(destination))
    except Exception:
        for temporary in staged:
            temporary.unlink(missing_ok=True)
        raise
    staged_csv, staged_summary, staged_plot = staged

    point_count = int(samples.shape[0])
    summary = {
        "node": NODE_NAME,
        "topic": topic,
        "frame_id": FRAME_ID,
        "sample_period_s": SAMPLE_PERIOD_S,
        "point_count": point_count,
        "sampled_duration_s": float(samples[-1, 0]),
        "computation_time_s": compute_time,
        "acceleration_upper_mps2": ACCELERATION_UPPER_MPS2,
        "acceleration_lower_mps2": ACCELERATION_LOWER_MPS2,
        "configured_circle_points": CONFIGURED_CIRCLE_POINTS,
        "total_waypoints_including_start": int(waypoints.shape[0]),
        "radius_m": RADIUS_M,
        "height_m": HEIGHT_M,
        "start_position_m": list(START_POSITION_M),
        "start_velocity_mps": list(START_VELOCITY_MPS),
        "fix_replan": FIX_REPLAN,
    }
    try:
        with staged_csv.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(("t_s", "x_m", "y_m", "z_m"))
            writer.writerows(samples.tolist())
        staged_summary.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

        _configure_plot_font()
        figure = plt.figure(figsize=(12, 6), dpi=150, constrained_layout=True)
        try:
            trajectory = figure.add_subplot(1, 2, 1, projection="3d")
            trajectory.plot(samples[:, 1], samples[:, 2], samples[:, 3], label="Published Path", color="#0072B2")
            trajectory.scatter(*waypoints[0], label="Source start", color="#009E73", s=45, zorder=3)
            trajectory.scatter(
                waypoints[1:, 0],
                waypoints[1:, 1],
                waypoints[1:, 2],
                label="Source circular waypoints",
                color="#D55E00",
                marker="^",
                s=40,
                zorder=3,
            )
            trajectory.set_xlabel("x (m)")
            trajectory.set_ylabel("y (m)")
            trajectory.set_zlabel("z (m)")
            trajectory.set_title("Published trajectory and source waypoints")
            trajectory.grid(True)
            trajectory.legend(loc="best")

            time_series = figure.add_subplot(1, 2, 2)
            time_series.plot(samples[:, 0], samples[:, 1], label="x(t)", color="#0072B2")
            time_series.plot(samples[:, 0], samples[:, 2], label="y(t)", color="#D55E00")
            time_series.plot(samples[:, 0], samples[:, 3], label="z(t)", color="#009E73")
            time_series.set_xlabel("time (s)")
            time_series.set_ylabel("position (m)")
            time_series.set_title(f"Published positions: {point_count} points, {samples[-1, 0]:.2f} s")
            time_series.grid(True)
            time_series.legend()
            figure.savefig(staged_plot, dpi=150, format="png")
        finally:
            plt.close(figure)
        _validate_staged_outputs(staged_csv, staged_summary, staged_plot)
        _commit_staged_outputs((staged_csv, staged_summary, staged_plot), destinations)
    finally:
        for temporary in (staged_csv, staged_summary, staged_plot):
            temporary.unlink(missing_ok=True)


def _argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default=TOPIC, help=f"Path topic (default: {TOPIC})")
    parser.add_argument("--timeout", type=float, default=30.0, help="capture timeout in seconds")
    parser.add_argument("--node-log", required=True, type=Path, help="captured test_pm node log")
    parser.add_argument("--csv", required=True, type=Path, help="destination CSV path")
    parser.add_argument("--summary", required=True, type=Path, help="destination JSON summary path")
    parser.add_argument("--plot", required=True, type=Path, help="destination 1800x900 PNG path")
    return parser


def main() -> None:
    args = _argument_parser().parse_args()
    samples = path_samples(wait_for_path(args.topic, args.timeout), SAMPLE_PERIOD_S)
    waypoints = circular_waypoints(CONFIGURED_CIRCLE_POINTS, RADIUS_M, HEIGHT_M)
    write_outputs(
        samples, waypoints, parse_compute_time(args.node_log), args.csv, args.summary, args.plot, args.topic
    )


if __name__ == "__main__":
    main()
