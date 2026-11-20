#!/usr/bin/env python3
"""Write report.md and plots from filter benchmark results."""

import argparse
from itertools import combinations
import json
from pathlib import Path
import sys

import numpy

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "robot_localization_parity"))
import parity_metrics  # noqa: E402

_ORDER = ("ekf", "ukf", "inekf")


def _track(result: dict) -> parity_metrics.Track:
    track = parity_metrics.Track()
    data = result["track"]
    track.time = data["time"]
    track.position = [tuple(p) for p in data["position"]]
    track.yaw = data["yaw"]
    track.velocity = [tuple(v) for v in data["velocity"]]
    track.position_variance = [tuple(v) for v in data["position_variance"]]
    return track


def _accuracy_table(results: dict[str, dict]) -> list[str]:
    gnss = any("gnss_rms_m" in r for r in results.values())
    header = "| Filter | End error [m] | Rate [Hz] | CPU [s] |"
    divider = "|---|---|---|---|"
    if gnss:
        header = "| Filter | End error [m] | GNSS fit RMS [m] | Rate [Hz] | CPU [s] |"
        divider = "|---|---|---|---|---|"
    lines = [header, divider]
    for name in _ORDER:
        if name not in results:
            continue
        r = results[name]
        s = r["summary"]
        row = f"| {name.upper()} | {s['final_error_m']:.3f} |"
        if gnss:
            row += f" {r.get('gnss_rms_m', float('nan')):.2f} |"
        row += f" {s['rate_hz']:.0f} | {r['cpu_seconds']:.1f} |"
        lines.append(row)
    return lines


def _difference_table(results: dict[str, dict]) -> list[str]:
    lines = ["| Pair | Position diff RMS / max [m] | Yaw diff RMS [deg] |", "|---|---|---|"]
    names = [name for name in _ORDER if name in results]
    for a, b in combinations(names, 2):
        d = parity_metrics.compare(_track(results[a]), _track(results[b]))
        lines.append(f"| {a.upper()} / {b.upper()} | {d['position_rms_m']:.3f} / "
                     f"{d['position_max_m']:.3f} | {d['yaw_rms_deg']:.2f} |")
    return lines


def _plot(bag: str, results: dict[str, dict], output: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, (left, right) = plt.subplots(1, 2, figsize=(12, 5))
    expected = next(iter(results.values()))["expected_final_position"]
    for name in _ORDER:
        if name not in results:
            continue
        track = {key: numpy.asarray(value) for key, value in results[name]["track"].items()}
        left.plot(track["position"][:, 0], track["position"][:, 1], label=name.upper())
        right.plot(track["time"] - track["time"][0], track["position"][:, 2], label=name.upper())
    left.plot(*expected[:2], "k+", markersize=12, label="upstream end point")
    left.set_xlabel("x [m]")
    left.set_ylabel("y [m]")
    left.set_aspect("equal", adjustable="datalim")
    left.legend()
    left.set_title(f"{bag}: track")
    right.axhline(expected[2], color="k", linestyle=":", label="upstream end z")
    right.set_xlabel("time [s]")
    right.set_ylabel("z [m]")
    right.legend()
    right.set_title(f"{bag}: height")
    figure.tight_layout()
    figure.savefig(output, dpi=80)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results", type=Path, help="BENCHMARK_RESULTS_DIR")
    parser.add_argument("--output", type=Path, required=True, help="report directory")
    parser.add_argument("--no-plots", action="store_true")
    arguments = parser.parse_args()
    arguments.output.mkdir(parents=True, exist_ok=True)

    by_bag: dict[str, dict[str, dict]] = {}
    for path in sorted(arguments.results.glob("test*_*.json")):
        result = json.loads(path.read_text())
        by_bag.setdefault(result["bag"], {})[result["filter"]] = result
    if not by_bag:
        raise SystemExit(f"no results in {arguments.results}")

    report = ["# EKF vs UKF vs InEKF", ""]
    for bag, results in sorted(by_bag.items()):
        tolerance = next(iter(results.values()))["upstream_tolerance_m"]
        report += [f"## {bag}", "", f"Upstream end point tolerance: {tolerance:.2f} m.", ""]
        report += _accuracy_table(results) + [""] + _difference_table(results) + [""]
        if not arguments.no_plots:
            name = f"{bag}.png"
            _plot(bag, results, arguments.output / name)
            report += [f"![{bag}]({name})", ""]
    (arguments.output / "report.md").write_text("\n".join(report))
    print("\n".join(line for line in report if not line.startswith("![")))


if __name__ == "__main__":
    main()
