#!/usr/bin/env python3
"""Write report.md and plots from filter benchmark results, one results directory per setup."""

import argparse
from itertools import combinations
import json
from pathlib import Path

import numpy

import benchmark_metrics

_ORDER = ("ekf", "ukf", "inekf")
_TITLES = {
    "upstream": "Upstream sensors",
    "common": "Common sensors",
}
_DESCRIPTIONS = {
    "upstream": "EKF and UKF with the upstream bag configs, the InEKF with the IMU and the odometry "
                "velocity (and height on test3).",
    "common": "All three fuse the same sensors: IMU orientation, rates and acceleration, odometry "
              "velocity (and height on test3).",
}


def _load(directory: Path) -> dict[str, dict[str, dict]]:
    by_bag: dict[str, dict[str, dict]] = {}
    for path in sorted(directory.glob("test*_*.json")):
        result = json.loads(path.read_text())
        by_bag.setdefault(result["bag"], {})[result["filter"]] = result
    if not by_bag:
        raise SystemExit(f"no results in {directory}")
    return by_bag


def _names(results: dict) -> list[str]:
    return [name for name in _ORDER if name in results]


def _accuracy_table(results: dict[str, dict]) -> list[str]:
    gnss = any("fixes" in r for r in results.values())
    header = "| Filter | End error x / y / z [m] | End error [m] |"
    if gnss:
        header += " GNSS drift at 100 / 200 / 300 s / end [m] | GNSS drift RMS [m] |"
    header += " Rate [Hz] | CPU [s] |"
    lines = [header, "|" + "---|" * (header.count("|") - 1)]
    for name in _names(results):
        r = results[name]
        end = benchmark_metrics.end_error_axes(r["track"], r["expected_final_position"])
        row = (f"| {name.upper()} | {end['x']:+.3f} / {end['y']:+.3f} / {end['z']:+.3f} "
               f"| {end['norm']:.3f} |")
        if gnss:
            drift = benchmark_metrics.gnss_drift(r["track"], r["fixes"])
            row += (f" {drift['at_100s_m']:.1f} / {drift['at_200s_m']:.1f} / "
                    f"{drift['at_300s_m']:.1f} / {drift['end_m']:.1f} | {drift['rms_after_m']:.1f} |")
        row += f" {r['summary']['rate_hz']:.0f} | {r['cpu_seconds']:.1f} |"
        lines.append(row)
    return lines


def _difference_table(results: dict[str, dict]) -> list[str]:
    lines = ["| Pair | Diff RMS x / y / z [m] | Yaw diff RMS [deg] |", "|---|---|---|"]
    for a, b in combinations(_names(results), 2):
        d = benchmark_metrics.difference_axes(results[a]["track"], results[b]["track"])
        lines.append(f"| {a.upper()} / {b.upper()} | {d['x']:.3f} / {d['y']:.3f} / {d['z']:.3f} "
                     f"| {d['yaw_deg']:.2f} |")
    return lines


def _plot_tracks(bag: str, results: dict[str, dict], output: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, (left, right) = plt.subplots(1, 2, figsize=(12, 5))
    expected = next(iter(results.values()))["expected_final_position"]
    for name in _names(results):
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
    for setup in _TITLES:
        parser.add_argument(f"--{setup}", type=Path, help=f"results of BENCHMARK_SETUP={setup}")
    parser.add_argument("--output", type=Path, required=True, help="report directory")
    parser.add_argument("--no-plots", action="store_true")
    arguments = parser.parse_args()
    arguments.output.mkdir(parents=True, exist_ok=True)

    loaded = {setup: _load(getattr(arguments, setup))
              for setup in _TITLES if getattr(arguments, setup)}

    report = ["# EKF vs UKF vs InEKF", ""]
    for setup, by_bag in loaded.items():
        report += [f"## {_TITLES[setup]}", "", _DESCRIPTIONS[setup], ""]
        for bag, results in sorted(by_bag.items()):
            tolerance = next(iter(results.values()))["upstream_tolerance_m"]
            report += [f"### {bag}", ""]
            report += [f"Upstream end point tolerance: {tolerance:.2f} m.", ""]
            report += _accuracy_table(results) + [""] + _difference_table(results) + [""]
            if not arguments.no_plots:
                name = f"{setup}_{bag}.png"
                _plot_tracks(bag, results, arguments.output / name)
                report += [f"![{bag}]({name})", ""]
    (arguments.output / "report.md").write_text("\n".join(report))
    print("\n".join(line for line in report if not line.startswith("![")))


if __name__ == "__main__":
    main()
