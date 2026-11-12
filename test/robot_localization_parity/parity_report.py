#!/usr/bin/env python3
"""Write report.md and plots from robot_localization parity results."""

import argparse
import json
from pathlib import Path

import numpy


def _table(results: list[dict]) -> str:
    lines = [
        "| Bag | Filter | End error RL / rpp (tol.) [m] | Pos. diff RMS [m] | Yaw diff RMS [deg] "
        "| Rate RL / rpp [Hz] | CPU RL / rpp [s] |",
        "|---|---|---|---|---|---|---|",
    ]
    for r in results:
        rl, rpp, d = r["robot_localization"], r["rpp_localization"], r["difference"]
        cpu = r.get("cpu_seconds", {})
        stopped = cpu.get("rpp_localization", 1.0) == 0.0
        rpp_end = "stopped" if stopped else f"{rpp['final_error_m']:.3f}"
        lines.append(
            f"| {r['bag']} | {r['filter'].upper()} "
            f"| {rl['final_error_m']:.3f} / {rpp_end} ({r['upstream_tolerance_m']:.2f}) "
            f"| {d['position_rms_m']:.4f} | {d['yaw_rms_deg']:.2f} "
            f"| {rl['rate_hz']:.0f} / {rpp['rate_hz']:.0f} "
            f"| {cpu.get('robot_localization', float('nan')):.0f} / "
            f"{cpu.get('rpp_localization', float('nan')):.0f} |")
    return "\n".join(lines)


def _plot(result: dict, output: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    tracks = {name: {key: numpy.asarray(value) for key, value in track.items()}
              for name, track in result["tracks"].items()}
    a, b = tracks["robot_localization"], tracks["rpp_localization"]
    figure, (left, right) = plt.subplots(1, 2, figsize=(12, 5))
    left.plot(a["position"][:, 0], a["position"][:, 1], label="robot_localization", linewidth=2.5)
    left.plot(b["position"][:, 0], b["position"][:, 1], "--", label="rpp_localization", linewidth=1.5)
    left.plot(*result["expected_final_position"][:2], "k+", markersize=12, label="upstream end point")
    left.set_xlabel("x [m]")
    left.set_ylabel("y [m]")
    left.set_aspect("equal", adjustable="datalim")
    left.legend()
    left.set_title(f"{result['bag']} {result['filter'].upper()}: track")

    time = a["time"]
    inside = (time >= b["time"][0]) & (time <= b["time"][-1])
    time = time[inside]
    difference = numpy.linalg.norm(numpy.column_stack(
        [numpy.interp(time, b["time"], b["position"][:, i]) for i in range(3)])
        - a["position"][inside], axis=1)
    right.plot(time - time[0], difference)
    right.set_xlabel("time [s]")
    right.set_ylabel("|p_rpp - p_rl| [m]")
    right.set_title("position difference")
    figure.tight_layout()
    figure.savefig(output, dpi=80)
    plt.close(figure)


def _load(results_directory: Path) -> list[dict]:
    results = [json.loads(path.read_text()) for path in sorted(results_directory.glob("test*_*.json"))]
    if not results:
        raise SystemExit(f"no results in {results_directory}")
    return results


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", help="PARITY_RESULTS_DIR, optionally as 'Title=directory'")
    parser.add_argument("--output", type=Path, required=True, help="report directory")
    parser.add_argument("--no-plots", action="store_true")
    arguments = parser.parse_args()
    arguments.output.mkdir(parents=True, exist_ok=True)

    report = ["# rpp_localization vs robot_localization", ""]
    for index, run in enumerate(arguments.runs):
        title, _, directory = run.rpartition("=")
        results = _load(Path(directory))
        report += [f"## {title or 'Results'}", "",
                   f"robot_localization {results[0].get('robot_localization_version', '?')}.", "",
                   _table(results), ""]
        if arguments.no_plots:
            continue
        for result in results:
            name = f"run{index}_{result['bag']}_{result['filter']}.png"
            _plot(result, arguments.output / name)
            report += [f"![{result['bag']} {result['filter']}]({name})", ""]
    report += ["stopped: rpp_localization exited before the bag ended.", ""]
    (arguments.output / "report.md").write_text("\n".join(report))
    print("\n".join(line for line in report if not line.startswith("![")))


if __name__ == "__main__":
    main()
