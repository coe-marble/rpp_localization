#!/usr/bin/env python3
"""Report rpp_localization parity, process usage, and estimator metrics."""

import argparse
import json
from pathlib import Path

import numpy


def _format(value, digits=3):
    return f"{value:.{digits}f}" if value is not None else "—"


def _stopped(result, name):
    resource = result.get("resources", {}).get("processes", {}).get(name)
    return resource is not None and not resource["running_at_end"]


def _worker_columns(result: dict) -> str:
    worker = result.get("worker_metrics", {})
    if "error" in worker:
        return "— | — | —"
    allocations = worker.get("allocation_calls")
    peak_bytes = worker.get("peak_live_heap_bytes") if worker.get("memory_complete") else None
    return (f"{_format(worker.get('cpu_seconds'), 3)} | "
            f"{f'{allocations:,}' if allocations is not None else '—'} | "
            f"{_format(peak_bytes / 1024 if peak_bytes is not None else None, 2)}")


def _table(results: list[dict]) -> str:
    lines = [
        "| Bag | Filter | End error RL / RPP (tol.) [m] | Pos. diff RMS [m] | Yaw diff RMS [deg] "
        "| Rate RL / RPP [Hz] | Process CPU RL / RPP [s] | Process peak RSS RL / RPP [MiB] "
        "| Worker CPU [s] | Worker allocation calls | Worker peak live heap [KiB] |",
        "|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for result in results:
        rl = result.get("robot_localization") or {}
        rpp = result.get("rpp_localization") or {}
        difference = result.get("difference") or {}
        cpu = result.get("cpu_seconds", {})
        processes = result["resources"]["processes"]
        reference_memory = processes.get("robot_localization", {}).get("peak_rss_mib")
        rpp_memory = processes.get("rpp_localization", {}).get("peak_rss_mib")
        end = "stopped" if _stopped(result, "rpp_localization") else _format(rpp.get("final_error_m"))
        lines.append(
            f"| {result['bag']} | {result['filter'].upper()} "
            f"| {_format(rl.get('final_error_m'))} / {end} ({result['upstream_tolerance_m']:.2f}) "
            f"| {_format(difference.get('position_rms_m'), 4)} | {_format(difference.get('yaw_rms_deg'), 2)} "
            f"| {_format(rl.get('rate_hz'), 0)} / {_format(rpp.get('rate_hz'), 0)} "
            f"| {_format(cpu.get('robot_localization'), 2)} / {_format(cpu.get('rpp_localization'), 2)} "
            f"| {_format(reference_memory, 1)} / {_format(rpp_memory, 1)} "
            f"| {_worker_columns(result)} |")
    return "\n".join(lines)


def _timing_tables(results: list[dict]) -> str:
    rows = [result for result in results if result.get("profiling_enabled")]
    if not rows:
        return ""
    lines = [
        "### Estimator execution scopes", "",
        "CPU measures the execution thread inside each scope. Worker estimation includes live "
        "prediction/correction, history, adoption and estimate copies. ROS replay has its own scope. "
        "Add estimation and replay CPU for total estimator work; predict/correct scopes are nested "
        "inside estimation and must not be added again.", "",
        "| Bag | Filter | Scope | Calls | CPU [s] | CPU mean [µs] | Wall p99 upper [µs] | Wall max [µs] |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for result in rows:
        timing = result.get("estimation_timing", {}).get("rpp_localization", {})
        if "error" in timing:
            lines += [f"{result['bag']} {result['filter'].upper()}: {timing['error']}.", ""]
            continue
        for scope, values in timing.get("operations", {}).items():
            cpu, wall = values["cpu"], values["wall"]
            if scope == "replay" and not wall.get("count"):
                continue
            cpu_valid = cpu["count"] == wall["count"] and cpu["count"] > 0
            lines.append(
                f"| {result['bag']} | {result['filter'].upper()} | {scope} | {wall['count']} "
                f"| {_format(cpu['total_ns'] / 1e9 if cpu_valid else None, 4)} "
                f"| {_format(cpu['mean_ns'] / 1e3 if cpu_valid else None, 2)} "
                f"| {_format(wall['p99_upper_ns'] / 1e3 if wall['count'] else None, 2)} "
                f"| {_format(wall['max_ns'] / 1e3 if wall['count'] else None, 2)} |")
    return "\n".join(lines)


def _execution_table(results: list[dict]) -> str:
    rows = []
    for result in results:
        timing = result.get("estimation_timing", {}).get("rpp_localization", {})
        if "execution_counts" in timing:
            rows.append((result, timing["execution_counts"]))
    if not rows:
        return ""
    lines = ["### Internal correction and replay workload", "",
             "| Bag | Filter | Core predict / correct | Projected corrections | Late samples | Replay success / failure "
             "| Replayed predict / correct | Tail events mean / max | Rollback span mean / max [ms] "
             "| Covariance diagonal / cache / LDLT |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for result, counts in rows:
        attempts = counts["replay_attempts"]
        tail_mean = counts["tail_events_total"] / attempts if attempts else 0.0
        span_mean = counts["rollback_span_ns_total"] / attempts / 1e6 if attempts else 0.0
        lines.append(
            f"| {result['bag']} | {result['filter'].upper()} "
            f"| {counts['core_predictions']:,} / {counts['core_corrections']:,} "
            f"| {counts['projected_corrections']:,} | {counts['late_measurements']:,} "
            f"| {counts['replay_successes']:,} / {counts['replay_failures']:,} "
            f"| {counts['replayed_predictions']:,} / {counts['replayed_corrections']:,} "
            f"| {tail_mean:.2f} / {counts['tail_events_max']} "
            f"| {span_mean:.3f} / {counts['rollback_span_ns_max'] / 1e6:.3f} "
            f"| {counts['covariance_diagonal_checks']:,} / {counts['covariance_cache_hits']:,} / "
            f"{counts['covariance_factorizations']:,} |")
    lines += ["", "| Bag | Filter | Replay jobs started | Role switches | Jobs cancelled | Stale replay samples | Replay pending |",
              "|---|---|---|---|---|---|---|"]
    for result, counts in rows:
        lines.append(f"| {result['bag']} | {result['filter'].upper()} "
                     f"| {counts['replay_jobs_started']:,} | {counts['replay_role_switches']:,} "
                     f"| {counts['replay_jobs_cancelled']:,} "
                     f"| {counts['replay_stale_corrections']:,} | {counts['replay_pending']} |")
    lines += ["", "Core counts aggregate both physical estimator instances in ROS replay mode, including live work, "
              "catch-up and attempted calls inside historical replay, including gated or failed work. "
              "Replayed predictions include split intervals; replayed corrections exclude the newly arrived sample. "
              "Tail length counts retained events after the insertion point; rollback span reaches the restored checkpoint. "
              "Failed replay work remains counted after the posterior is restored. "
              "Covariance counters cover boundary validation, including initialization; they exclude the required innovation "
              "LDLT and UKF sigma-point factorization.", ""]
    return "\n".join(lines)


def _replay_table(results: list[dict]) -> str:
    rows = [(result, result.get("estimation_timing", {}).get("rpp_localization", {}))
            for result in results if result.get("profiling_enabled")]
    if not rows:
        return ""
    lines = [
        "### Replay history statistics", "",
        "| Bag | Filter | Replay jobs | Internal rewinds | Mean replay states | Std. dev. | Max replay states |",
        "|---|---|---|---|---|---|---|",
    ]
    for result, timing in rows:
        counts = timing.get("execution_counts", {})
        statistics = timing.get("replay_statistics", {})
        jobs = counts.get("replay_jobs_started")
        if result.get("effective_parameters", {}).get("rpp_localization", {}).get("replay_execution") == "worker":
            jobs = None
        rewinds = counts.get("replay_attempts")
        lines.append(
            f"| {result['bag']} | {result['filter'].upper()} "
            f"| {f'{jobs:,}' if jobs is not None else '—'} "
            f"| {f'{rewinds:,}' if rewinds is not None else '—'} "
            f"| {_format(statistics.get('states_mean'), 2)} "
            f"| {_format(statistics.get('states_stddev'), 2)} "
            f"| {_format(statistics.get('states_max'), 0)} |")
    lines += ["",
              "Jobs count background repair sessions. Internal rewinds count historical replay "
              "attempts within those jobs. A service batch shares one rewind; later batches may "
              "rewind again. Worker-mode jobs appear as — because replay is inline. "
              "Replay states count retained prediction/correction records after the rollback point. "
              "The mean and population standard deviation include failed attempts and empty tails.", ""]
    return "\n".join(lines)


def _plot(result: dict, output: Path) -> bool:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    recorded = result["tracks"]
    if any(len(recorded[name]["time"]) < 2 for name in ("robot_localization", "rpp_localization")):
        return False
    tracks = {name: {key: numpy.asarray(value) for key, value in recorded[name].items()}
              for name in ("robot_localization", "rpp_localization")}
    reference = tracks["robot_localization"]
    rpp = tracks["rpp_localization"]
    figure, (left, right) = plt.subplots(1, 2, figsize=(12, 5))
    for name, track in tracks.items():
        label = "RPP" if name == "rpp_localization" else name
        left.plot(track["position"][:, 0], track["position"][:, 1], label=label, linewidth=1.5)
    left.plot(*result["expected_final_position"][:2], "k+", markersize=12, label="upstream end point")
    left.set_xlabel("x [m]")
    left.set_ylabel("y [m]")
    left.set_aspect("equal", adjustable="datalim")
    left.legend()
    left.set_title(f"{result['bag']} {result['filter'].upper()}: track")
    inside = (reference["time"] >= rpp["time"][0]) & (reference["time"] <= rpp["time"][-1])
    time = reference["time"][inside]
    if time.size:
        difference = numpy.linalg.norm(numpy.column_stack(
            [numpy.interp(time, rpp["time"], rpp["position"][:, i]) for i in range(3)])
            - reference["position"][inside], axis=1)
        right.plot(time - reference["time"][0], difference, label="RPP")
        right.legend()
    right.set_xlabel("time [s]")
    right.set_ylabel("position difference from RL [m]")
    figure.tight_layout()
    figure.savefig(output, dpi=80)
    plt.close(figure)
    return True


def _load(directory: Path) -> list[dict]:
    results = [json.loads(path.read_text()) for path in sorted(directory.glob("test*_*.json"))]
    if not results:
        raise SystemExit(f"No results in {directory}")
    return results


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", help="PARITY_RESULTS_DIR, optionally as 'Title=directory'")
    parser.add_argument("--output", type=Path, required=True, help="report directory")
    parser.add_argument("--no-plots", action="store_true")
    arguments = parser.parse_args()
    arguments.output.mkdir(parents=True, exist_ok=True)
    report = ["# Localization bag comparison", ""]
    for index, run in enumerate(arguments.runs):
        title, _, directory = run.rpartition("=")
        results = _load(Path(directory))
        first = results[0]
        report += [
            f"## {title or 'Results'}", "",
            f"robot_localization {first['robot_localization_version']} / rpp_localization; "
            f"bag rate: {first['bag_rate']}; replay: {first['replay_enabled']}; "
            f"RPP replay execution: {first['effective_parameters']['rpp_localization']['replay_execution']}.",
            "", _table(results), "",
        ]
        for render in (_timing_tables, _execution_table, _replay_table):
            section = render(results)
            if section:
                report += [section, ""]
        for result in results:
            worker = result.get("worker_metrics", {})
            if "error" in worker:
                report += [f"{result['bag']} {result['filter'].upper()} worker metrics: {worker['error']}.", ""]
            elif worker and not worker.get("memory_complete"):
                report += [f"{result['bag']} {result['filter'].upper()}: worker heap tracking capacity "
                           "was exceeded; peak heap usage is unavailable.", ""]
        if not arguments.no_plots:
            for result in results:
                name = f"run{index}_{result['bag']}_{result['filter']}.png"
                if _plot(result, arguments.output / name):
                    report += [f"![{result['bag']} {result['filter']}]({name})", ""]
    report += [
        "Process CPU and RSS cover each complete ROS process. Worker columns cover only "
        "rpp_estimator, from worker entry through the paused stats snapshot. ROS replay and "
        "catch-up run outside that worker; inline worker-mode replay is included. "
        "Peak live heap counts requested bytes allocated by the worker, excluding buffers "
        "prepared elsewhere and stack storage. Profiling adds measurement overhead. "
        "Repeat runs when comparing CPU or timing.", "",
    ]
    report_path = (arguments.output / "report.md").resolve()
    report_path.write_text("\n".join(report))
    print(report_path)


if __name__ == "__main__":
    main()
