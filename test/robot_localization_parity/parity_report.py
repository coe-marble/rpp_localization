#!/usr/bin/env python3
"""Write accuracy, CPU, memory tables and plots from the existing bag cases."""

import argparse
import json
from pathlib import Path

import numpy
import parity_metrics


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
        "| Bag | Filter | End error RL / candidate (tol.) [m] | Pos. diff RMS [m] | Yaw diff RMS [deg] "
        "| Rate RL / candidate [Hz] | CPU RL / candidate [s] "
        "| New worker CPU [s] | New worker allocation calls | New worker peak live heap [KiB] |",
        "|---|---|---|---|---|---|---|---|---|---|",
    ]
    for result in results:
        rl = result.get("robot_localization") or {}
        candidate = result.get("rpp_localization") or {}
        difference = result.get("difference") or {}
        cpu = result.get("cpu_seconds", {})
        end = "stopped" if _stopped(result, "rpp_localization") else _format(candidate.get("final_error_m"))
        lines.append(
            f"| {result['bag']} | {result['filter'].upper()} "
            f"| {_format(rl.get('final_error_m'))} / {end} ({result['upstream_tolerance_m']:.2f}) "
            f"| {_format(difference.get('position_rms_m'), 4)} | {_format(difference.get('yaw_rms_deg'), 2)} "
            f"| {_format(rl.get('rate_hz'), 0)} / {_format(candidate.get('rate_hz'), 0)} "
            f"| {_format(cpu.get('robot_localization'), 2)} / {_format(cpu.get('rpp_localization'), 2)} "
            f"| {_worker_columns(result)} |")
    return "\n".join(lines)


def _migration_table(results: list[dict]) -> str:
    rows = [result for result in results if "legacy_rpp_localization" in result]
    if not rows:
        return ""
    lines = [
        "| Bag | Filter | End error old / new [m] | Old/new pos. RMS [m] | Old/new yaw RMS [deg] "
        "| Rate old / new [Hz] | CPU old / new [s] | CPU change | Peak RSS old / new [MiB] "
        "| New worker CPU [s] | New worker allocation calls | New worker peak live heap [KiB] |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for result in rows:
        old, new = result.get("legacy_rpp_localization") or {}, result.get("rpp_localization") or {}
        difference = result.get("legacy_difference") or {}
        cpu = result.get("cpu_seconds", {})
        processes = result.get("resources", {}).get("processes", {})
        old_end = "stopped" if _stopped(result, "legacy_rpp_localization") else _format(old.get("final_error_m"))
        new_end = "stopped" if _stopped(result, "rpp_localization") else _format(new.get("final_error_m"))
        change = result.get("generic_cpu_change_percent")
        lines.append(
            f"| {result['bag']} | {result['filter'].upper()} | {old_end} / {new_end} "
            f"| {_format(difference.get('position_rms_m'), 4)} | {_format(difference.get('yaw_rms_deg'), 2)} "
            f"| {_format(old.get('rate_hz'), 1)} / {_format(new.get('rate_hz'), 1)} "
            f"| {_format(cpu.get('legacy_rpp_localization'), 2)} / {_format(cpu.get('rpp_localization'), 2)} "
            f"| {f'{change:+.1f}%' if change is not None else '—'} "
            f"| {_format(processes.get('legacy_rpp_localization', {}).get('peak_rss_mib'), 1)} / "
            f"{_format(processes.get('rpp_localization', {}).get('peak_rss_mib'), 1)} "
            f"| {_worker_columns(result)} |")
    return "\n".join(lines)


def _timing_tables(results: list[dict]) -> str:
    rows = [result for result in results if result.get("profiling_enabled")]
    if not rows:
        return ""
    lines = [
        "### Estimator execution scopes", "",
        "CPU is execution-thread time inside the scope. Wall time includes preemption. "
        "Worker batches include live prediction/correction, history, handoff and estimate copies. "
        "In ROS replay mode, the separate replay scope includes historical replay and catch-up on the ROS executor. "
        "The estimator CPU comparison adds worker batch CPU and ROS replay CPU. Legacy replay stays inside its batch. "
        "ROS sensor conversion, publication and blocked waits are outside. "
        "The legacy cold initialization batch is excluded. Scopes are nested: do not sum batch and operation CPU.", "",
        "| Bag | Filter | Implementation | Scope | Calls | CPU [s] | CPU mean [µs] | Wall p99 upper [µs] | Wall max [µs] |",
        "|---|---|---|---|---|---|---|---|---|",
    ]
    diagnostics = []
    for result in rows:
        change = result.get("generic_estimation_cpu_change_percent")
        if change is not None:
            diagnostics.append(f"{result['bag']} {result['filter'].upper()}: aggregate estimation batch "
                               f"CPU change {change:+.1f}% (worker batch plus ROS replay; positive means the new stack used more).")
        for name, timing in result.get("estimation_timing", {}).items():
            label = ("old" if name == "legacy_rpp_localization" or result.get("implementation") == "legacy"
                     else "new")
            prefix = f"{result['bag']} {result['filter'].upper()} {label}"
            if "error" in timing:
                diagnostics.append(f"{prefix}: {timing['error']}.")
                continue
            for scope, values in timing.get("operations", {}).items():
                cpu, wall = values["cpu"], values["wall"]
                if scope == "replay" and not wall.get("count"):
                    continue
                cpu_valid = cpu["count"] == wall["count"] and cpu["count"] > 0
                lines.append(
                    f"| {result['bag']} | {result['filter'].upper()} | {label} | {scope} "
                    f"| {wall['count']} | {_format(cpu['total_ns'] / 1e9 if cpu_valid else None, 4)} "
                    f"| {_format(cpu['mean_ns'] / 1e3 if cpu_valid else None, 2)} "
                    f"| {_format(wall['p99_upper_ns'] / 1e3, 2)} | {_format(wall['max_ns'] / 1e3, 2)} |")
            diagnostics.append(
                f"{prefix}: {timing['batch_budget_overruns']} batches exceeded the "
                f"{timing['batch_budget_ns'] / 1e6:.3f} ms execution budget.")
            queue = timing.get("queue_wait_wall", {})
            worker = result.get("resources", {}).get("processes", {}).get(name, {}).get("worker_cpu_seconds")
            if worker is not None:
                diagnostics.append(f"{prefix}: numerical worker used {worker:.3f} s CPU "
                                   "in the playback/drain window (Linux thread ticks).")
            if queue.get("count"):
                diagnostics.append(f"{prefix}: queue wait p99 upper {queue['p99_upper_ns'] / 1e3:.2f} µs, "
                                   f"maximum {queue['max_ns'] / 1e3:.2f} µs ({queue['count']} events).")
    lines += ["", *diagnostics, "",
              "Quantiles are fixed-histogram upper bounds (at most 6.25% above 16 ns); maxima and totals are exact clock "
              "measurements. Recording uses prepared counters, with serialization only in the stats service. "
              "The clocks and nested scopes add measurement overhead to both implementations.", "",
              "Compare worker batch plus ROS replay CPU for the estimator workload and inspect operation counts: "
              "generic worker mode replays inside correction, generic ROS mode uses its separate replay scope, "
              "and legacy replay calls predict/correct again. "
              "Per-call averages and batch distributions therefore include different work mixes and scheduling. "
              "Raw new-worker CPU is separate from scoped CPU and cannot be compared directly with an entire old ROS executor. "
              "Budget overruns are execution durations, not scheduler deadline misses. "
              "Use bag rate 1.0 for real-time load, and repeat isolated runs to assess CPU contention.", ""]
    return "\n".join(lines)


def _execution_table(results: list[dict]) -> str:
    rows = [(result, timing["execution_counts"])
            for result in results for timing in result.get("estimation_timing", {}).values()
            if "execution_counts" in timing]
    if not rows:
        return ""
    lines = ["### Generic internal correction and replay workload", "",
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
    jobs = [(result, counts) for result, counts in rows if "replay_jobs_started" in counts]
    if jobs:
        lines += ["", "| Bag | Filter | Replay jobs started | Role switches | Jobs cancelled | Stale replay samples | Replay pending |",
                  "|---|---|---|---|---|---|---|"]
        for result, counts in jobs:
            lines.append(f"| {result['bag']} | {result['filter'].upper()} "
                         f"| {counts['replay_jobs_started']:,} | {counts['replay_role_switches']:,} "
                         f"| {counts['replay_jobs_cancelled']:,} "
                         f"| {_format(counts.get('replay_stale_corrections'), 0)} | {counts['replay_pending']} |")
    lines += ["", "Core counts aggregate both physical estimator instances in ROS replay mode, including live work, "
              "catch-up and attempted calls inside historical replay, including gated or failed work. "
              "Replayed predictions include split intervals; replayed corrections exclude the newly arrived sample. "
              "Tail length counts retained events after the insertion point; rollback span reaches the restored checkpoint. "
              "Failed replay work remains counted after the posterior is restored. "
              "Covariance counters cover boundary validation, including initialization; they exclude the required innovation "
              "LDLT and UKF sigma-point factorization. Legacy interface call counts remain in the scope table. "
              "Older results without these counters cannot reconstruct internal replay work.", ""]
    return "\n".join(lines)


def _replay_table(results: list[dict]) -> str:
    rows = [(result, name, timing) for result in results
            for name, timing in result.get("estimation_timing", {}).items()
            if name in ("rpp_localization", "legacy_rpp_localization") and "error" not in timing]
    if not rows:
        return ""
    lines = ["### Replay history statistics", "",
             "| Bag | Filter | Implementation | Replay jobs / legacy batches | Internal rewinds | Mean replay states | Std. dev. | Max replay states |",
             "|---|---|---|---|---|---|---|---|"]
    for result, name, timing in rows:
        label = "old" if name == "legacy_rpp_localization" or result.get("implementation") == "legacy" else "new"
        counts = timing.get("execution_counts", {})
        statistics = timing.get("replay_statistics")
        if statistics is None:
            statistics = counts.get("replay_states")
        if statistics is None and "replay_attempts" in counts:
            attempts = counts["replay_attempts"]
            statistics = {"count": attempts, "states_max": counts.get("tail_events_max"),
                          "states_mean": counts.get("tail_events_total", 0) / attempts if attempts else 0.0}
        statistics = statistics or {}
        count = statistics.get("count")
        jobs = count if label == "old" else counts.get("replay_jobs_started")
        if label == "new" and result.get("effective_parameters", {}).get(name, {}).get("replay_execution") == "worker":
            jobs = None
        rewinds = counts.get("replay_attempts", count) if label == "new" else count
        lines.append(f"| {result['bag']} | {result['filter'].upper()} | {label} "
                     f"| {f'{jobs:,}' if jobs is not None else '—'} "
                     f"| {f'{rewinds:,}' if rewinds is not None else '—'} "
                     f"| {_format(statistics.get('states_mean'), 2)} "
                     f"| {_format(statistics.get('states_stddev'), 2)} "
                     f"| {_format(statistics.get('states_max'), 0)} |")
    lines += ["", "Replay jobs count generic background catch-up sessions; legacy batches count rollback calls. "
              "Internal rewinds count generic historical replay attempts inside those jobs. A bounded batch of queued "
              "corrections shares one rewind; later arrivals or another service batch can require more. "
              "Legacy performs one rollback attempt per replay batch, so its two counts match. "
              "Generic worker-mode jobs are not recorded and appear as —. "
              "One history sample is the saved-history tail selected for an internal rewind, including failed attempts. "
              "The mean and population standard deviation use all recorded attempts; zero-length tails count too. "
              "Generic checkpoints follow prediction/correction events; legacy checkpoints follow measurement "
              "timestamp batches. These are history-record counts, not equal units of numerical work. "
              "Old recordings show available count/mean/max data and — for missing statistics; "
              "standard deviation cannot be recovered from totals and maxima.", ""]
    return "\n".join(lines)


def _plot(result: dict, output: Path) -> bool:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    tracks = {name: {key: numpy.asarray(value) for key, value in track.items()}
              for name, track in result["tracks"].items() if len(track["time"]) > 1}
    if "robot_localization" not in tracks or "rpp_localization" not in tracks:
        return False
    reference = tracks["robot_localization"]
    figure, (left, right) = plt.subplots(1, 2, figsize=(12, 5))
    for name, track in tracks.items():
        label = result.get("implementation", "legacy") if name == "rpp_localization" else name
        left.plot(track["position"][:, 0], track["position"][:, 1], label=label, linewidth=1.5)
    left.plot(*result["expected_final_position"][:2], "k+", markersize=12, label="upstream end point")
    left.set_xlabel("x [m]"); left.set_ylabel("y [m]")
    left.set_aspect("equal", adjustable="datalim"); left.legend()
    left.set_title(f"{result['bag']} {result['filter'].upper()}: track")
    for name, track in tracks.items():
        if name == "robot_localization":
            continue
        inside = (reference["time"] >= track["time"][0]) & (reference["time"] <= track["time"][-1])
        time = reference["time"][inside]
        if time.size == 0:
            continue
        difference = numpy.linalg.norm(numpy.column_stack(
            [numpy.interp(time, track["time"], track["position"][:, i]) for i in range(3)])
            - reference["position"][inside], axis=1)
        label = result.get("implementation", "legacy") if name == "rpp_localization" else name
        right.plot(time - reference["time"][0], difference, label=label)
    right.set_xlabel("time [s]"); right.set_ylabel("position difference from RL [m]")
    right.legend(); figure.tight_layout()
    figure.savefig(output, dpi=80); plt.close(figure)
    return True


def _load(directory: Path) -> list[dict]:
    results = [json.loads(path.read_text()) for path in sorted(directory.glob("test*_*.json"))]
    if not results:
        raise SystemExit(f"no results in {directory}")
    # Reprocess saved tracks from earlier runs; no filter or bag execution is
    # needed to correct the extra pair's comparison clock in the report.
    for result in results:
        tracks = result.get("tracks", {})
        names = ("robot_localization", "legacy_rpp_localization", "rpp_localization")
        if result.get("comparison_timebase") == "robot_localization" or not all(
                name in tracks and tracks[name]["time"] for name in names):
            continue
        reference, legacy, candidate = (parity_metrics.Track(**tracks[name]) for name in names)
        start = max(min(legacy.time), min(candidate.time))
        end = min(max(legacy.time), max(candidate.time))
        if not any(start <= stamp <= end for stamp in reference.time):
            continue
        result["legacy_timeline_difference"] = result.get("legacy_difference")
        result["legacy_difference"] = parity_metrics.compare(legacy, candidate, timebase=reference)
        result["comparison_timebase"] = "robot_localization"
    return results


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", help="PARITY_RESULTS_DIR, optionally as 'Title=directory'")
    parser.add_argument("--output", type=Path, required=True, help="report directory")
    parser.add_argument("--no-plots", action="store_true")
    arguments = parser.parse_args(); arguments.output.mkdir(parents=True, exist_ok=True)
    report = ["# Localization bag comparison", ""]
    for index, run in enumerate(arguments.runs):
        title, _, directory = run.rpartition("=")
        results = _load(Path(directory))
        replay_setting = (f"replay (all implementations): {results[0]['replay_enabled']}"
                          if "replay_enabled" in results[0]
                          else f"reference replay: {results[0].get('robot_localization_replay', '?')}")
        report += [f"## {title or 'Results'}", "",
                   f"robot_localization {results[0].get('robot_localization_version', '?')}; "
                   f"candidate: {results[0].get('implementation', 'legacy')}; "
                   f"bag rate: {results[0].get('bag_rate', '?')}; "
                   f"{replay_setting}.", "",
                   _table(results), ""]
        migration = _migration_table(results)
        if migration:
            report += ["### Previous implementation versus generic stack", "", migration, "",
                       "Positive CPU change means the generic stack used more process CPU time. "
                       "CPU includes all worker threads during playback and the drain window; "
                       "RSS is the process lifetime peak, including startup.", ""]
            if any(result.get("comparison_timebase") == "robot_localization" for result in results):
                report += ["Direct old/new errors use robot_localization timestamps within the shared overlap, "
                           "matching the original accuracy comparisons. Saved test outcomes are unchanged.", ""]
                for result in results:
                    diagnostic = result.get("legacy_timeline_difference")
                    if diagnostic:
                        report += [f"{result['bag']} {result['filter'].upper()}: on the legacy timestamp grid "
                                   f"(including any earlier startup samples), yaw RMS was "
                                   f"{diagnostic['yaw_rms_deg']:.4f}° and maximum difference was "
                                   f"{diagnostic['yaw_max_deg']:.4f}°.", ""]
        timing = _timing_tables(results)
        if timing:
            report += [timing, ""]
        execution = _execution_table(results)
        if execution:
            report += [execution, ""]
        replay = _replay_table(results)
        if replay:
            report += [replay, ""]
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
    report += ["Process CPU and RSS describe the complete ROS processes. The new worker columns cover only "
               "rpp_estimator, from worker entry through the paused stats snapshot, including live prediction, correction "
               "and role adoption. ROS replay and catch-up are outside that worker; inline worker-mode replay is included. "
               "Process CPU covers playback/drain instead. Allocation calls include malloc/calloc/realloc "
               "and aligned allocation requests, including failed requests. Peak live heap counts requested bytes "
               "allocated by that worker, excluding buffers prepared on other threads, shared memory and stack. "
               "Worker metrics require PARITY_PROFILE=1; allocator bookkeeping adds measurement overhead. "
               "Missing worker measurements in unprofiled or older runs are shown as —. "
               "Side-by-side runs share CPU and DDS resources; repeat isolated legacy/generic runs for timing confirmation.", ""]
    report_path = (arguments.output / "report.md").resolve()
    report_path.write_text("\n".join(report))
    print(report_path)


if __name__ == "__main__":
    main()
