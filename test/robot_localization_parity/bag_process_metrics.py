"""Linux process metrics, identified by launch PIDs rather than command names."""

import os
from pathlib import Path
import threading
import time


def _worker(pid: int, tid: int | None = None) -> dict | None:
    """The named numerical worker, without executor/DDS/TF thread CPU."""
    try:
        directory = Path("/proc") / str(pid) / "task"
        for task in [directory / str(tid)] if tid is not None else directory.iterdir():
            try:
                stat = (task / "stat").read_text()
                if stat[stat.index("(") + 1:stat.rindex(")")] != "rpp_estimator":
                    continue
                fields = stat[stat.rindex(")") + 2:].split()
                if fields[0] != "Z":
                    return {"tid": int(task.name), "start_ticks": int(fields[19]),
                            "cpu_seconds": (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")}
            except (OSError, ValueError, IndexError):
                continue
    except (OSError, ValueError, IndexError):
        pass
    return None


def _read(pid: int, profile=False, worker_tid: int | None = None) -> dict | None:
    try:
        process = Path("/proc") / str(pid)
        stat = (process / "stat").read_text()
        fields = stat[stat.rindex(")") + 2:].split()
        if fields[0] == "Z":
            return None
        status = (process / "status").read_text().splitlines()
        peak_kib = next(int(line.split()[1]) for line in status if line.startswith("VmHWM:"))
        return {"cpu_seconds": (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK"),
                "peak_rss_mib": peak_kib / 1024.0, "start_ticks": int(fields[19]),
                "worker": _worker(pid, worker_tid) if profile or worker_tid is not None else None}
    except (OSError, ValueError, IndexError, StopIteration):
        return None


class Processes:
    """The playback window excludes process startup; CPU includes all threads."""

    def __init__(self, profile=False):
        self._profile = profile
        self._lock = threading.Lock()
        self._records = {}
        self._exits = {}
        self._started = None
        self._sampled = 0.0

    def register(self, name: str, pid: int) -> None:
        with self._lock:
            self._records[name] = {"pid": pid, "baseline": None, "latest": None,
                                   "running_at_end": True,
                                   "started_after_playback": self._started is not None}

    def begin(self) -> None:
        with self._lock:
            self._started = time.monotonic()
            for name, record in self._records.items():
                record["baseline"] = _read(record["pid"], self._profile and name == "rpp_localization")

    def exited(self, name: str, returncode: int) -> None:
        with self._lock:
            self._exits[name] = returncode

    def exits(self) -> dict:
        with self._lock:
            return dict(self._exits)

    def sample(self, force=False) -> None:
        with self._lock:
            now = time.monotonic()
            if self._started is None or (not force and now - self._sampled < 0.1):
                return
            self._sampled = now
            for record in self._records.values():
                baseline = record["baseline"]
                worker = baseline.get("worker") if baseline else None
                # Discover once, then sample just that TID to limit observer CPU.
                current = _read(record["pid"], worker_tid=worker["tid"] if worker else None)
                record["running_at_end"] = current is not None and baseline is not None and (
                    current["start_ticks"] == baseline["start_ticks"])
                if record["running_at_end"]:
                    record["latest"] = current

    def finish(self) -> dict:
        self.sample(force=True)
        with self._lock:
            processes = {}
            for name, record in self._records.items():
                baseline, latest = record["baseline"], record["latest"]
                worker_start = baseline.get("worker") if baseline else None
                worker_end = latest.get("worker") if latest else None
                same_worker = (record["running_at_end"] and worker_start and worker_end
                               and worker_start["tid"] == worker_end["tid"]
                               and worker_start["start_ticks"] == worker_end["start_ticks"])
                processes[name] = {
                    "pid": record["pid"], "running_at_end": record["running_at_end"],
                    "started_after_playback": record["started_after_playback"],
                    "exit_code": self._exits.get(name),
                    "cpu_seconds": max(0.0, latest["cpu_seconds"] - baseline["cpu_seconds"])
                    if latest and baseline else None,
                    # Linux VmHWM is the lifetime peak, including configuration.
                    "peak_rss_mib": latest["peak_rss_mib"] if latest else None,
                    "worker_cpu_seconds": max(0.0, worker_end["cpu_seconds"] - worker_start["cpu_seconds"])
                    if same_worker else None,
                    "worker_thread_id": worker_end["tid"] if same_worker else None,
                }
            return {"window_wall_seconds": time.monotonic() - self._started if self._started else None,
                    "cpu_window": "bag playback through output drain; all process threads",
                    "memory_window": "process lifetime peak RSS, including startup",
                    "worker_cpu_window": "same playback/drain window; Linux /proc thread ticks; generic only",
                    "processes": processes}
