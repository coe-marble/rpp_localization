"""Translate the existing bag cases into equivalent adapter configurations."""

import json
from pathlib import Path
import shutil

from rpp_plugin_registrator import plugin_id_from_name


def workspace(installed_share: Path, destination: Path, upstream: dict, *, replay: bool, profile=False) -> str:
    """Use temporary parameters; leave installed/source workspaces untouched."""
    # Symlink installs can retain dangling entries from profiles moved between
    # plugin folders. Required profiles are checked explicitly before launch.
    shutil.copytree(installed_share / ".rppws", destination / ".rppws",
                    ignore_dangling_symlinks=True)
    noise = upstream.get("process_noise_covariance")
    world = upstream.get("world_frame", upstream.get("odom_frame", "odom"))
    body = upstream.get("base_link_frame", "base_link")
    for description in (destination / ".rppws/parts").rglob("description.json"):
        plugin = json.loads(description.read_text()).get("PluginName", "")
        path = description.parent / "params/parameters.py"
        if not path.is_file():
            continue
        overrides = {}
        if plugin.endswith(("::Ekf15", "::Ukf15", "::ConstantAccelerationModel15")):
            if noise is not None:
                overrides["process_noise_covariance"] = noise
        if plugin.endswith(("::Ukf15", "::Ukf")):
            overrides.update({name: upstream[name] for name in ("alpha", "kappa", "beta") if name in upstream})
        if plugin == "rpp_localization::NavigationModel":
            overrides.update(world_frame=world, body_frame=body)
        if plugin.endswith(("::Ekf", "::Ukf")):
            # History lives in the persisted plugin profile; zero also disables the spare.
            overrides["history_capacity"] = 512 if replay else 0
        text = path.read_text()
        if plugin.endswith(("::Ekf", "::Ukf")):
            # The sample measurement fields declare literal map/base_link frames.
            # Match both the model and observation descriptors to the bag case.
            text += (
                "\nfor _layout in getattr(ComponentParameters, 'measurements', []):\n"
                "    for _field in _layout['fields']:\n"
                f"        _field['frameId'] = {world!r} if _field['quantity'] in (1, 4) else {body!r}\n"
                f"        _field['childFrameId'] = {body!r}\n"
            )
        for key, value in overrides.items():
            text += f"\nComponentParameters.{key} = {value!r}\n"
        path.write_text(text)
    if profile:
        _benchmark_estimators(destination / ".rppws")
    return str(destination)


def _benchmark_estimators(root: Path) -> None:
    """Select compile-time instrumented plugins in this temporary copy only."""
    replacements = {f"rpp_localization::{name}": f"rpp_localization::Profiled{name}"
                    for name in ("Ekf", "Ukf")}

    def replace(value):
        changed = False
        if isinstance(value, dict):
            for key, item in value.items():
                if key == "PluginName" and item in replacements:
                    value[key] = replacements[item]
                    changed = True
                else:
                    changed = replace(item) or changed
        elif isinstance(value, list):
            for item in value:
                changed = replace(item) or changed
        return changed

    for path in root.rglob("*.json"):
        updated = json.loads(path.read_text())
        if replace(updated):
            path.write_text(json.dumps(updated, indent=2) + "\n")
    for old, new in replacements.items():
        source = root / "parts" / plugin_id_from_name(old)
        if source.is_dir():
            source.rename(root / "parts" / plugin_id_from_name(new))


def validate_workspace(rpp_workspace: str, script_name: str, configuration: str) -> None:
    """Check the same plugin/id lookup as RPP before starting ROS processes."""
    root = Path(rpp_workspace) / ".rppws"
    script = root / "script_descriptions" / f"{script_name}.json"
    description = json.loads(script.read_text())
    for components in description["Configurations"][configuration]["Components"].values():
        for component in components:
            path = root / "parts" / plugin_id_from_name(component["PluginName"]) / component["Id"]
            record = path / "description.json"
            if not record.is_file():
                raise RuntimeError(
                    f"Installed localization profile is missing at {path}. "
                    "Rebuild rpp_localization and source install/setup.bash before retrying.")
            actual = json.loads(record.read_text())
            if actual["Id"] != component["Id"] or actual["PluginName"] != component["PluginName"]:
                raise RuntimeError(f"Localization profile identity mismatch at {path}")


def generic(upstream: dict, rpp_workspace: str, filter_type: str) -> dict:
    """Explicit translation, including source-specific masks and process noise."""
    odometry = upstream.get("odom0_config", [False] * 15)
    imu = upstream.get("imu0_config", [False] * 15)
    if len(odometry) != 15 or len(imu) != 15:
        raise ValueError("Bag sensor configurations must have fifteen axes")
    for prefix in ("odom", "imu", "pose", "twist"):
        if any(key.startswith(prefix) and key[len(prefix):].isdigit() and key != prefix + "0"
               for key in upstream):
            raise ValueError("Add an explicit mapping before benchmarking additional sensor topics")
    for key in ("use_control", "imu0_differential", "imu0_relative"):
        if upstream.get(key, False):
            raise ValueError(f"The existing bag profiles do not implement {key}")
    frequency = upstream.get("frequency", 30.0)
    world = upstream.get("world_frame", upstream.get("odom_frame", "odom"))
    result = {
        "use_sim_time": upstream.get("use_sim_time", True),
        "script_library": "rpp_localization", "script_name": "navigation_estimation",
        "configuration": filter_type + ("_2d" if upstream.get("two_d_mode", False) else ""),
        "rpp_workspace": rpp_workspace,
        "activate_on_start": True, "expose_navigation_topics": True,
        "frequency": frequency, "publish_frequency": frequency,
        "queue_capacity": 128, "initialize_from_measurement": True,
        # Match the original profiles: predict to measurement timestamps.
        "predict_to_current_time": upstream.get("predict_to_current_time", False),
        "sensor_timeout": upstream.get("sensor_timeout", 1.0 / frequency),
        "navigation.two_d_mode": upstream.get("two_d_mode", False),
        "navigation.odometry_topic": upstream.get("odom0", ""),
        "navigation.imu_topic": upstream.get("imu0", ""),
        "navigation.odometry_axes": odometry, "navigation.imu_axes": imu,
        "navigation.odometry_queue_size": upstream.get("odom0_queue_size", 10),
        "navigation.imu_queue_size": upstream.get("imu0_queue_size", 10),
        "navigation.pose_topic": "", "navigation.twist_topic": "",
        "navigation.odometry_relative": upstream.get("odom0_relative", False),
        "navigation.odometry_differential": upstream.get("odom0_differential", False),
        "navigation.imu_world_frame": world,
        "navigation.remove_gravity": upstream.get("imu0_remove_gravitational_acceleration", False),
        "navigation.publish_tf": False, "navigation.publish_acceleration": False,
        "navigation.publish_control_state": False,
    }
    for old, new in (("process_noise_covariance", "process_noise_density"),
                     ("initial_estimate_covariance", "initial_covariance"),
                     ("initial_state", "initial_state")):
        if old in upstream:
            result[new] = upstream[old]
    return result
