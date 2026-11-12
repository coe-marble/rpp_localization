rpp_localization
==================

rpp_localization is a package of nonlinear state estimation nodes. The package was developed by Charles River Analytics, Inc.

Please see documentation here: http://wiki.ros.org/rpp_localization

The ROS parameter `controller.activate_on_start` defaults to `true`, matching
`rpp_control`. Set it to `false` to configure the RPP filter graph without
activating it. Measurement processing and publication wait until the filter
component is activated through RPP. This parameter is read during startup.

Set `rpp_workspace` to a workspace directory (or its `.rppws` directory) to
select component parameters from that workspace. By default, the adapter uses
the installed script library's workspace. Filter and model settings belong to
those components' RPP parameters. The ROS adapter selects the script and
configuration without referring to concrete filter or model plugin names.
ROS YAML settings for process noise, UKF constants, and model control limits
are not automatically forwarded to component parameters. Configure those in
the selected RPP workspace; ROS `use_control` and `control_config` govern the
adapter's command handling, while the model retains its own control settings.

Filter plugins retain the most recent 512 prediction/correction events and
their checkpoints. A stale correction replays only the affected suffix,
preserving UKF internal state. Older measurements return `staleMeasurement`;
the ROS adapter drops them with a warning. The ROS `smooth_lagged_data` history
remains controlled by `history_length`. Update-rate warnings show both elapsed
ROS time and wall time, which differ during accelerated bag playback.

The default library and localization tests use the built-in constant-acceleration
model and do not depend on `more_common` or `more_dynamics`. The optional
`VehicleModel3DNavModel15` bridge remains available in
`plugins/models/vehicle_model3d_nav_model15.hpp`. After installing and registering
its MORE dependencies, opt in with
`rpp library rpp_localization register plugins/models/vehicle_model3d_nav_model15.hpp`
from this repository. It is not registered by the default `plugins.json`.

CI uses the test image published by `rpp_dev`. It registers the existing library
with `rpp library register`, then builds localization once with its plugin tests
enabled. Rosbag integration tests remain opt-in through
`RPP_LOCALIZATION_ENABLE_ROSBAG_INTEGRATION_TESTS`.

The numerical core (EKF, UKF, navigation models, and filter utilities) is
header-only. CMake consumers can link `rpp_localization::numerics` to use it
without linking the ROS adapter library. The ROS adapters and nodes remain
compiled, and the existing `rpp_localization_core` shared target is retained.
Plugin registration compiles the numerical implementation from the headers,
so it does not need a previously built localization library.

Rebuild downstream C++ binaries and re-register existing localization plugins
after this change; numerical implementation symbols are no longer supplied
by the shared library.
