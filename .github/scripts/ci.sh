#!/usr/bin/env bash
# Run from the GitHub Actions workspace; dependencies are checked out by ci.yml.
set -eo pipefail
source /opt/ros/lyrical/setup.bash

# Use the Python tooling pinned by rpp_dev; system prerequisites are in its image.
for package in rpp_common rpp_plugin_registrator rpp_orchestrator rpp_py rpp_cli; do
  python -m pip install --no-deps -e "rpp_dev/src/$package"
done

colcon build --base-paths rpp_dev/src/rpp_cpp \
  --symlink-install --cmake-args -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/opt/rpp-test-venv/bin/python
source install/setup.bash
rpp init-home

rpp library register "$PWD/libraries/rpp_localization" --link

colcon build --base-paths libraries/rpp_localization --symlink-install --cmake-args \
  -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/opt/rpp-test-venv/bin/python \
  -DRPP_LOCALIZATION_ENABLE_PLUGIN_TESTS=ON \
  -DRPP_LOCALIZATION_ENABLE_ROSBAG_INTEGRATION_TESTS=ON
source install/setup.bash

colcon test --packages-select rpp_localization --event-handlers console_direct+ \
  --return-code-on-test-failure
colcon test-result --verbose
