#!/usr/bin/env bash
# Install, build, and register rpp_localization in a sourced ROS 2 workspace.
#
# The core library needs the plugin-type headers that registration generates,
# and the plugins link against the core library that the build produces, so
# the library is registered before and after the build. The first
# registration only needs to generate the headers and may fail at the
# plugins.
set -euo pipefail

package_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
workspace_dir=${1:-$(pwd)}

if [ -z "${ROS_DISTRO:-}" ]; then
    echo "Source a ROS 2 environment first." >&2
    exit 1
fi
if [ ! -d "${workspace_dir}/src" ]; then
    echo "Usage: $0 [workspace directory containing src/]" >&2
    exit 1
fi

# rpp_cpp is built from source in the workspace and has no rosdep key.
sudo apt-get update
rosdep update --rosdistro "${ROS_DISTRO}"
rosdep install --from-paths "${package_dir}" --ignore-src -y \
    --rosdistro "${ROS_DISTRO}" --skip-keys rpp_cpp

echo "Generating the plugin-type headers (first registration pass)."
rpp library register "${package_dir}" --link > /dev/null 2>&1 || true

cd "${workspace_dir}"
colcon build --packages-select rpp_localization --symlink-install \
    --cmake-args -DBUILD_TESTING=OFF
# shellcheck disable=SC1091
set +u; source install/setup.bash; set -u

rpp library register "${package_dir}" --link
echo "rpp_localization is built and registered."
