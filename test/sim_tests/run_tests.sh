#!/usr/bin/env bash
set -eo pipefail

# Find dir location of this script
SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

# Use that location to find workspace root
WS_ROOT="$(cd "$SCRIPT_DIR/../../../../" && pwd)"

cd "$WS_ROOT"

# Source the workspace 
source $WS_ROOT/.venv/bin/activate
source /opt/ros/humble/setup.bash

colcon build --packages-select sim_tests

source install/local_setup.bash

# Select algorithm (currently only works with ftg)
export ALGORITHM_PACKAGE="ftg"
export ALGORITHM_EXECUTABLE="ftg_node"

# Select map (currently only works with levine)
export SIM_CONFIG="$SCRIPT_DIR/config/levine.yaml"

#run tests
colcon test --packages-select sim_tests \
 --return-code-on-test-failure \
 --event-handlers console_direct+

colcon test-result --verbose