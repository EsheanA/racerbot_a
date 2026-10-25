#!/usr/bin/env bash
set -eo pipefail

#find dir location of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
#use that location to find workspace root
WS_ROOT="$(cd "$SCRIPT_DIR/../../../../")"

cd "$WS_ROOT"

#source and build
source /racerbot_ws/.venv/bin/activate
source /opt/ros/humble/setup.bash

colcon build --packages-select sim_tests

source install/local_setup.bash

# Select algorithm
export ALGORITHM_PACKAGE="ftg"
export ALGORITHM_EXECUTABLE="ftg_node"

# Select map
export SIM_CONFIG="/racerbot_ws/src/racerbot_a/test/sim_tests/config/levine.yaml"

#run tests
colcon test --packages-select sim_tests \
 --return-code-on-test-failure \
 --event-handlers console_direct+

colcon test-result --verbose