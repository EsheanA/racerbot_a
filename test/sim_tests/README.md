# Simulator Tests

These are full-system tests that test out the algorithm nodes inside the simulator


## Option A: Run colcon manually

### Building
```bash
cd /racerbot_ws
colcon build --packages-select sim_tests
```

### Running the tests

```bash
colcon test --packages-select sim_tests \
 --return-code-on-test-failure \
 --event-handlers console_direct+
```
## Option B: Run using the following shell script

```bash
cd /racerbot_ws
mkdir scripts
touch scripts/run_tests.sh
```

### Paste the following script into the run_tests.sh
```bash
#!/usr/bin/env bash
set -eo pipefail

# Find dir location of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Use that location to find workspace root (Assumes script is in a scripts folder)
WS_ROOT="$(cd "$SCRIPT_DIR/../" && pwd)"

cd "$WS_ROOT"

# Source the workspace (Example)
source /racerbot_ws/.venv/bin/activate
source /opt/ros/humble/setup.bash

colcon build --packages-select sim_tests

source install/local_setup.bash

# Select algorithm (currently only works with ftg)
export ALGORITHM_PACKAGE="ftg"
export ALGORITHM_EXECUTABLE="ftg_node"

# Select map (currently only works with levine)
export SIM_CONFIG="/racerbot_ws/src/racerbot_a/test/sim_tests/config/levine.yaml"

#run tests
colcon test --packages-select sim_tests \
 --return-code-on-test-failure \
 --event-handlers console_direct+

colcon test-result --verbose
```


