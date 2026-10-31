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

#build algorithm nodes
colcon build --packages-select \
    ftg \
    disparity_extender \
    pure_pursuit_a \
    reactive \

#build safety node 
colcon build --packages-select safety_node
    
source install/local_setup.bash

MAP_CONFIG="$WS_ROOT/src/f1tenth_gym_ros/config"
MAPS_DIR="$WS_ROOT/src/f1tenth_gym_ros/maps"

MAPS=("levine" "Spielberg")

ALGORITHM_PACKAGES=("disparity_extender" "ftg" "pure_pursuit_a" "reactive")
ALGORITHM_EXECUTABLES=("disparity_extender_node" "ftg_node" "pure_pursuit_a_node" "reactive_node")

# Select algorithm (currently only works with ftg)
export ALGORITHM_PACKAGE="ftg"
export ALGORITHM_EXECUTABLE="ftg_node"

# Select map (currently only works with levine)
export SIM_CONFIG="$SCRIPT_DIR/config/sim.yaml"


for map in "${MAPS[@]}"; do
    #Select map 
    #Using stream editor to replace the whole line
    #for e.g #thing to be replaced#replacement#
    sed -i "s#map_path: '.*'#map_path: '${MAPS_DIR}/${map}'#" $SIM_CONFIG
    for ((i=0; i<${#ALGORITHM_EXECUTABLE[@]}; i++)); do
        export ALGORITHM_PACKAGE=${ALGORITHM_PACKAGES[i]}
        export ALGORITHM_EXECUTABLE=${ALGORITHM_EXECUTABLES[i]}

        #run tests
        colcon test --packages-select sim_tests \
            --return-code-on-test-failure \
            --event-handlers console_direct+
        colcon test-result --verbose
    done
done



