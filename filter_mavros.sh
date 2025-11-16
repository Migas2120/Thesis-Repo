#!/bin/bash

echo "=== Setpoint Topics ==="
rostopic list | grep "/mavros/setpoint"

echo -e "\n=== State Topics ==="
rostopic list | grep "/mavros/state"

echo -e "\n=== RC Override Topics ==="
rostopic list | grep "/mavros/rc"

echo -e "\n=== Vision & Pose ==="
rostopic list | grep "/mavros/.*pose"

echo -e "\n=== Services (Arm, Mode, Takeoff, Land) ==="
rosservice list | grep -E "/mavros/(cmd|set_mode)"
