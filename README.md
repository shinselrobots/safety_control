# Safety_control

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build Status](https://travis-ci.org/shinselrobots/safety_control.svg?branch=master)](https://travis-ci.org/shinselrobots/safety_control)

# Info
   This node provides basic collision / cliff detection and prevention
   To simplify behavior, potential collisions are detected by "zones" around the robot
   as defined in SafetySensorSummary

# Hints
   - Tune how long the robot stays stopped by modifying "timeout" in param/cmd_vel_mux.yaml
   - Make sure you remap /cmd_vel in your launch file
   - Tune for robot using parameters shown in the launch file.
   - To view model in RVIZ, add topic "safety_control/zone_marker".  This will display a marker 
     whenever an object is in one of the robot's danger zones

# Prerequisites
   (there may be a better way to add these packages...?)
   - modify "cmd_vel_mux.yaml" to add this safety controller:
        name:        "Safety Control"
        topic:       "safety_control"
        timeout:     1.0
        priority:    5

   - Install required packages:
Use the following command to install the required build and runtime dependencies:
```bash
cd <catkin workspace>
rosdep install --from-paths `pwd`/src --ignore-src --rosdistro=kinetic -y
```

