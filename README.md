# Safety_control

# Info
   This node provides basic collision / cliff detection and prevention
   To simplify behavior, potential collisions are detected by "zones" around the robot
   as defined in SafetySensorSummary

# Hints
   - Tune how long the robot stays stopped by modifying "timeout" in param/cmd_vel_mux.yaml
   - Make sure you remap /cmd_vel in your launch file
   - Tune for robot using parameters shown in the launch file.



