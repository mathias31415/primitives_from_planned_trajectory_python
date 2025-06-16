primitives_from_planned_trajectory
==========================================

Package to approximate a planned trajectory using motion primitives such as PTP, LIN, and CIRC

![Licence](https://img.shields.io/badge/License-Apache-2.0-blue.svg)


# Usage notes
## Launch UR Driver
**With Simulation**
Start Simulation
```
ros2 run ur_client_library start_ursim.sh -m ur10e
```
Start Driver
```
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=192.168.56.101 initial_joint_controller:=scaled_joint_trajectory_controller launch_rviz:=false
```
Start MoveIt
```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10e launch_rviz:=true
```

## Save planned trajectory
Start the python script with the following command:
```
ros2 run primitives_from_planned_trajectory save_planned_trajectory
```

Then plan a trajectory in RViz with MoveIt by pressing `plan`. The python script will save the trajectory to a `planned_trajectory_<date>_<time>.csv` file.

## Plot saved trajectory
```
ros2 run primitives_from_planned_trajectory plot_saved_trajectory "<filename>"
```
(The filename can either be passed as a command line argument or specified directly in the Python script.)
