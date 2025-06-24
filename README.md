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
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10e robot_ip:=192.168.56.101 initial_joint_controller:=motion_primitive_controller launch_rviz:=false
```
(optional) switch control mode
```
ros2 control switch_controllers --activate motion_primitive_controller --deactivate scaled_joint_trajectory_controller
```
Start MoveIt
```
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur10e launch_rviz:=true
```

## Process planned trajectory
Start the python script with the following command:
```
ros2 run primitives_from_planned_trajectory process_trajectory_to_primitives
```
Then plan a trajectory in RViz with MoveIt by pressing `plan`. The python script will:
    1. Read the planned trajectory from `/display_planned_path`.
    2. Calculate the endefector pose for every point in the trajectory using the `/compute_fk` service.
    3. Save the trajectory and endefector poses to a `planned_trajectory_<date>_<time>.csv` file.
    4. Approximate the movement with motion primitives.
    5. Polt the trajectory vs the aproximated motion primitives path.
    6. Ask user if planned primitives should get executed.
    7. Execution using the [motion_primitives_forward_controller](https://github.com/b-robotized-forks/ros2_controllers/tree/motion_primitive_forward_controller/motion_primitives_forward_controller).
    8. Save the executed trajectory to compare it with the planned trajectory.

## Plot saved planned trajectory
```
ros2 run primitives_from_planned_trajectory plot_saved_planned_trajectory "<filename>"
```
(The filename can either be passed as a command line argument or specified directly in the Python script.)

## Plot saved executed trajectory
```
ros2 run primitives_from_planned_trajectory plot_saved_executed_trajectory "<filename>"
```
(The filename can either be passed as a command line argument or specified directly in the Python script.)

## Compare planned and executed trajectory
```
ros2 run primitives_from_planned_trajectory compare_planned_and_executed_traj
```