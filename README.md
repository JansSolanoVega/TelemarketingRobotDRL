# Telemarketing_Robot for autonomous navigation in dynamic obstacles using DRL

## Dependencies
* ROS Noetic
* Ubuntu 20
* Navigation package installed

## Download
```sh
$ https://github.com/aflores-c/Telemarketing_Robot
```

## Usage
1.Run GAZEBO Simulation
```sh
$ roslaunch telemarketing_gazebo telemarketing_store.launch
```

2.Run Obstacle avoidance
```sh
$ roslaunch laser_filters range_filter_example.launch
```

3.Run RVIZ and NAVIGATION. You have three options: DWA Planner, TEB Planner or MPC Planner
```sh
$ roslaunch telemarketing_navigation telemarketing_navigation.launch
or 
$ roslaunch telemarketing_navigation telemarketing_navigation_teb.launch 
or
$ roslaunch telemarketing_navigation telemarketing_navigation_mpc.launch
```

4.You can teleoperate to help the robot to localize itself
```sh
$ roslaunch telemarketing_teleop telemarketing_teleop_key.launch 
```
