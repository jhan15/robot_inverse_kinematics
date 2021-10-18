# robot_inverse_kinematics

## Project setup

```bash
# put the package in the workspace
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
```

## Description

In this project, we program the inverse kinematic algorithm for a robot, which will move its joints so that it follows a desired path with the end-effector.

It is composed of two parts:

* A 3 DOF scara robot, with an inverse kinematic solution that is possible in analytic form.
* A 7 DOF kuka robot, with the inverse kinematic solution to be implemented with iterative algorithms.

## Run the simulator

Scara robot

```bash
$ roslaunch kinematics_assignment scara_launch.launch
```

Kuka robot

```bash
$ roslaunch kinematics_assignment kuka_launch.launch
```

## Simulation result (kuka)

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/137804639-fe2ab29b-333c-4385-b9d8-b2332268e9a0.gif" width="600">
</p>
