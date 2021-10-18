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

* A 3 DOF scara robot, with an inverse kinematic solution that is possible in analytic form. Solving this part is the minimum requirement to pass the assignment.
* A 7 DOF kuka robot, with the inverse kinematic solution to be implemented with iterative algorithms. Solving this part will guarantee the best grade for the assignment.
