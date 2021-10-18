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

### Scara robot

The following image shows the distances between the joints and the end-effector frame, in the robot's zero configuration. Two joints (q1 and q2) are revolute, and one (q3) is prismatic. Notice that the end-effector frame and the base frame are at the same height, which means that the end-effector z coordinate coincides with the value of the last prismatic joint (q3).

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/137815066-2c2df8a5-2ec3-4d06-b2e2-c1e209198b69.png" width="400">
</p>

### Kuka robot

This robot has a kinematics structure much more complex than the scara, therefore it is not feasible to obtain an analytic solution for the inverse kinematics problem. The inputs to this function are: 

* point = (x, y, z), the desired position of the end-effector.
* R = 3x3 rotation matrix, the desired orientation of the end-effector.
* joint_positions = (q1, q2, q3, q4, q5, q5, q7) the current joint positions.

The output of this function is a vector q containing the 7 joint values that give the desired pose of the end-effector.

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/137815059-03c594ee-d76e-4ef5-887e-8d750aba119c.png" width="400">
</p>

This is the DH table of the kuka robot, with the depicted frames:

<p align="center">
  <img src="https://user-images.githubusercontent.com/62132206/137815470-a0dfa099-7aae-4151-bea9-af3dadf71ed4.png" width="400">
</p>

The DH table follows this convention:

𝑎𝑖 distance between 𝑧𝑖−1 and 𝑧𝑖 along the axis 𝑥𝑖
𝛼𝑖 angle between 𝑧𝑖−1 and 𝑧𝑖 about the axis 𝑥𝑖
𝑑𝑖 distance between 𝑥𝑖−1 and 𝑥𝑖 along the axis 𝑧𝑖−1
𝜃𝑖 angle between 𝑥𝑖−1 and 𝑥𝑖 about the axis 𝑧𝑖−1
The frame transformation can be found as:

𝑖−1𝑇𝑖=𝑇𝑟𝑎𝑛𝑠(𝑧𝑖−1,𝑑𝑖)𝑅𝑜𝑡(𝑧𝑖−1,𝜃𝑖)𝑇𝑟𝑎𝑛𝑠(𝑥𝑖,𝑎𝑖)𝑅𝑜𝑡(𝑥𝑖,𝛼𝑖)

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
