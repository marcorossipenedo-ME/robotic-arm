# Direct Kinematics

## Objective:

Determine basic direct kinematics model for robot.


## Requirements:

- Modular design, capability of moving objects (motors, joints, links, ...) and adding DOFs easily.
- Input: angle between each joint and its predecessor.
- Output: position of any point of the robot with respet to any of the links.


## Architecture

As it can be seen in model\diagrams\basic_link_joint_layout.png, the robot is divided in links and joints.

Each joint represents a moving union between two links. 

L0: base link.
J0: base joint.
Rotation axis perpendicular to ground pane.

L1: first link (or first arm).
J1: joint between base and first link.
Rotation axis perpendicular to base link and paralel to ground plane.

L2: second link (or second).
J2: joint between first and second link.
Rotation axis perpendicular to first link and paralel to ground plane.

In this model the end effector is considered the end point of the last link.


- Each link and joint group has its own origin and associated vectorial space, located at the joint.
- Each link can be defined as a vector, respect to it's corresponding origin.
- The global vectorial space is defined by the ground.
- If a joint is moved, the hole vectorial space associated to it moves too, in respect to the global space.
- If a joint is rotated, the hole vectorial space associated to it rotates too, in respect to the global space.


Therfore, for each link and joint group, the following parameters exist:


## Parameters And Ecuations

For a link and joint group number n:

- X_n = [x_n, y_n, z_n, t_n] Any point in space, respect arigin n. (z is up)
- L_n = [x_{ln}, y_{ln}, z_{ln}] Link end position respect to its associated joint. Defined as a vector.
- J_n = [a_{jn}, b_{jn}, c_{jn}] Angle between a vectorial space and its predecessor. Defined as a vector. (a spin in x axis, b spin in y axis, c spin in z axis)

We can define the linear transformation:
$$
X_{n-1} = A(J_n, L_{n-1}) * X_n
$$
A is defined as:
$$
A =
\begin{bmatrix}
\cos(c_{jn}) & -\sin(c_{jn}) & 0 & 0 \\
\sin(c_{jn}) & \cos(c_{jn}) & 0 & 0 \\
0 & 0 & 1 &  \\
0 & 0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
\cos(b_{jn}) & 0 & \sin(b_{jn}) & 0 \\
0 & 1 & 0 & 0 \\
-\sin(b_{jn}) & 0 & \cos(b_{jn}) & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
\cos(a_{jn}) & -\sin(a_{jn}) & 0 & 0 \\
\sin(a_{jn}) & \cos(a_{jn}) & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$