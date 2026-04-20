# Direct Kinematics

## Objective:

Determine basic direct kinematics model for robot.


## Requirements:

- Modular design, capability of moving objects (motors, joints, links, ...) and adding DOFs easily.
- Input: angle between each joint and its predecessor.
- Output: position of any point of the robot with respet to any of the links.


## Architecture

As it can be seen in model\diagrams\basic_link_joint_layout.png, the robot is divided in links and joints.

Each joint defines one degree of freedom between two consecutive rigid links.

L0: Base link.
J0: Base joint.
Rotation axis perpendicular to ground pane.

L1: First link (or first arm).
J1: Joint between base and first link.
Rotation axis perpendicular to base link and paralel to ground plane.

L2: Second link (or second).
J2: Joint between first and second link.
Rotation axis perpendicular to first link and paralel to ground plane.

In this model the end effector is considered the end point of the last link.


- Each link and joint group has its own reference frame, with its origin located at the joint.
- Each link can be defined as a vector, respect to it's corresponding previous joit origin.
- The global reference frame is defined by the ground.
- If a joint is moved, the hole reference frame to it moves too, in respect to the global reference frame.
- If a joint is rotated, the hole reference frame associated to it rotates too, in respect to the global reference frame.


Therfore, for each link and joint group, the following parameters exist:


## Parameters And Ecuations

For a link and joint group number n:

Any point in space, defined in n reference frame.
$$
X_n = 
\begin{bmatrix}
x_{n}\\
y_{n}\\
z_{n}\\
1
\end{bmatrix}  
$$
Link end position respect to its associated joint. Defined as a vector in n reference frame. Unione between two joints.
$$
L_n=
\begin{bmatrix}
x_{ln}\\
y_{ln}\\
z_{ln}
\end{bmatrix} 
$$
Angle between a reference frame and its predecessor around the n joint rotation axis. Defined as a escalar.
$$
j_n
$$
The following matrices can be defined:

Rotation around the x axis of the n reference frame:
$$
A_x =
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & \cos(j_n) & -\sin(j_n) & 0 \\
0 & \sin(j_n) & \cos(j_n) & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Rotation around the y axis of the n reference frame:
$$
A_y = \begin{bmatrix}
\cos(j_n) & 0 & \sin(j_n) & 0 \\
0 & 1 & 0 & 0 \\
-\sin(j_n) & 0 & \cos(j_n) & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Rotation around the z axis of the n reference frame:
$$
A_z = \begin{bmatrix}
\cos(j_n) & -\sin(j_n) & 0 & 0 \\
\sin(j_n) & \cos(j_n) & 0 & 0 \\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Movement following L_n vector:
$$
A_m = \begin{bmatrix}
1 & 0 & 0 & x_{ln} \\
0 & 1 & 0 & y_{ln} \\
0 & 0 & 1 & z_{ln}\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

A combines all previous transformations and can be defined as:
$$
A(j_n, L_{n-1}) = A_m(L_{n-1}) * A_r(j_n)
$$
In this project application, A does not need all rotation matrices, as each joint will have only one axis of rotation. Ar represents the rotation axis choosen. Transformations are applied from right to left.

Using A, the follownig linear transformation can be defined:
$$
X_{n-1} = A(j_n, L_{n-1}) * X_n
$$

In which, given some coordinates defined in a n joint reference frame, it outputs the same point in space defined in a n-1 joint reference frame. Using the angles between reference frames (J_n) and the position of the n coordinate system center, all defined in the n-1 reference frame.

The inverse matrix of A can determine, given a point in space defined in a n-1 joint reference frame, the same point defined in a n joint reference frame.

An unique A matrix can be defined for each joint in the robot. For example:
$$
A_2: \text{ J2 to J1}\\
A_1: \text{ J1 to J0} \\
A_0: \text{ J0 to ground reference plane}
$$
and 
$$
A_0^{-1}: \text{ Ground reference plane to J0} \\
A_1^{-1}: \text{ J0 to J1} \\
A_2^{-1}: \text{ J1 to J2} 
$$

Using each joint matrice, the following matrices can be defined:
$$
T_n^0 = A_0*A_1*A_2*[...]*A_n
$$
Ad transforms any point defined in Jn reference frame to a point defined in ground reference frame.
$$
T_0^n = A_n^{-1}*[...]*A_2^{-1}* A_1^{-1}* A_0^{-1}
$$
Ai transforms any point defined in ground reference frame to a point defined in Jn reference frame.

These are used in the following way:
$$
X_{gnd} = T_n^0*X_n
$$
$$
X_n = T_0^n*X_{gnd}
$$


## References

### Technical references
1. Rotation matrix – Wikipedia  
   https://en.wikipedia.org/wiki/Rotation_matrix#Basic_3D_rotations

2. Forward kinematics – Wikipedia  
   https://en.wikipedia.org/wiki/Forward_kinematics

3. Robot kinematics - Wikipedia
   https://en.wikipedia.org/wiki/Robot_kinematics

### Learning resources
1. Introductory robotics lecture (YouTube)  
   https://www.youtube.com/watch?v=_8T7RjXL07M