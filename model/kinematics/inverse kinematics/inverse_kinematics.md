# Inverse Kinematics

## Objective:

Determine basic inverse kinematics model for robot.


## Requirements:

- Modular design, capability of moving objects (motors, joints, links, ...) and adding DOFs easily.
- Input: position of any point of the robot with respet to any of the links.
- Output: angle between each joint and its predecessor.


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

- Any point in space, defined in n reference frame.
```math
X_n = 
\begin{bmatrix}
x_{n}\\
y_{n}\\
z_{n}\\
1
\end{bmatrix}  
```
- Link end position respect to its associated joint. Defined as a vector in n reference frame. Unione between two joints.
```math
L_n=
\begin{bmatrix}
x_{ln}\\
y_{ln}\\
z_{ln}
\end{bmatrix} 
```
- Angle between a reference frame and its predecessor around the n joint rotation axis. Defined as a escalar.
```math
j_n
```

Based in the direct kinematics model matrix, which for a determined X_2 point in space in the end effector reference frame, gives a X_gnd point in ground reference frame:
```math
T_n^0 = 
\begin{bmatrix}
cos(j_0)*cos(j_1+j_2) & -sin(j_0) & cos(j_0)*sin(j_1+j_2) & cos(j_0​)*(l_1*​cos(j_1)​+l_2​*cos(j_1​+j_2​)) \\
sin(j_0)*cos(j_1+j_2) & cos(j_0) & sin(j_0)*sin(j_1+j_2) & sin(j0​)*(l_1*​cos(j_1)​+l_2*​cos(j_1​+j_2​)) \\
-sin(j_1+j_2) & 0 & cos(j_1+j_2) & l_0​+l_1*​sin(j_1)​+l_2*​sin(j_1​+j_2​)\\
0 & 0 & 0 & 1
\end{bmatrix}
```
If the input X_2 point is a zero vector, it is referring to the end effector position, as it is the center of its own reference frame.

- This can be used to determine the global position of the end effector using each joint angle.
- This can be used to determine each joint angle needed to put the end effector in a determined global position.

The global position of the end effector, by inputting a zero vector into the matrix mentioned, is defined as:
```math
\begin{bmatrix}
x \\
y \\
z \\
1
\end{bmatrix}
=
\begin{bmatrix}
cos(j_0)*cos(j_1+j_2) & -sin(j_0) & cos(j_0)*sin(j_1+j_2) & cos(j_0​)*(l_1*​cos(j_1)​+l_2​*cos(j_1​+j_2​)) \\
sin(j_0)*cos(j_1+j_2) & cos(j_0) & sin(j_0)*sin(j_1+j_2) & sin(j_0​)*(l_1*​cos(j_1)​+l_2*​cos(j_1​+j_2​)) \\
-sin(j_1+j_2) & 0 & cos(j_1+j_2) & l_0​+l_1*​sin(j_1)​+l_2*​sin(j_1​+j_2​)\\
0 & 0 & 0 & 1
\end{bmatrix}

\begin{bmatrix}
0 \\
0 \\
0 \\
1
\end{bmatrix}
=
\begin{bmatrix}
cos(j_0​)*(l_1*​cos(j_1)​+l_2​*cos(j_1​+j_2​)) \\
sin(j_0​)*(l_1*​cos(j_1)​+l_2*​cos(j_1​+j_2​)) \\
l_1*​sin(j_1)​+l_2*​sin(j_1​+j_2​) \\
1
\end{bmatrix}
```
This defines the following equation system:
```math
x=cos(j_0​)*(l_1*​cos(j_1)​+l_2​*cos(j_1​+j_2​)) \\
y=sin(j_0​)*(l_1*​cos(j_1)​+l_2*​cos(j_1​+j_2​)) \\
z=l_1*​sin(j_1)​+l_2*​sin(j_1​+j_2​)
```
Using this system of equations it is possible to express each angle as a function of the global position of the end effector:
```math
j_0​=atan2(y,x)
```
```math
j_1=atan2(z,\sqrt{x^2+y^2})−atan2(l_2​*sin(j_2)​,l_1​+l_2*​cos(j_2​))
```
```math
j_2=±arccos(\frac{x^2+y^2+z^2−l_1^2​−l_2^2}{2*l_1*l_2​​})
```