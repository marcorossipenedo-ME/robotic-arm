# Direct Kinematics

## Objective

Determine basic direct kinematics model for robot. 

Given a cartesian coordinates defined in a reference frame, the DK model should be able to change the reference frame in which the coordinates are defined.

## Equations

The following matrices can be defined:

- Rotation \(theta_n\) around the x axis in \(RF_n-1\):
```math
R_x =
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & \cos(\theta_n) & -\sin(\theta_n) & 0 \\
0 & \sin(\theta_n) & \cos(\theta_n) & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
```

- Rotation \(theta_n\) around the y axis in \(RF_n-1\):
```math
R_y = \begin{bmatrix}
\cos(\theta_n) & 0 & \sin(\theta_n) & 0 \\
0 & 1 & 0 & 0 \\
-\sin(\theta_n) & 0 & \cos(\theta_n) & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
```

- Rotation \(theta_n\) around the z axis in \(RF_n-1\):
```math
R_z = \begin{bmatrix}
\cos(\theta_n) & -\sin(\theta_n) & 0 & 0 \\
\sin(\theta_n) & \cos(\theta_n) & 0 & 0 \\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
```

- Movement following \(L_n\) vector:
```math
T = \begin{bmatrix}
1 & 0 & 0 & x_{ln} \\
0 & 1 & 0 & y_{ln} \\
0 & 0 & 1 & z_{ln}\\
0 & 0 & 0 & 1
\end{bmatrix}
```

A combines all previous transformations and can be defined as:
```math
{}^{n-1}A(\theta_n, L_{n})_n = R_n(\theta_n)*T(L_{n})
```
In this project application, A does not need all rotation matrices, as each joint will have only one axis of rotation. Ar represents the rotation axis choosen. Transformations are applied from right to left.

Using A, the follownig linear transformation can be defined:
```math
X_{n-1} = {}^{n-1}A(\theta_n, L_{n})_n * X_n
```
Note that in this configuration, the homogeneous transformation is applied as a translation followed by a rotation.

This ordering is important because when testing the transformation using the origin vector ([0,0,0,1]^T), the rotational component does not produce any visible effect, since the origin is invariant under rotation. As a result, the interpretation of the transformation effect is not the desired if evaluated at the origin.

The inverse matrix of A can determine, given a point in space defined in a n-1 joint reference frame, the same point defined in a n joint reference frame.

An unique A matrix can be defined for each link in the robot. For example:
```math
{}^{2}A_3: \text{ RF3 to RF2}\\
{}^{1}A_2: \text{ RF2 to RF1} \\
{}^{0}A_1: \text{ RF1 to RF0}
```
and 
```math
{}^{1}A_0: \text{ RF0 to RF1} \\
{}^{2}A_1: \text{ RF1 to RF2} \\
{}^{3}A_2: \text{ RF2 to RF3} 
```

## Global Matrix

Using each joint matrix, the following matrices can be defined:
```math
{}^{0}T_n = {}^{0}A_1*{}^{1}A_2*[...]*{}^{n-1}A_n
```
This matrix transforms any point defined in \(RF_n\) to a point defined in \(RF_0\).
```math
{}^{n}T_0 = {}^{n}A_{n-1}*[...]* {}^{2}A_1* {}^{1}A_0
```
This matrix transforms any point defined in \(RF_0\) to a point defined in \(RF_n\).

These are used in the following way:
```math
X_{0} = {}^{0}T_n*X_n
```
```math
X_n = {}^{n}T_0*X_{0}
```


Based in the 3DOF robot diagram, the following global matrix can be defined:
```math
{}^{g}T_n = \begin{bmatrix}
cos(\theta_0)cos(\theta_1+\theta_2) & -sin(\theta_0) & cos(\theta_0)sin(\theta_1+\theta_2) & cos(\theta_0​)(l_1sin(\theta_1)​+l_2sin(\theta_1​+\theta_2​)) \\
sin(\theta_0)cos(\theta_1+\theta_2) & cos(\theta_0) & sin(\theta_0)sin(\theta_1+\theta_2) & sin(\theta_0​)(l_1sin(\theta_1)​+l_2sin(\theta_1​+\theta_2​)) \\
-sin(\theta_1+\theta_2) & 0 & cos(\theta_1+\theta_2) & l_0+l_1cos(\theta_1)​+l_2cos(\theta_1​+\theta_2​)\\
0 & 0 & 0 & 1
\end{bmatrix}
=
\begin{bmatrix}
R & p\\
0 & 1 
\end{bmatrix}
```
```math
{}^{n}T_g = 
\begin{bmatrix}
R^T & -R^T*p\\
0 & 1 
\end{bmatrix}
```
Where:

\(J_1\) rotates around \(RF_0\) z axis.

\(J_2\) and \(J_3\) rotate around \(RF_1\) and \(RF_2\) y axis, respectively.

When \(q=0\), each link stays alligned to \(RF_0\) z axis.


## End Effector Global Coordinates

In order to get the cartesian coordinates of the end effector defined on the ground reference frame given each joint angle, the zero vector should be used. As the end effector is located at the center of \(RF_2\). 
```math
X_{zero}={}^{2}X_{EE}=\begin{bmatrix}
0\\
0\\
0\\
1
\end{bmatrix}  
```
```math
{}^{0}X_{EE}={}^{0}T_2*{}^{2}X_{EE}
```
This gives the following equations, which can be used to determine the global cartesian coordinates of the end effector, given each joint angle:
```math
x=cos(\theta_0​)(l_1sin(\theta_1)​+l_2sin(\theta_1​+\theta_2​)) \\
y=sin(\theta_0​)(l_1sin(\theta_1)​+l_2sin(\theta_1​+\theta_2​)) \\
z=l_0+l_1cos(\theta_1)​+l_2cos(\theta_1​+\theta_2​)
```
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