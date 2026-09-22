# Direct Kinematics

## Objective:

Determine basic direct kinematics model for robot. 

Given a cartesian coordinates defined in a reference frame, the DK model should be able to change the reference frame in which the coordinates are defined.


## Parameters And Ecuations

For a link \(L_n\):

- Any point in space, defined in \(R_n\).
```math
{}^{n}X = 
\begin{bmatrix}
x_{n}\\
y_{n}\\
z_{n}\\
1
\end{bmatrix}  
```
- Link displacement vector defined in \(R_n\). \(L_n\) represents the displacement from the joint \(J_n\), located at the start of link \(L_n\), to the origin of the reference frame \(R_n\), located at the end of the link. The vector is expressed in \(R_n\), and therefore moves and rotates together with \(R_n\).
```math
L_n=
\begin{bmatrix}
x_{ln}\\
y_{ln}\\
z_{ln}
\end{bmatrix} 
```
- Angle between a link and its predecessor around \(J_n\) joint rotation axis. Defined as a scalar.
```math
\theta_n
```
The following matrices can be defined:

- Rotation \(theta_n\) around the x axis in \(R_n-1\):
```math
R_x =
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & \cos(\theta_n) & -\sin(\theta_n) & 0 \\
0 & \sin(\theta_n) & \cos(\theta_n) & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
```

- Rotation \(theta_n\) around the y axis in \(R_n-1\):
```math
R_y = \begin{bmatrix}
\cos(\theta_n) & 0 & \sin(\theta_n) & 0 \\
0 & 1 & 0 & 0 \\
-\sin(\theta_n) & 0 & \cos(\theta_n) & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
```

- Rotation \(theta_n\) around the z axis in \(R_n-1\):
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

An unique A matrix can be defined for each joint in the robot. For example:
```math
{}^{1}A_2: \text{ J2 to J1}\\
{}^{0}A_1: \text{ J1 to J0} \\
{}^{g}A_0: \text{ J0 to ground reference plane}
```
and 
```math
{}^{0}A_g: \text{ Ground reference plane to J0} \\
{}^{1}A_0: \text{ J0 to J1} \\
{}^{2}A_1: \text{ J1 to J2} 
```

## Global Matrix


Using each joint matrix, the following matrices can be defined:
```math
{}^{g}T_n = {}^{g}A_0*{}^{0}A_1*[...]*{}^{n-1}A_n
```
This matrix transforms any point defined in \(R_n\) to a point defined in \(R_g\).
```math
{}^{n}T_g = {}^{n}A_{n-1}*[...]* {}^{1}A_0* {}^{0}A_g
```
This matrix transforms any point defined in \(R_g\) to a point defined in \(R_n\).

These are used in the following way:
```math
X_{g} = {}^{g}T_n*X_n
```
```math
X_n = {}^{n}T_g*X_{g}
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

\(J_0\) rotates around \(R_g\) z axis.

\(J_1\) and \(J_2\) rotate around \(R_0\) and \(R_1\) y axis, respectively.

When \(q=0\), each link stays alligned to \(R_g\) z axis.


## End Effector Global Coordinates

In order to get the cartesian coordinates of the end effector defined on the ground reference frame given each joint angle, the zero vector should be used. As the end effector is located at the center of \(R_2\). 
```math
X_{zero}={}^{2}X_{EE}=\begin{bmatrix}
0\\
0\\
0\\
1
\end{bmatrix}  
```
```math
{}^{g}X_{EE}={}^{g}T_2*{}^{2}X_{EE}
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