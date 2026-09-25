# Inverse Kinematics

## Objective:

Determine basic inverse kinematics model for robot.

Given a cartesian coordinates defined in the ground reference frame describing the position of the end effector, the IK model should be able to output each joint necessary angle to put the end effector in the input coordinates.


## Equations

Based in the following direct kinematics model matrix:
```math
{}^{0}T_n = 
\begin{bmatrix}
cos(\theta_0)*cos(\theta_1+\theta_2) & -sin(\theta_0) & cos(\theta_0)*sin(\theta_1+\theta_2) & cos(\theta_0​)*(l_1*​cos(\theta_1)​+l_2​*cos(\theta_1​+\theta_2​)) \\
sin(\theta_0)*cos(\theta_1+\theta_2) & cos(\theta_0) & sin(\theta_0)*sin(\theta_1+\theta_2) & sin(j0​)*(l_1*​cos(\theta_1)​+l_2*​cos(\theta_1​+\theta_2​)) \\
-sin(\theta_1+\theta_2) & 0 & cos(\theta_1+\theta_2) & l_0​+l_1*​sin(\theta_1)​+l_2*​sin(\theta_1​+\theta_2​)\\
0 & 0 & 0 & 1
\end{bmatrix}
```
If the input point is a zero vector, it is referring to the end effector position, as it is the center of its own reference frame.

- This can be used to determine the global position of the end effector using each joint angle.
- This can be used to determine each joint angle needed to put the end effector in a determined cartesian \(RF_0\) position.

The \(RF_0\) cartesian position of the end effector, by inputting a zero vector into the matrix mentioned, is defined as:
```math
\begin{bmatrix}
x \\
y \\
z \\
1
\end{bmatrix}
=
\begin{bmatrix}
cos(\theta_0)*cos(\theta_1+\theta_2) & -sin(\theta_0) & cos(\theta_0)*sin(\theta_1+\theta_2) & cos(\theta_0​)*(l_1*​cos(\theta_1)​+l_2​*cos(\theta_1​+\theta_2​)) \\
sin(\theta_0)*cos(\theta_1+\theta_2) & cos(\theta_0) & sin(\theta_0)*sin(\theta_1+\theta_2) & sin(\theta_0​)*(l_1*​cos(\theta_1)​+l_2*​cos(\theta_1​+\theta_2​)) \\
-sin(\theta_1+\theta_2) & 0 & cos(\theta_1+\theta_2) & l_0​+l_1*​sin(\theta_1)​+l_2*​sin(\theta_1​+\theta_2​)\\
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
cos(\theta_0​)(l_1sin(\theta_1)​+l_2sin(\theta_1​+\theta_2​)) \\
sin(\theta_0​)(l_1sin(\theta_1)​+l_2sin(\theta_1​+\theta_2​)) \\
l_0+l_1cos(\theta_1)​+l_2cos(\theta_1​+\theta_2​) \\
1
\end{bmatrix}
```
This defines the following equation system:
```math
x=cos(\theta_0​)(l_1sin(\theta_1)​+l_2sin(\theta_1​+\theta_2​)) \\
y=sin(\theta_0​)(l_1sin(\theta_1)​+l_2sin(\theta_1​+\theta_2​)) \\
z=l_0+l_1cos(\theta_1)​+l_2cos(\theta_1​+\theta_2​)
```
Using this system of equations it is possible to express each angle as a function of the \(RF_0\) cartesian coordinates of the end effector:
```math
\theta_0​=atan2(y,x)
```
```math
\theta_1=atan2(z,\sqrt{x^2+y^2})−atan2(l_2​*sin(\theta_2)​,l_1​+l_2*​cos(\theta_2​))
```
```math
\theta_2=±arccos(\frac{x^2+y^2+z^2−l_1^2​−l_2^2}{2*l_1*l_2​​})
```

## References

### Learning resources
1. Introductory robotics lecture (YouTube) 
   https://www.youtube.com/watch?v=8D0sO8mymQ8