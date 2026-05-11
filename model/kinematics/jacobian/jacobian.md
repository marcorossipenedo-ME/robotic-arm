# Jacobian Matrix

## Objective:

Determine basic jacobian matrix model for robot and explore its applications.


## Requirements:

- Modular design, capability of moving objects (motors, joints, links, ...) and adding DOFs easily.
- Input: joint angular velocity.
- Output: end effector linear velocity.


## Architecture

From the direct kinematics model:
```math
x=cos(\theta_0​)(l_1​cos(\theta_1)​+l_2​cos(\theta_1​+\theta_2​)) \\
y=sin(\theta_0​)(l_1​cos(\theta_1)​+l_2​cos(\theta_1​+\theta_2​)) \\
z=l_1​sin(\theta_1)​+l_2​sin(\theta_1​+\theta_2​)
```
Note that the previously used j_n notation for joint angles is being changed to theta_n, in order to increase point differentiation notation identification.

If theta_n = theta_n(t) is assumed and differentiation in function of time is done on both sides:
```math
\dot{x}=-\dot{\theta_0}l_1sin(\theta_0​)​cos(\theta_1)-\dot{\theta_1}l_1sin(\theta_1​)​cos(\theta_0)-\dot{\theta_0}l_2sin(\theta_0​)​cos(\theta_1+\theta_2)-(\dot{\theta_1}+\dot{\theta_2})l_2sin(\theta_1+\theta_2​)​cos(\theta_0) \\

\dot{y}=\dot{\theta_0}l_1cos(\theta_0​)​cos(\theta_1)-\dot{\theta_1}l_1sin(\theta_1​)sin(\theta_0)+\dot{\theta_0}l_2cos(\theta_0​)​cos(\theta_1+\theta_2)-(\dot{\theta_1}+\dot{\theta_2})l_2sin(\theta_1+\theta_2​)sin(\theta_0) \\

\dot{z}=\dot{\theta_1}l_1cos(\theta_1)​+(\dot{\theta_1}+\dot{\theta_2})l_2cos(\theta_1​+\theta_2​)

```

The following matrix can be defined:
```math
J(\theta_0, \theta_1, \theta_2) = \begin{bmatrix}
-sin(\theta_0)(l_1cos(\theta_1)+l_2cos(\theta_1+\theta_2)) & -cos(\theta_0)(l_1sin(\theta_1)+l_2sin(\theta_1+\theta_2)) & -l_2cos(\theta_0)sin(\theta_1+\theta_2) \\
cos(\theta_0)(l_1cos(\theta_1)+l_2cos(\theta_1+\theta_2)) & -sin(\theta_0)(l_1sin(\theta_1)+l_2sin(\theta_1+\theta_2)) & -l_2sin(\theta_0)sin(\theta_1+\theta_2) \\
0 & l_1cos(\theta_1)​+l_2cos(\theta_1​+\theta_2​) & l_2cos(\theta_1​+\theta_2​)
\end{bmatrix}
```

Using this matrix, any point in space can be defined by the following application:
```math
p=\begin{bmatrix}
x \\
y \\
z
\end{bmatrix}
```

```math
\theta =\begin{bmatrix}
\theta_0 \\
\theta_1 \\
\theta_2
\end{bmatrix}
```

```math
\dot{p}  =  J(\theta)*\dot{\theta}
```

## Singularities



## References

### Learning resources
1. Introductory general lecture (YouTube) 
   https://www.youtube.com/watch?v=bohL918kXQk

2. Introductory general lecture (YouTube) 
   https://www.youtube.com/watch?v=h2YM0CDzDl4&t=1s

3. Introductory robotics oriented lecture
   https://publish.illinois.edu/ece470-intro-robotics/files/2021/09/ECE470Lec10FA21.pdf