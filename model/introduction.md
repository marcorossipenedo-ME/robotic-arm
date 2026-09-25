# Objective

- Defining a mathematical and phyisical model of the robotic arm.
- Being able to simulate the robotic arm using python.
- Being able to use the model to control the real arm.


# Requirements

- Modular design, capability of moving objects (motors, joints, links, ...) and adding DOFs easily.
- Visual output and display.
- Gravity accountability.
- Coriolis accountability.
- External forces accountability.
  

# Final result

- Given a desired cartesian path, the program should be able to simulate the movement and calculate articulation torques in real time.


# Architecture

As it can be seen in model\diagrams\basic_link_joint_layout.png, the robot is divided in links, joints and reference frames.

- EE: End effector. In this model the end effector is considered the end point of the second link.
- \(RF_0\): Ground reference frame.
- \(RF_i\) is the reference frame attached to link \(L_i\), with its origin located at the end of the link.
- Each joint \(J_i\) defines one degree of freedom between two consecutive rigid links. It is located at the start of the link \(L_i\).
- If a joint moves, the associated link and all subsequent reference frames are transformed accordingly with respect to the global reference frame.
- If a joint rotates, the orientation of the associated link and all subsequent reference frames changes accordingly with respect to the global reference frame.
- The global reference frame is defined by the ground.


# Notation

An example of the notation used in the model.

- Any point in space, defined in \(RF_n\).
```math
{}^{n}X
```

- i link velocity, defined in \(RF_n\).
```math
{}^{n}v_i
```

- i link velocity first derivative, defined in \(RF_n\).
```math
{}^{n}\dot{v_i}
```

# Generalized coordinates

The configuration of the robot is described by the vector of joint coordinates:
```math
q= \begin{bmatrix} \theta_0\\ \theta_1\\ \theta_2 \end{bmatrix} 
```
Its first and second time derivatives are:
```math
\dot q= \begin{bmatrix} \dot\theta_0\\ \dot\theta_1\\ \dot\theta_2 \end{bmatrix}\\

\ddot q= \begin{bmatrix} \ddot\theta_0\\ \ddot\theta_1\\ \ddot\theta_2 \end{bmatrix} 
```
These variables define the instantaneous configuration, velocity and acceleration of the robot.

For a link \(L_n\):

- Any point in space, defined in \(RF_n\).
```math
{}^{n}X = 
\begin{bmatrix}
x_{n}\\
y_{n}\\
z_{n}\\
1
\end{bmatrix}  
```
- Link displacement vector defined in \(RF_n\). \(L_n\) represents the displacement from the joint \(J_n\), located at the start of link \(L_n\), to the origin of the reference frame \(RF_n\), located at the end of the link. The vector is expressed in \(RF_n\), and therefore moves and rotates together with \(RF_n\).
```math
L_n=
\begin{bmatrix}
x_{ln}\\
y_{ln}\\
z_{ln}
\end{bmatrix} 
```
