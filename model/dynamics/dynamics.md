# Dynamics Model

## Objective:

Determine basic dynamics model for robot. 

Taking into account requirements stated in model\introduction.md.


## Newton-Euler method

The Newton-Euler method will be used to define the dynamic model.

In this method, dynamic equations are written for each link.

It is best for real time dynamics calculation and model control, as equations are evaluated in a recursive and numeric way.


## Basic principles

All equations are derived from the following principles:

### Derivative of a vector in a moving frame

```math
{}^{n}a_i={}^{n}\dot{v_i}+{}^{n}w_i\times{}^{n}v_i
``` 

### Newton dynamic equation

```math
\sum f_i=\frac{d}{dt}(mv_c)=m\dot v_c
``` 

### Euler dynamic equation

```math
\sum \tau _i=I\dot w+w\times Iw
``` 

### Action reaction law

Forces applied by body i to i+1 = -applied by body i+1 to body i.


## Newton-Euler equations definition

Using the previous principles the following equations can be defined for \(L_i\), defined in \(RF_n\).

```math
{}^{n}f_{i}-{}^{n}f_{i+1}+m_i{}^{n}g=m_i{}^{n}a_{ci}
``` 

```math
{}^{n}\tau _{i}-{}^{n}\tau _{i+1}+{}^{n}f_{i}\times {}^{n}r_{i-1,ci}-{}^{n}f_{i+1}\times {}^{n}r_{i,ci}=I_i{}^{n}\dot w_i+{}^{n}w_i\times (I_i{}^{n}w_i)
``` 

Where:

\(a_{ci}\): \(L_i\) center of mass acceleration.

\(f_{i}\): force exerced by \(L_{i-1}\) to \(L_i\).

\(f_{i+1}\): force exerced by \(L_i\) to \(L_{i+1}\).

\(\tau _{i}\): torque exerced by \(L_{i-1}\) to \(L_i\).

\(\tau _{i+1}\): torque exerced by \(L_i\) to \(L_{i+1}\).

\(r_{i-1,Ci}\): vector from \(R_{i-1}\) center to \(L_i\) center of mass.

\(r_{i,Ci}\): vector from \(R_{i}\) center to \(L_i\) center of mass.


## Calculation Algorithm

For a robot consisting of n links.

### Forward Recursion

The following ecuations are evaluated from i=0 to i=n.
```math
{}^{i}w_i={}^{i-1}R_{i}^T*[{}^{i-1}w_{i-1}+\dot q_i{}^{i-1}z_{i-1}]
``` 
Initialized by \({}^{0}w_0\).
```math
{}^{i}\dot w_i={}^{i-1}R_{i}^T*[{}^{i-1}\dot w_{i-1}+\ddot q_i{}^{i-1}z_{i-1}+\dot q_i{}^{i-1}w_{i-1}\times {}^{i-1}z_{i-1}]
``` 
Initialized by \({}^{0}\dot w_0\).
```math
{}^{i}a_i={}^{i-1}R_{i}^T*{}^{i-1}a_{i-1}+{}^{i}\dot w_{i}\times {}^{i}r_{i-1,i}+{}^{i}w_i\times ({}^{i}w_i\times {}^{i}r_{i-1,i})
``` 
Initialized by \({}^{0}a_0-{}^{0}g\).
```math
{}^{i}a_{ci}={}^{i}a_i+{}^{i}\dot w_{i}\times {}^{i}r_{i,ci}+w_i\times (w_i\times {}^{i}r_{i,ci})
``` 

Where:

\({}^{i-1}R_{i}^T\): rotational matrix assigned to \(J_i\) (\(R_x\), \(R_y\) or \(R_z\)).

\({}^{i-1}z_{i-1}\): unit vector representing the rotational axis of \(J_i\).

### Backward Recursion

The following ecuations are evaluated from i=n to i=0.
```math
{}^{i}f_{i}={}^{i+1}f_{i+1}+m_i{}^{i}a_{ci}
``` 
Initializated by external force applied to \(L_n\) tip.
```math
{}^{i}\tau _{i}={}^{i+1}\tau _{i+1}-{}^{i}f_{i}\times {}^{i}r_{i-1,ci}+{}^{i+1}f_{i+1}\times {}^{i}r_{i,ci}+I_i{}^{i}\dot w_i+{}^{i}w_i\times (I_i{}^{i}w_i)
``` 
Initializated by external torque applied to \(L_n\) center of mass.


## Inerta Matrix




### Learning resources

1. University lecture
https://ocw.mit.edu/courses/2-12-introduction-to-robotics-fall-2005/c7caaa2376b8ec01e270328a3b80b029_chapter7.pdf

2. University lecture
https://www.diag.uniroma1.it/deluca/rob2_en/06_NewtonEulerDynamics.pdf

