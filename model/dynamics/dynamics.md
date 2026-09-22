# Dynamics Model

## Objective:

Determine basic dynamics model for robot.


## Requirements:

- Modular design, capability of moving objects (motors, joints, links, ...) and adding DOFs easily.


## Architecture

### Newton-Euler method

The Newton-Euler method will be used to define the dynamic model.

In this method, dynamic equations are written for each link.

It is best for real time dynamics calculation and model control, as equations are evaluated in a recursive and numeric way.

### Newton-Euler equation definition

All equations are derived from the following principles:

#### Derivative of a vector in a moving frame

Using Newton-Euler the following equations can be defined for any link i:
```math
\vec{F_{i-1,i}}-\vec{F_{i,i+1}}+m_i\vec{g}-m_i\vec{a_{Ci}}=\vec{0}
``` 
Where:
F_{i-1,i}: force effected by link i-1 into link i
F_{i,i+1}: force effected by link i into link i+1
g: gravity
m_i: link total mass
a_{ci}: acceleration of the i link center of mass

```math
\vec{N_{i-1,i}}-\vec{N_{i,i+1}}-(\vec{r_{i-1,i}}+\vec{r_{i,Ci}}) \times \vec{F_{i-1,i}} + (-\vec{r_{i,Ci}}) \times (-\vec{F_{i,i+1}})-I_i\dot{\vec{w_i}}-\vec{w_i} \times (I_i\vec{w_i})
``` 
Where:
F_{i-1,i}: force effected by link i-1 into link i
F_{i,i+1}: force effected by link i into link i+1
r_{i-1,i}: vector representing i link
r_{i,Ci}: vector from end of link i to its center of mass
g: gravity
m_i: link total mass
x_{ci}: position of the i link center of mass
N_{i-1,i}: torque effected by link i-1 into link i (at joints)
N_{i,i+1}: torque effected by link i into link i+1 (at joints)
w_i: link angular velocity
I_i: link inertia matrix

### Inerta Matrix

### Calculation Algorithm

For any i link in a robot consisting on n links and given:

- I_i 
- m_i
- r_{i-1,i}
- r_{i,Ci}
- g
- m_i
- a_{ci}

from link 0 to n:

- a_i calculation
- w_i calculation

from link n to 0, using Newto-Euler equations:

- F_i calculation
- N_i calculation



### Learning resources

1. University lecture
https://ocw.mit.edu/courses/2-12-introduction-to-robotics-fall-2005/c7caaa2376b8ec01e270328a3b80b029_chapter7.pdf

2. University lecture
https://www.diag.uniroma1.it/deluca/rob2_en/06_NewtonEulerDynamics.pdf

