# LOG0001 - 2026-04-14

## Context
Initial phase of the robot arm project. No defined architecture existed yet.

## Change
Defined initial system architecture and project structure.
Established main requirements and constraints.

## Reason
To provide a structured foundation before starting design.

## Impact
Enables separation of concerns between requirements, modeling, hardware design, and validation.


# LOG0002 - 2026-04-15

## Context
Initial project setup phase. 

## Change
Redefined complete project architecture including:
- separation between research, design, architecture, and logs
- redefinition of structured engineering workflow

## Reason
To provide a structured foundation before starting design.

## Impact
All future design decisions will follow this structured workflow.
Enables traceability between requirements, design decisions, and validation.


# LOG0003 - 2026-04-16

## Context
Initial V1 design phase. 

## Change
- Motor options research for actuators.
- Redefined system requirements for V1.

## Reason
To assess viability of proposed system requirments for V1.

## Impact
All future design decisions will follow current system requirments.
Acclaration of motor options and possibility of deciding.


# LOG0004 - 2026-04-16

## Context
Initial V1 design phase. 

## Change
Reduction system options research for actuators.

## Reason
To assess viability of proposed system requirments for V1 and continue design process.

## Impact
Acclaration of reduction system options and possibility of deciding.


# LOG0005 - 2026-04-17

## Context
Initial V1 design phase. 

## Change
Started simple direct kinematics model. Defined architecture, parameters and started formula definition.

## Reason
To help determine requirments for motor, motor placement and reduction design process.

## Impact
Better future design decisions.


# LOG0006 - 2026-04-20

## Context
Initial V1 design phase. 

## Change
Finished simple direct kinematics model. 

## Reason
To help determine requirments for motor, motor placement and reduction design process.

## Impact
Better future design decisions.


# LOG0007 - 2026-04-21

## Context
Initial V1 design phase. 

## Change
Added 3DOF case into simple kinematics model.
Started inverse kinematics model. Defined each angle equation.

## Reason
To help determine requirments for motor, motor placement and reduction system design process.

## Impact
Better future design decisions.


# LOG0008 - 2026-04-22

## Context
Initial V1 design phase. 

## Change
Started kinematics model simulation coding for model testing.

## Reason
To help test direct and inverse kinematics model.

## Impact
Model improvements and verification.


# LOG0009 - 2026-04-22

## Context
Initial V1 design phase. 

## Change
In direct kinematics model, order of multiplication in A matrix definition was changed from A(j_n, L_{n-1}) =T(L_{n-1})*R_n(j_n)  to A(j_n, L_{n-1}) = R_n(j_n)*T(L_{n-1}).

## Reason
During validation of the kinematics implementation, it was observed that applying the transformation chain to the homogeneous origin vector produced no visible rotational effect. The issue was related to the use of the origin point ([0,0,0,1]^T), which is invariant under rotation.

## Impact
Model improvements and verification.


# LOG0010 - 2026-04-22

## Context
Initial V1 design phase. 

## Change
In direct kinematics model, the global matrix was changed. Thus the inverse kinematics equations had to be changed.

## Reason
During validation of the kinematics implementation, it was observed that zero angle position and positive angle direction were indetermined with previous configuration, where l0 was defined in z axis direction, l1 in y axis and l2 in y axis. Now all links are defined in z axis direcion, in this configuration, when all angles are zero, the arm is completelly vertical. In this position, positive joint angles make links rotate to a positive x position. Then if J0 is rotated +90º, the links located in a positive x position rotate to a positive y position.

## Impact
Model improvements and verification.


# LOG0011 - 2026-04-28

## Context
Initial V1 design phase. 

## Change
Direct and inverse kinematics model simulation finished and verification done.

## Reason
-

## Impact
3DOF direct and inverse kinematics model finished and validated. 


# LOG0012 - 2026-05-11

## Context
Initial V1 design phase. 

## Change
Started Jacobian definition and research for 3DOF robot arm. Finished basic mathematical formulation.

## Reason
Further development of robot control and kinematic model definition.

## Impact
Foundation for differential kinematics and velocity control established.