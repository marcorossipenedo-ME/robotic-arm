# Direct and Inverse Jacobian Model Simulation and Validation Results

- Tested multiple initial points in space
- Inverse Jacobian calculated end effector velocity visualization is the desired.
- Direct Jacobian calculated end effector velocity visualization is the desired.

Numerical end effector velocity error, in non-singulrity points, remains in the order of:
```math
10^[-3]
```
When aproaching singularity regions, output can be erratic and error is increased. When designing the real arm motion program, care should be taken to avoid entering these regions. 

This validates the correctness of the direct and inverse Jacobian model.