# Motor Selection

## Objective:

Select motor type for joint actuation. Overview of possibilities under cost and performance constrains.


## Motor Requirements:

- Direct drive, a custom reduction system will be designed.
- Capability for external torque control via microcontroller (required for model-based control)
- Low-cost solution for initial prototype (V1)


## Motor Considerations:

- Stepper motor
- Brushless DC motor (BLDC)
- Servomotor


## Stepper Motor

- Typically open loop. Closed loop possibility if encoder amd FOC controller is used.
- High positional resolution (especially with microstepping)
- High torque at low speed. Torque decreases with rotational speed. If a highter toque is desired, a higher supply voltage is needed.
- Cheap, if FOC isn't used.
- Heavy.
  

## BLDC Motor

- Closed loop.
- High efficiency.
- Encoder required, for this application.
- FOC controller required, for this application.
- High speed.
- Wider usable speed range with better torque retention than stepper
- Expensive, due to FOC controller.
- Light.


## Servo Motor (Hobby Grade)

- Cheap.
- Noisy.
- Small.
- Usually limited rotation range.
- No direct external torque control.
- No direct drive.
- Precision depends on quality of the reductions system.


## Conclusion for Design

- Stepper motors are the most suitable option for a first prototype due to low cost and simplicity.
- BLDC motors provide superior performance and control but are not viable within current budget constraints.
- Hobby servos are not suitable due to lack of torque control and limited flexibility.
- A hybrid approach (stepper + BLDC) may be considered depending on joint requirements and budget allocation.


## Context Note

A reduced 3-axis architecture is considered for the first prototype (V1), which may influence motor selection due to budget increase.

This will be revisited in future iterations (V2+) where full use of BLDC motors may be implemented.
