**# README**

**## Objective**

Simulate, design, and build a 6-DOF robotic arm controlled by a microcontroller.

Capable of:

- Moving precisely, using defined trajectories
- Having a maximum carryinge capability of 1kg with a full extended arm
- Capable of grabbing objects

The system will use:

- A simplified dynamic model, written in Python or Matlab
- Closed loop control for better precision
- Parametric and modular design to ease modification and iterations


**## Scope**

- Complete mechanical design
- BLDC motors
- Bought motor controllers
- Python or Matlab complete simulation
- STM32 implementation
- Experimental validation


**## Restrictions**

- Manufacturing:
    - 3D printed parts (except: screws, axles, bearings, electronics, motors, etc...)
    - Maximum area of printing: 220 × 220 × 250 mm (Ender 3 V1)
  
- Budget:
  - Maximum: 300€
  
- Hardware:
    - Controller: STM32
    - Motors: BLDC
    - Motor controllers: (to be selected)
  
- Geometry:
    - Maximum part length: ~300 mm (diagonally printed)


**## Current Status**

Project in early design phase.

- System architecture defined
- Requirements defined
- Trade studies not yet started
- Mechanical design not started


**## Design Approach**

The system is developed using an iterative engineering workflow:

1. Define requirements
2. Perform trade studies
3. Define system architecture
4. Build initial design baseline
5. Develop mathematical model
6. Validate through simulation and experiments