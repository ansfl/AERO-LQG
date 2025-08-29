<p align="center">
  <img src="https://github.com/ansfl/MEMS-IMU-Denoising/blob/de1832c6fc4e07936bd36e13a5fab6fe5dfc78d3/figrues/Logo.png" alt="Project logo" />
</p>

# AERO-LQG

### Introduction
Quadrotors are indispensable across civilian, industrial, and defense domains, taking on complex, high-precision tasks once reserved for specialized systems. In all contexts, **energy efficiency** is a hard constraint: vehicles must reconcile the high power demands of agility with the minimal consumption required for **extended** endurance.

<p align="center">
  <img src="https://github.com/ansfl/C-ZUPT/blob/main/data/GIF-Bird-2.gif?raw=true" width="600" alt="Quadrotor GIF" />
</p>

Meeting this trade-off calls for mode-specific optimization frameworks that adapt to diverse mission profiles. At their core lie optimal control policies with error functions whose minimization yields robust, mission-tailored behavior. While solutions are straightforward for fixed weight matrices, **choosing** those weights is far harder—there is little analytical guidance, so practitioners rely on exhaustive or stochastic search. This interdependence can be posed as a **bi-level optimization** problem, with the outer loop selecting weights a priori.

<p align="center">
  <img src="https://github.com/ansfl/AERO-LQG/blob/main/data/Fig_system.png?raw=true" width="550" alt="Dynamics overview" />
</p>

This work introduces **AERO-LQG** (Aerial-Enabled Robust Optimization for LQG tuning), a framework that uses evolutionary strategies to fine-tune LQG weighting parameters. Applied to the linearized hovering mode of quadrotor flight, AERO-LQG yields performance gains of several tens of percent, highlighting its potential for high-performance, energy-efficient control.

### What makes this work special?
We expose the optimization mechanism itself—illustrating how covariance tuning evolves from random initialization toward a diagonal-centered structure:

<p align="center">
  <img src="https://github.com/ansfl/AERO-LQG/blob/main/data/Fig_Covariance_1.png?raw=true" width="1050" alt="Covariance evolution" />
</p>

Interpolated across time, the effect becomes striking:

<p align="center">
  <img src="https://github.com/ansfl/AERO-LQG/blob/main/data/Interpolation.gif?raw=true" width="500" alt="Interpolation animation" />
</p>

Thereby shedding light on how optimization methods naturally suppress off-diagonal entries, revealing—almost incidentally—that cross-correlations between different state variables are weak.

### Simulation results
We first demonstrate an LQG controller tailored to quadrotor dynamics. It corrects deviations induced by non-equilibrium initial conditions (\( \boldsymbol{x}_0 \neq \boldsymbol{x}_e \)) and process noise (wind). In this baseline scenario, the position update frequency equals the prediction rate (\( \Gamma = 1 \)). State trajectories over a 10-second interval are shown below:

<p align="center">
  <img src="https://github.com/ansfl/AERO-LQG/blob/main/data/Fig_Instability.png?raw=true" width="500" alt="State trajectories" />
</p>

To clarify the stabilization mechanism coordinating all four control inputs: on the left, solid lines show the LQR commands; dashed brown lines show the realized outputs with minimal phase lag. The dashed black line marks the steady-state hover setpoint (thrust at top; roll, pitch, yaw torques below). On the right, rotor speeds from the low-level speed controller are obtained by inverting the mixer matrix. Dashed black lines indicate steady-state hover RPM; positive values denote counter-clockwise rotation.

<p align="center">
  <img src="https://github.com/ansfl/AERO-LQG/blob/main/data/Fig_Stability.png?raw=true" width="500" alt="Stability and actuator behavior" />
</p>

## Code
The code runs on **MATLAB R2022b or later** and requires:

- Global Optimization Toolbox  
- Statistics and Machine Learning Toolbox

### Directory tree
