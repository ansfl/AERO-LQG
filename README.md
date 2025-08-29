&nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp;  &nbsp;  <img src="https://github.com/ansfl/MEMS-IMU-Denoising/blob/de1832c6fc4e07936bd36e13a5fab6fe5dfc78d3/figrues/Logo.png" />


### Introduction
Quadrotors are indispensable in civilian, industrial, and military domains, undertaking complex, high-precision tasks once reserved for specialized systems. Across all contexts, energy efficiency remains a critical constraint: quadrotors must reconcile the high power demands of agility with the minimal consumption required for EXTENDED aerial endurance.

&nbsp; &nbsp;  &nbsp; &nbsp; &nbsp; &nbsp;  &nbsp; &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp; 
<img src="https://github.com/ansfl/C-ZUPT/blob/main/data/GIF-Bird-2.gif?raw=true" width="600" class='center'/>

Meeting this trade-off calls for mode-specific optimization frameworks that adapt to diverse mission profiles. At their core lie optimal control policies defining error functions whose minimization yields robust, mission-tailored performance. While solutions are straightforward for fixed weight matrices, selecting those weights is a far greater challenge—lacking analytical guidance and thus relying on exhaustive or stochastic search. This interdependence can be framed as a bi-level optimization problem, with the outer loop determining weights a priori.

&nbsp; &nbsp;  &nbsp; &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp; 
<img src="https://github.com/ansfl/C-ZUPT/blob/main/data/Fig_Dynamics.png?raw=true" width="1050" class='center'/>

This work introduces an aerial-enabled robust optimization for LQG tuning (AERO-LQG), a framework employing evolutionary strategy to fine-tune LQG weighting parameters. Applied to the linearized hovering mode of quadrotor flight, AERO-LQG achieves performance gains of several tens of percent, underscoring its potential for enabling high-performance, energy-efficient quadrotor control.

### What makes this work special ??

This work enables a sneak peek into the optimization mechanism, showing us how the covariance tuning behaves in the eyes of the optimizer, starting from 

&nbsp; &nbsp;  &nbsp; &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp; 
<img src="https://github.com/ansfl/AERO-LQG/blob/main/data/Fig_Covariance_1.png?raw=true" width="1050" class='center'/>

when interpolated across the time axis, the results can be indeed mind blowing : 

&nbsp; &nbsp;  &nbsp; &nbsp; &nbsp; &nbsp;  &nbsp; &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp; 
<img src="https://github.com/ansfl/AERO-LQG/blob/main/data/Interpolation.gif?raw=true" width="500" class='center'/>

### Simulation Results

To begin, we demonstrate the functionality of the LQG framework tailored to quadrotor dynamics, focusing on its ability to correct deviations caused by non-equilibrium initial conditions ($\boldsymbol{x}_0 \neq \boldsymbol{x}_e$) and process noise simulating wind. In this baseline scenario, the position update frequency is set equal to the prediction rate ($\Gamma = 1$), as shown in the state trajectories below, observed over a ten-second interval:

&nbsp; &nbsp;  &nbsp; &nbsp; &nbsp; &nbsp;  &nbsp; &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp; 
<img src="https://github.com/ansfl/AERO-LQG/blob/main/data/Fig_Instability.png?raw=true" width="600" class='center'/>

To understand the stabilization mechanism coordinating all four control inputs, the left side of the figure shows the LQR input commands as solid lines, with dashed brown lines representing the actual outputs, demonstrating minimal phase lag. The desired setpoint, corresponding to the hovering equilibrium, is marked by a black dashed line, with steady-state reference values for thrust (top) and roll, pitch, and yaw torques. On the right, rotor speeds commanded by the low-level speed controller are calculated by inverting the control outputs via the mixer matrix. Steady-state hover RPM is indicated by black dashed lines, with positive values denoting counter-clockwise rotation.

&nbsp; &nbsp;  &nbsp; &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp; 
<img src="https://github.com/ansfl/AERO-LQGblob/main/data/Fig_Stability.png?raw=true" width="1050" class='center'/>

 
## Code

The code can be implemented using MATLAB R2022b or any later releases, with strict REQUIREMENT of two packeges installed : 
Global Optimization Toolbox, and Statistics and Machine Learning Toolbox, and is organized as follows :

### Directory tree
<pre>
[root directory]
├── code
|   ├── main.m
|   ├── Linearize_Quad.m
|   ├── f_Runga_Kutta.m
|   ├── System_Parameters.m
    ...
|   ├── 
    └── u_saturation.m
├── data
...
└── requirements.txt
<!--  Readme.md -->
</pre>

File | Purpose
--- | --- 
**main** | Main Launcher file
**Linearize_Quad** | Linearization and computation of state and control jacobians
**f_Runga_Kutta** | 4-th order numerical solver for **f(x_k,u_k)**
**System_Parameters** | Upload all relevant system parameters
**u_saturation** | Actuation physical limitations


## Citation

This work is a follow-up to our previous study, where LQG optimization was thoroughly examined and analyzed, and is currently under advanced review in a leading aerospace journal :
```
@article{engelsman2025c,
  title={C-ZUPT: Stationarity-Aided Aerial Hovering},
  author={Engelsman, Daniel and Klein, Itzik},
  journal={arXiv preprint arXiv:2507.09344},
  year={2025}
}
```
When using this repository, we kindly ask users to star it and cite our article as follows :
```
@article{engelsman2025aero,
  title={AERO-LQG: Aerial-Enabled Robust Optimization for LQG-Based Quadrotor Flight Controller},
  author={Engelsman, Daniel and Klein, Itzik},
  journal={arXiv preprint arXiv:2507.09344},
  year={2025}
}
```
[<img src=https://upload.wikimedia.org/wikipedia/commons/thumb/a/a8/ArXiv_web.svg/250px-ArXiv_web.svg.png width=70/>](https://arxiv.org/abs/2508.20888)
