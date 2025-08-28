&nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp;  &nbsp;  <img src="https://github.com/ansfl/MEMS-IMU-Denoising/blob/de1832c6fc4e07936bd36e13a5fab6fe5dfc78d3/figrues/Logo.png" />


### Introduction
Quadrotors are indispensable in civilian, industrial, and military domains, undertaking complex, high-precision tasks once reserved for specialized systems. Across all contexts, energy efficiency remains a critical constraint: quadrotors must reconcile the high power demands of agility with the minimal consumption required for extended endurance.

Meeting this trade-off calls for mode-specific optimization frameworks that adapt to diverse mission profiles. At their core lie optimal control policies defining error functions whose minimization yields robust, mission-tailored performance. While solutions are straightforward for fixed weight matrices, selecting those weights is a far greater challenge—lacking analytical guidance and thus relying on exhaustive or stochastic search. This interdependence can be framed as a bi-level optimization problem, with the outer loop determining weights a priori.

This work introduces an aerial-enabled robust optimization for LQG tuning (AERO-LQG), a framework employing evolutionary strategy to fine-tune LQG weighting parameters. Applied to the linearized hovering mode of quadrotor flight, AERO-LQG achieves performance gains of several tens of percent, underscoring its potential for enabling high-performance, energy-efficient quadrotor control.

&nbsp; &nbsp;  &nbsp; &nbsp; &nbsp; &nbsp;  &nbsp; &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp; 
<img src="https://github.com/ansfl/C-ZUPT/blob/main/data/GIF-Bird-2.gif?raw=true" width="700" class='center'/>


&nbsp; &nbsp;  &nbsp; &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp; 
<img src="https://github.com/ansfl/C-ZUPT/blob/main/data/Fig_Dynamics.png?raw=true" width="1050" class='center'/>


### Simulation Results

To begin, we demonstrate the functionality of the LQG framework tailored to quadrotor dynamics, focusing on its ability to correct deviations caused by non-equilibrium initial conditions ($\boldsymbol{x}_0 \neq \boldsymbol{x}_e$) and process noise simulating wind. In this baseline scenario, the position update frequency is set equal to the prediction rate ($\Gamma = 1$), as shown in the state trajectories below, observed over a ten-second interval:

&nbsp; &nbsp;  &nbsp; &nbsp; &nbsp; &nbsp;  &nbsp; &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp; 
<img src="https://github.com/ansfl/C-ZUPT/blob/main/data/Fig_States_1.png?raw=true" width="600" class='center'/>

To understand the stabilization mechanism coordinating all four control inputs, the left side of the figure shows the LQR input commands as solid lines, with dashed brown lines representing the actual outputs, demonstrating minimal phase lag. The desired setpoint, corresponding to the hovering equilibrium, is marked by a black dashed line, with steady-state reference values for thrust (top) and roll, pitch, and yaw torques. On the right, rotor speeds commanded by the low-level speed controller are calculated by inverting the control outputs via the mixer matrix. Steady-state hover RPM is indicated by black dashed lines, with positive values denoting counter-clockwise rotation.

&nbsp; &nbsp;  &nbsp; &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp; 
<img src="https://github.com/ansfl/C-ZUPT/blob/main/data/Fig_Inputs.png?raw=true" width="1050" class='center'/>

Lastly, to assess the contribution of the proposed C-ZUPT mechanism, the figure below presents a head-to-head comparison with previous methods, using the same error analysis framework as in earlier evaluations.

&nbsp; &nbsp;  &nbsp; &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp; &nbsp;  &nbsp;  &nbsp; &nbsp;  &nbsp;  &nbsp;  &nbsp; &nbsp; 
<img src="https://github.com/ansfl/C-ZUPT/blob/main/data/Fig_ZUPT_err_1.png?raw=true" width="550" class='center'/>

As shown, with infrequent updates, the accuracy of estimation rapidly degrades due to inertial sensor noise and the platform's inherently erratic dynamics. This is particularly evident in the unaided case (blue), where errors grow unchecked, requiring aggressive control actions. In contrast, the C-ZUPT cases (pale green) consistently maintain smaller error magnitudes by leveraging detected stationary periods to provide informative corrections.
 
## Code

The code can be implemented using MATLAB R2022b or any later releases, and is organized as follows :

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

The authors would appreciate users giving stars to this repository and citing our article as follows:
```
@article{engelsman2025c,
  title={C-ZUPT: Stationarity-Aided Aerial Hovering},
  author={Engelsman, Daniel and Klein, Itzik},
  journal={arXiv preprint arXiv:2507.09344},
  year={2025}
}
```

[<img src=https://upload.wikimedia.org/wikipedia/commons/thumb/a/a8/ArXiv_web.svg/250px-ArXiv_web.svg.png width=70/>](https://arxiv.org/abs/2507.09344)
