<p align="center">
  <img src="https://github.com/ansfl/MEMS-IMU-Denoising/blob/de1832c6fc4e07936bd36e13a5fab6fe5dfc78d3/figrues/Logo.png"  width="600" alt="Project logo" />
</p>

# AERO-LQG

### Introduction
Quadrotors are indispensable across civilian, industrial, and defense domains, taking on complex, high-precision tasks once reserved for specialized systems. In all contexts, **energy efficiency** is a hard constraint: vehicles must reconcile the high power demands of agility with the minimal consumption required for **extended** endurance.

<p align="center">
  <img src="https://github.com/ansfl/C-ZUPT/blob/main/data/GIF-Bird-2.gif?raw=true" width="500" alt="Quadrotor GIF" />
</p>

Meeting this trade-off calls for mode-specific optimization frameworks that adapt to diverse mission profiles. At their core lie optimal control policies with error functions whose minimization yields robust, mission-tailored behavior. While solutions are straightforward for fixed weight matrices, **choosing** those weights is far harder—there is little analytical guidance, so practitioners rely on exhaustive or stochastic search. This interdependence can be posed as a **bi-level optimization** problem, with the outer loop selecting weights a priori.

<p align="center">
  <img src="https://github.com/ansfl/AERO-LQG/blob/main/data/Fig_system.png?raw=true" width="450" alt="Dynamics overview" />
</p>

This work introduces **AERO-LQG** (Aerial-Enabled Robust Optimization for LQG tuning), a framework that uses evolutionary strategies to fine-tune LQG weighting parameters. Applied to the linearized hovering mode of quadrotor flight, AERO-LQG yields performance gains of several tens of percent, highlighting its potential for high-performance, energy-efficient control.

### What makes this work special?
We expose the optimization mechanism itself—illustrating how covariance tuning evolves from random initialization toward a diagonal-centered structure:

<p align="center">
  <img src="https://github.com/ansfl/AERO-LQG/blob/main/data/Fig_Covariance_1.png?raw=true" width="1050" alt="Covariance evolution" />
</p>

Interpolated across time, the effect becomes striking:

<p align="center">
  <img src="https://github.com/ansfl/AERO-LQG/blob/main/data/Interpolation.gif?raw=true" width="400" alt="Interpolation animation" />
</p>

Thereby shedding light on how optimization methods naturally suppress off-diagonal entries, revealing—almost incidentally—that cross-correlations between different state variables are weak.

### Simulation Results

We first examine the time evolution of the quadrotor state variables under standardized gust disturbances, using either manual tuning or purely heuristic approaches. As shown, deviations from the hovering equilibrium quickly amplify across translational and rotational channels, with coupled oscillations emerging between roll, pitch, and yaw. The **estimator fails** to correct these growing errors, and the feedback loop **cannot suppress** the accumulating drift, ultimately causing **divergence** in both position and attitude. This outcome highlights the fragility of baseline configurations when the weighting matrices remain unoptimized. State trajectories over a 10-second interval are presented below:

<p align="center">
  <img src="https://github.com/ansfl/AERO-LQG/blob/main/data/Fig_Instability.png?raw=true" width="450" alt="State trajectories under instability" />
</p>

In contrast, the second figure shows the closed-loop response once the **optimization framework** is applied. Under the same gust inputs, deviations are **limited** to short-lived transients that are rapidly attenuated by the tuned controller. State variables **converge** smoothly back to their equilibrium values, with cross-axis coupling significantly reduced. The estimator and controller act in concert, maintaining bounded errors and restoring **stable hover**. These results demonstrate the ability of the optimization-based approach to enforce robustness against disturbances while preserving control efficiency.

<p align="center">
  <img src="https://github.com/ansfl/AERO-LQG/blob/main/data/Fig_Stability.png?raw=true" width="450" alt="Stability and actuator behavior" />
</p>


## Code
The code runs on **MATLAB R2022b or later** and requires:

- Global Optimization Toolbox  
- Statistics and Machine Learning Toolbox

### Directory tree
```

[root]
├─ code
│  ├─ main.m
│  ├─ Linearize_Quad.m
│  ├─ f_Runga_Kutta.m
│  ├─ System_Parameters.m
│  └─ u_saturation.m
├─ data
└─ requirements.txt

```

| File                | Purpose                                                            |
|---------------------|--------------------------------------------------------------------|
| `main.m`            | Entry point                                                        |
| `Linearize_Quad.m`  | Linearization; state and input Jacobians                           |
| `f_Runga_Kutta.m`   | 4th-order Runge–Kutta integrator for \( f(x_k,u_k) \)              |
| `System_Parameters.m` | Load physical and mixer parameters                               |
| `u_saturation.m`    | Enforce actuator limits                                            |

## Citation
This work follows our earlier study on LQG optimization, currently under advanced review in a leading aerospace venue:

```

@article{engelsman2025c,
title   = {C-ZUPT: Stationarity-Aided Aerial Hovering},
author  = {Engelsman, Daniel and Klein, Itzik},
journal = {arXiv preprint arXiv:2507.09344},
year    = {2025}
}

```

When using this repository, please star it and cite:

```

@article{engelsman2025aero,
title   = {AERO-LQG: Aerial-Enabled Robust Optimization for LQG-Based Quadrotor Flight Controller},
author  = {Engelsman, Daniel and Klein, Itzik},
journal = {arXiv preprint arXiv:2508.20888},
year    = {2025}
}

```

[<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/a/a8/ArXiv_web.svg/250px-ArXiv_web.svg.png" width="70" alt="arXiv">](https://arxiv.org/abs/2508.20888)
```
