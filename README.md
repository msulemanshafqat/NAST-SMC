# Neural-AS-NST-STSMC

[![License](https://img.shields.io/github/license/msulemanshafqat/Neural-AS-NST-STSMC?style=flat-square)](LICENSE)
[![GitHub stars](https://img.shields.io/github/stars/msulemanshafqat/Neural-AS-NST-STSMC?style=flat-square)](https://github.com/msulemanshafqat/Neural-AS-NST-STSMC/stargazers)
[![Last commit](https://img.shields.io/github/last-commit/msulemanshafqat/Neural-AS-NST-STSMC?style=flat-square)](https://github.com/msulemanshafqat/Neural-AS-NST-STSMC/commits/main)
[![Zenodo DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.20384523.svg)](https://doi.org/10.5281/zenodo.20384523)
[![Dryad DOI](https://img.shields.io/badge/DOI-10.5061%2Fdryad.t1g1jwtj4-blue?style=flat-square)](https://doi.org/10.5061/dryad.t1g1jwtj4)


A repository for simulation results, visualizations, and data supporting the published work on **Hybrid Neural Adaptive Smooth Non-Singular Terminal Super-Twisting Sliding Mode Control (NAST-SMC)** for a safety-critical multirotor UAV.

## Overview

This project documents the NAST-SMC framework applied to an octrotor UAV and reports extensive performance results across trajectory tracking, actuator delay, rotor fault, wind disturbance, uncertainty, and sensor-noise scenarios.

NAST-SMC combines:

* Nonsingular terminal sliding surface design
* Smooth continuous approximation of the signum function
* Hybrid offline PSO gain tuning and online PPO-based adaptation
* Robust performance under faults, delays, disturbances, and model uncertainty

## Highlights

* Accurate trajectory tracking across circle, square, spiral, random waypoint, and step inputs
* Low energy consumption while maintaining superior robustness
* Comparative performance against STSMC, FracFITSMC, DISMC, ISMC, and PD controllers

## Repository Contents

* `Code/` — MATLAB and Python scripts used for simulation, analysis, plotting, and DRL adaptation
* `Consolidated_Results.xlsx` — primary dataset with experiment metadata and performance metrics
* `Interactive Figures/` — interactive visualizations for key results
* `Static Figures/` — publication-ready plots and comparative figures
* `Results/` — consolidated results and analysis artifacts
* `Manuscript/` — related manuscript files

## Code Folder

The `Code/` folder includes:

* MATLAB experiment scripts: `Exp1_v5.m`, `Exp2_v4.m`, `Exp3.m`, `Exp4_v1.m`, `ExpAll_v1.m`
* Core dynamics and initialization: `octrotor_dynamics.m`, `Initialization.m`, `rotation_matrix.m`, `RT_matrix.m`
* Data processing and plotting utilities: `compile_results.m`, `compile_results_init.m`, `Visualization.m`, `Visualization_Comparison.m`, `export_fig.m`
* Supporting utilities: `copyfig.m`, `crop_borders.m`, `eps2pdf.m`, `pdf2eps.m`, `pdftops.m`, `pd_from_freq.m`, `print2array.m`, `print2eps.m`, `fix_lines.m`, `read_write_entire_textfile.m`
* DRL training: `drl_neural_adaptive_training.py`

## Experimental Setup

* **Vehicle:** Octrotor UAV, mass = 6.8 kg, inertia `J = diag([0.34 0.34 0.65]) kg·m²`
* **Simulation:** MATLAB R2025a, fixed-step 0.001 s to correspond 1kHz controller update rate
* **Trajectories:** circle, square, spiral, random waypoints, unit step
* **Scenarios:** nominal operations, rotor faults, actuator delays, wind gusts, torque disturbances, parameter uncertainty, sensor noise, and mixed conditions
* **Metrics:** position RMSE, attitude RMSE, overshoot, energy consumption, SMAPE, ISE, IAE, ITAE, ITSE, composite score

## Usage

1. Open the repository in MATLAB or your preferred analysis environment.
2. Review `Consolidated_Results.xlsx` for the full experimental dataset.
3. Use the files in `Interactive Figures/` for dynamic result exploration.
4. Use the plots in `Static Figures/` for publication-quality visuals.

## Notes

* This repository is intended to accompany the published dataset and preprint.
* Data are simulation-generated and reflect the results reported in the associated study.

## Access

* **GitHub Repository:** https://github.com/msulemanshafqat/Neural-AS-NST-STSMC
* **Dryad Dataset:** https://doi.org/10.5061/dryad.t1g1jwtj4
* **Zenodo Preprint:** https://doi.org/10.5281/zenodo.20384523

## Citation

Please cite both the dataset and the preprint:

> Shafqat, M. S., & Ahmad, I. (2026). Hybrid Neural Adaptive Smooth Non-Singular Terminal Super-Twisting Sliding Mode Control of Multirotor UAV for Safety-Critical Applications [Preprint]. Zenodo. https://doi.org/10.5281/zenodo.20384523

> Shafqat, M. S., & Ahmad, I. (2026). Hybrid Neural Adaptive Smooth Non-Singular Terminal Super-Twisting Sliding Mode Control of Multirotor UAV – Simulation Results. Dryad Dataset. https://doi.org/10.5061/dryad.t1g1jwtj4

## Contact

Corresponding author: Muhammad Suleman Shafqat