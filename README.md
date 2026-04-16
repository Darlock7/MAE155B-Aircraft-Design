# MAE 155B Aircraft Design Project

## Overview
This repository contains the MATLAB code for our MAE 155B aircraft design and analysis project.

## Structure
- `src/geometry` : geometry design functions
- `src/aerodynamics` : aerodynamic estimation and supporting models
- `src/propulsion` : propulsion analysis
- `src/sizing` : preliminary sizing scripts
- `src/economics` : optimization and economic analysis
- `plotting` : plotting utilities
- `tests` : test scripts
- `data/reference` : reference input data
- `external/xfoil` : external XFOIL tools kept out of version control for now

## Setup
1. Clone the repository
2. Open the folder in MATLAB
3. Run `run_project.m` to load all paths

## Collaboration Workflow
1. Pull latest changes before starting work
2. Create a new branch for each feature or fix
3. Commit changes with clear messages
4. Push branch to GitHub
5. Open a Pull Request into `main`

## Notes
- Do not use MATLAB Drive as the source of truth
- Do not commit generated XFOIL outputs or conflict copies
- Keep `main` stable
