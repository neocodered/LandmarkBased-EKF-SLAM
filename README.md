# Feature-Based EKF SLAM for Autonomous Racing

![Project Banner](Images/Round%20Track%20EKF.png)

## 📌 Project Overview
This repository contains a **MATLAB implementation of an Extended Kalman Filter (EKF) for Simultaneous Localization and Mapping (SLAM)**. The project simulates a differential drive robot navigating through track boundaries defined by discrete landmarks (traffic cones), inspired by **Formula Student Driverless** competitions.

The system addresses the "Chicken and Egg" problem of SLAM by simultaneously estimating the robot's pose $(x, y, \theta)$ and the Cartesian coordinates of the landmarks using a range-bearing sensor model.

## 📂 Repository Structure
The project is organized into modular MATLAB scripts and a resources folder:

```plaintext
├── Cone_EKF.mlx                  # MAIN SCRIPT: Entry point for simulation & visualization
├── diffDriveModel.m              # HELPER: Differential drive kinematic motion model
├── run_ekf_simulation.m          # CORE LOGIC: The EKF prediction-correction loop
├── calculate_simulation_error.m  # VALIDATION: RMSE calculation against Ground Truth
├── gen_corner.m                  # UTILITY: Geometry generator for track corners
├── Images/                       # Simulation results (Images & Videos)
│   ├── Round Track EKF.png
│   ├── Round Track.mp4
│   ├── Extended Round Track.png
│   ├── Extended Round Track.mp4
│   ├── Square Rounded Corner Track.png
│   └── Square Rounded Corner Track.mp4
└── README.md
