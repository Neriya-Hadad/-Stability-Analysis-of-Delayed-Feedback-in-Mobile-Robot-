# Stability Analysis of Delayed Feedback in Mobile Robot

This repository contains all materials for a simulation project investigating the stability of mobile robot trajectories under delayed feedback, based on delay differential equations (DDEs).

## Contents

- `Stability Analysis of Delayed Feedback in Mobile Robot.pptx`  
  Project presentation summarizing the objectives, approach, main results, and practical conclusions.

- `ddefun_robot.m`  
  MATLAB function that defines the system's delay differential equations (DDEs) as presented in the referenced article.

- `robot_dde_stability.m`  
  MATLAB script for running simulations using the `dde23` solver, varying the delay parameter (τ) to examine the transition from stable to unstable behavior.

## Project Overview

The project numerically solves the robot's equations of motion with delayed feedback using MATLAB's `dde23` solver. Through systematic simulations, we validate theoretical stability results from the referenced research and pinpoint the critical time-delay threshold for instability.

### Main Contributions

- **Practical implementation** of the theoretical DDE model in MATLAB.
- **Mapping stability transitions** by gradually varying the delay parameter.
- **Bridging theory and simulation** for real-world application in robotics and automation.

### Practical Applications

- Autonomous ground robots with delayed sensor feedback.
- Drone-to-ground coordination with wireless latency.
- Industrial automation and teleoperated vehicles (e.g., search & rescue).

## Reference

> Domoshnitsky, A., Kupervasser, O., Kutomanov, H., & Yavich, R.  
> *A Method for Stabilization of Ground Robot Path Controlled by Airborne Autopilot with Time Delay*  
> Springer Proceedings in Mathematics & Statistics, Vol. 379, FDEA 2019, Ariel University, Israel.

---

Feel free to use, adapt, or expand these files for related research or applications.
