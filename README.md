# Cartesian Space Control and Inverse Kinematics of a 7-DOF Manipulator

## Project Overview
This repository provides an advanced MATLAB-based framework for the closed-loop kinematic control of a 7-DOF robotic manipulator operating in Cartesian space. The system calculates the exact joint velocities required to drive a custom Tool Center Point (TCP) to a predefined goal pose using the Moore-Penrose pseudoinverse of the analytically derived Tool Jacobian matrix.

## Mathematical Formulation

### 1. Tool Frame Kinematics
The Forward Kinematics model calculates the pose of the manipulator's flange relative to the base frame, denoted as $^0T_f(q)$. The actual Tool frame is rigidly attached to the flange with a constant transformation offset $^fT_t$. The tool pose in the base frame is given by:

$$^0T_t(q) = {^0T_f(q)} \cdot {^fT_t}$$

### 2. Differential Kinematics and Jacobian Shifting
To control the Tool frame rather than the flange, the Flange Jacobian $J_f(q)$ must be mathematically shifted. Let $r = [r_x, r_y, r_z]^T$ be the offset vector from the flange to the tool origin, expressed in the base frame. The Rigid Body Jacobian (Shift Matrix) is utilized to compute the Tool Jacobian $J_t(q)$:

```math
J_t(q) = \begin{bmatrix} 
I_{3 \times 3} & 0_{3 \times 3} \\ 
-[r]_{\times} & I_{3 \times 3} 
\end{bmatrix} J_f(q)


Where $[r]_{\times}$ is the skew-symmetric matrix of the offset vector $r$. This transformation properly maps the operational space twist from the flange to the tool.

### 3. Cartesian Error Computation
The control objective is to drive the Cartesian pose error to zero.

**Linear Error ($e_p$):**

$$e_p = P_{goal} - P_{tool}$$

**Rotational Error ($e_o$):**
The relative rotation from the current tool frame to the goal frame is calculated as $R_{err} = R_{tool}^T R_{goal}$. This matrix is converted into an equivalent Angle-Axis representation $(h, \theta)$ to avoid representation singularities:

$$\theta = \cos^{-1}\left(\frac{\text{tr}(R_{err}) - 1}{2}\right)$$

The rotational error vector, projected back into the base frame, is:

$$e_o = R_{tool} (h \theta)$$

### 4. Generalized Kinematic Control Law
To track a desired Cartesian trajectory, the generalized closed-loop kinematic control law combines a feed-forward velocity term (to track moving targets) with a proportional feedback term (to correct drift and initial offsets):

$$\dot{x}_{ref} = \dot{x}_{des} + K \cdot e$$

Where:
- $\dot{x}_{des} \in \mathbb{R}^6$ is the **Feed-forward term** (desired velocity of the goal). 
- $e = [e_o^T, e_p^T]^T$ is the combined Cartesian error vector.
- $K = \text{diag}(K_a, K_l)$ is the positive definite gain matrix.

*Note on Implementation:* In this specific simulation, since the control objective is a fixed posture regulation problem (static goal), the feed-forward term is zero ($\dot{x}_{des} = 0$). Thus, the control law simplifies to pure proportional feedback:

$$\dot{x}_{ref} = \begin{bmatrix} K_a e_o \\ K_l e_p \end{bmatrix}$$

### 5. Inverse Kinematics via Pseudoinverse
The required joint space velocities $\dot{q}$ are resolved using the Moore-Penrose pseudoinverse of the Tool Jacobian ($J_t^{\dagger}$) to natively handle kinematic redundancy (7-DOF):

$$\dot{q} = J_t^{\dagger} \dot{x}_{ref}$$

The joint positions are updated numerically via Euler integration, enforcing strict hardware joint limits $[q_{min}, q_{max}]$:

$$q_{k+1} = \max(q_{min}, \min(q_{max}, q_k + \dot{q} \Delta t))$$

## Software Architecture
The framework is rigorously modular:
- **`cartesianControl.m`**: Evaluates position and Angle-Axis rotational errors to compute the reference Cartesian twist.
- **`kinematicModel.m`**: Computes the Flange Jacobian and applies the Rigid Body Shift Matrix to derive the exact Tool Jacobian.
- **`geometricModel.m`**: Maintains $SE(3)$ configuration matrices and manages the $^fT_t$ offset.
- **`RotToAngleAxis.m` / `YPRToRot.m` / `IsRotationMatrix.m`**: Robust mathematical utilities for orientation mapping and validation.
- **`KinematicSimulation.m`**: Handles discrete-time integration and joint limit saturation.

## Simulation Results
The simulation validates the control architecture, demonstrating asymptotic convergence of both position and orientation errors to near-zero tolerance levels.

**Simulation Trajectory (3D)**

<img width="700" height="525" alt="MOTION OF THE MANIPULATOR" src="https://github.com/user-attachments/assets/6c73e195-eaf9-4217-8e72-40598821a94a" />

---

**Final Configuration**

<img width="700" height="525" alt="FINAL CONFIGURATION" src="https://github.com/user-attachments/assets/75519675-b0a6-421c-acfd-88fcd141e984" />

---

**End-Effector Velocity Profile**

<img width="700" height="525" alt="DIRECTION OF THE END-EFFECTOR VELOCITIES" src="https://github.com/user-attachments/assets/c9470be6-8c87-4abe-bda5-5a6beb79f6e1" />


## Execution Guide
1. Initialize the MATLAB environment and ensure the `include/` directory is in the active path.
2. Run `main.m` to start the closed-loop simulation. The script evaluates the Cartesian control iteratively and outputs final convergence metrics to the command window.

---
*Developed by Hussein Elnaggar.*
