
# FRA333_HW3_xx_xx
จัดทำโดย นายคณพล กาจธัญกิจ 65340500005 และ นายอาคม สนธิขันธ์ 65340500077

This repository contains two main files for the assignment:

1. `FRA333_HW3_xx_xx.py` – Contains the functions developed for solving inverse kinematics problems and other related computations.
2. `testScript.py` – A script to test the functions implemented in `FRA333_HW3_xx_xx.py` and compare results with the `Robotic Toolbox`.

## Files and Functions

### 1. `FRA333_HW3_xx_xx.py`
This file contains three main functions used to solve the assignment:

- **`endEffectorJacobianHW3(q: list[float]) -> list[float]`**  
  Computes the Jacobian matrix for the end-effector given the joint angles `q`.
  - **Input**:  
    - `q`: List of joint angles (size 3 for a 3-DOF robot).
  - **Output**:  
    - A 6x3 Jacobian matrix representing linear and angular velocity relationships.
  
- **`checkSingularityHW3(q: list[float]) -> bool`**  
  Checks if the robot is in a singular configuration by computing the determinant of the linear part of the Jacobian matrix.
  - **Input**:  
    - `q`: List of joint angles (size 3).
  - **Output**:  
    - A boolean flag (`True` if the robot is in a singular configuration, `False` otherwise).
  
- **`computeEffortHW3(q: list[float], w: list[float]) -> list[float]`**  
  Computes the required joint torques given the joint angles `q` and an external wrench `w`.
  - **Input**:  
    - `q`: List of joint angles (size 3).
    - `w`: A wrench vector (size 6) containing moments and forces.
  - **Output**:  
    - List of torques required at each joint.

### 2. `testScript.py`
This script verifies the correctness of the functions implemented in `FRA333_HW3_xx_xx.py` by comparing them with the results obtained using the Robotic Toolbox (`rtb`).

#### Test Descriptions:

- **Test for end-effector Jacobian**:  
  The Jacobian matrix computed using `endEffectorJacobianHW3` is compared with the result from the Robotic Toolbox model of the robot.

- **Test for singularity**:  
  The singularity check function is tested by verifying the determinant of the linear part of the Jacobian using both the custom implementation and Robotic Toolbox.

- **Test for joint torques**:  
  The joint torques calculated using `computeEffortHW3` are compared with the values computed using the Robotic Toolbox based on the same input wrench.

## How to Run

1. Run `FRA333_HW3_xx_xx.py` to execute the functions and display outputs for each problem.
2. Use `testScript.py` to validate the functions against the results obtained using Robotic Toolbox.
