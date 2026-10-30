# Modelisation3D : KUKA LBR4+ Kinematics & Simulation

![Octave](https://img.shields.io/badge/Octave-Compatible-blue)
![CoppeliaSim](https://img.shields.io/badge/CoppeliaSim-Validated-green)
![Robotics](https://img.shields.io/badge/Robotics-Kinematics-orange)

A complete mathematical modeling and simulation of the **KUKA LBR4+ (7-DOF redundant manipulator)** implemented in **GNU Octave**.

This project covers forward kinematics, the geometric Jacobian, numerical inverse kinematics, and center of mass calculation. The mathematical model can be validated against the **CoppeliaSim** simulator through the ZeroMQ Remote API.

## Demonstration

![KUKA LBR4+ Animation](docs/lbr4_demo.gif)

*Animation generated natively in Octave using the forward kinematics model.*

## Features

- **Forward Kinematics (`forward_kinematics.m`):** Computes the end-effector pose as a 4 × 4 homogeneous transformation matrix using the Modified Denavit–Hartenberg (Craig) convention.
- **Geometric Jacobian (`jacobian.m`):** Computes the 6 × 7 Jacobian matrix relating joint velocities to the end-effector's linear and angular velocities.
- **Inverse Kinematics (`inverse_kinematics.m`):** Implements a numerical inverse kinematics solver based on the Jacobian pseudoinverse to handle the manipulator's 7-DOF redundancy.
- **Center of Mass (`center_of_mass.m`):** Calculates the global center of mass using link masses and local center-of-mass coordinates.
- **3D Visualization (`plot_robot.m` and `animate_robot.m`):** Provides custom 3D plotting and smooth GIF generation, fully compatible with GNU Octave without requiring additional toolboxes.
- **CoppeliaSim Validation (`validate_coppelia.m`):** Connects to CoppeliaSim through the ZeroMQ Remote API to compare the simulated end-effector position with the result obtained from the Octave forward kinematics model.

## Project Structure

```text
Modelisation3D/
├── main.m                      # Entry point: runs tests and generates the GIF
├── README.md
├── .gitignore
├── kinematics/                 # Mathematical core
│   ├── forwardKinematics.m
│   ├── jacobian.m
│   ├── inverseKinematics.m
│   └── centerOfMass.m
├── visualization/              # 3D rendering
│   ├── plotRobot.m
│   └── animateRobot.m
├── validation/                 # CoppeliaSim integration
|   ├── alignmentMatrix.m
│   └── validateCoppelia.m
├── zmqRemoteApi/               # CoppeliaSim ZeroMQ Remote API (Git submodule)
└── docs/                       # Documentation and media assets
    └── lbr4_demo.gif
```

## How to Run

### Prerequisites

- **GNU Octave or MATLAB:** Required to execute the mathematical model and visualization scripts.
- **CoppeliaSim:** Optional; required only for validation against the simulator.

### 1. Clone the Repository

The project uses a Git submodule for the CoppeliaSim ZeroMQ Remote API. Clone the repository with its submodules:

```bash
git clone --recurse-submodules https://github.com/vinyIsCollapsing/Modelisation3D.git
cd Modelisation3D
```

If you have already cloned the repository without its submodules, initialize them with:

```bash
git submodule update --init --recursive
```

### 2. Run the Mathematical Simulation

Open GNU Octave in the project root directory and execute:

```octave
main
```

This runs the main simulation, displays the calculated end-effector pose, plots the robot, and generates or updates `docs/lbr4_demo.gif`.

### 3. Validate the Model Against CoppeliaSim

The project includes a validation script at `validation/validate_coppelia.m`. It connects to CoppeliaSim through the ZeroMQ Remote API, sends a set of joint angles, retrieves the simulated end-effector position, and compares it with the position calculated by the Octave forward kinematics model.

#### Step 1: Start CoppeliaSim

1. Launch CoppeliaSim.
2. Load a scene containing the KUKA LBR4+ model.
3. Ensure that the ZeroMQ Remote API add-on is available and running.

#### Step 2: Run the Validation Script

In GNU Octave, navigate to the project root directory and execute:

```octave
addpath('validation');
validateCoppelia
```

The script reports the end-effector position obtained from CoppeliaSim, the position calculated by the Octave model, and the Euclidean position error.

**Note:** A meaningful comparison requires consistent joint configurations, coordinate frames, base-frame alignment, and end-effector reference points. The CoppeliaSim joint angles must correspond to the same configuration used by the Octave model.

## Mathematical Convention

The robot model uses the **Modified Denavit–Hartenberg (MDH) convention**, following Craig's formulation.

The parameters are interpreted as follows:

- **`a` — Link length:** Translation along the local X-axis.
- **`d` — Link offset:** Translation along the local Z-axis.
- **`alpha` — Link twist:** Rotation about the local X-axis.
- **`theta` — Joint angle:** Rotation about the local Z-axis.

For a revolute joint, `theta` is the joint variable, while the other parameters are constant.

### Reference Frames

- **Base frame:** Aligned with the robot's physical base frame.
- **End-effector frame:** Defined at the selected terminal reference point, corresponding to the force sensor connection in the CoppeliaSim model.

The exact frame definitions and fixed transformations must be consistent between the Octave implementation and the CoppeliaSim scene for the validation results to be meaningful.

## Validation Results

The initial comparison between the Octave mathematical model and the CoppeliaSim simulation showed a discrepancy of **426 mm**. Through a systematic frame-alignment analysis, the discrepancy was traced to two main factors:

1. **Base Frame Translation:** The CAD model in CoppeliaSim is translated by `(-250 mm, +250 mm)` relative to the world origin.
2. **Joint Axis Conventions:** Testing each joint individually revealed that **joints 2, 4, and 6 rotate in the opposite direction** in the CoppeliaSim URDF compared to the standard Modified DH convention.

After applying these corrections (base translation and sign inversion for joints 2, 4, and 6), the end-effector position error dropped to **< 1 mm**.

| Measurement | X (m) | Y (m) | Z (m) |
| :--- | :--- | :--- | :--- |
| **CoppeliaSim** | -0.8638 | 0.2500 | 0.9243 |
| **Octave FK (adjusted)** | -0.8638 | 0.2500 | 0.9243 |

**Final Position Error: ~0.00 mm**

This confirms that the Modified Denavit-Hartenberg parameters are mathematically correct.

## References

- [GNU Octave](https://octave.org/)
- [CoppeliaSim](https://www.coppeliarobotics.com/)
- [CoppeliaSim ZeroMQ Remote API](https://github.com/CoppeliaRobotics/zmqRemoteApi)
- [CoppeliaSim Remote API Documentation](https://manual.coppeliarobotics.com/en/remoteApiOverview.htm)