# Moving Platform Inverse Kinematics (IK) Builder

A single-file browser tool for teaching and checking inverse kinematics of a moving sample/platform connected by rods, fixed anchors, 1D sliders, and simplified multi-link chains.

Repository:

[https://github.com/tiktaalika/moving-platform-inverse-kinematics-builder](https://github.com/tiktaalika/moving-platform-inverse-kinematics-builder)

Live web app:

- English: [https://tiktaalika.github.io/moving-platform-inverse-kinematics-builder/index-en.html](https://tiktaalika.github.io/moving-platform-inverse-kinematics-builder/index-en.html)
- Chinese: [https://tiktaalika.github.io/moving-platform-inverse-kinematics-builder/](https://tiktaalika.github.io/moving-platform-inverse-kinematics-builder/)

`IK` is short for `inverse kinematics`. The title keeps both the full term and the abbreviation so the purpose is clear to both robotics and non-robotics readers.

## Usage

This is a static browser app. Use `index-en.html` for the English interface or `index.html` for the Chinese interface.

## What It Solves

Given a target sample pose:

```text
p = [x, y, z]
R = yaw/pitch/roll rotation
```

the app computes each sample attachment point:

```text
q_i = p + R(a_i - r)
```

Then it solves each link:

- `1D slider`: analytic quadratic solution for the actuator stroke.
- `multi-link to slider`: feasible stroke interval from chain reach.
- `intermediate coupler slider`: several rods share one motor-side rigid coupler stroke and are solved together with a residual/Jacobian matrix.
- `fixed anchor`: constraint residual check.

## Path Check

The app samples a path from current pose to target pose:

```text
Current -> middle frames -> Target
```

At each frame it recomputes all actuator commands and checks:

- reachability,
- residual error,
- stroke min/max limits,
- branch continuity by choosing the root nearest to the previous frame.

It can also search a lifted path:

```text
Current -> W1 -> W2 -> Target
```

where:

```text
Z_high = max(Z_current, Z_target) + h
```

The app scans `h` from zero upward and reports the first feasible lift height.

## Verification Panel

The verification panel performs a back-substitution check for the current animation frame:

```text
m_i = b_i + s_i v_i
distance = ||q_i - m_i||
residual = distance - L_i
```

For multi-link chains it checks whether:

```text
Rmin <= distance <= Rmax
```

This panel is the main way to audit whether the numerical result is trustworthy.

## What Is Not Yet Industrial Grade

This tool is a geometry and teaching tool, not a full dynamics simulator. It does not yet include:

- collision detection,
- hinge-angle limits inside a multi-link chain,
- Jacobian singularity / condition-number checks,
- velocity and acceleration limits,
- force/torque/dynamics.

These are the next features to add if the tool needs to approach Drake, Simscape Multibody, or other industrial robotics simulation workflows.

## References and Acknowledgements

This project was designed after reviewing open-source robotics and inverse-kinematics tools. Their ideas helped clarify what should be checked in a serious IK/path-feasibility workflow. No source code from these projects was copied into this repository.

- [Drake](https://github.com/RobotLocomotion/drake): model-based robotics design and verification, constrained IK, differential IK, and joint/velocity/acceleration limits.
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio): rigid-body dynamics, analytical derivatives, constraints, and closed-loop mechanisms.
- [IKPy](https://github.com/Phylliade/ikpy): pure-Python inverse kinematics library for serial chains and URDF models.

The current web app intentionally implements only the small, inspectable geometry layer needed for this moving-platform teaching problem. More industrial features, such as collision checks and Jacobian singularity checks, should be added explicitly rather than hidden inside a black-box solver.
