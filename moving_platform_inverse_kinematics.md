# Moving Platform Inverse Kinematics

## 1. What Problem Are We Solving?

The setup is a moving platform connected to three independently controlled motors or actuator units. Each motor has its own local coordinate system. The platform has three corresponding attachment points.

The inverse kinematics problem is:

Given the desired position and orientation of the moving platform, compute the command that each motor/slider must produce.

This is different from simply adding three motor displacements. The platform is a rigid body, so its attachment points must move consistently with one single rigid-body transformation.

## 2. Coordinate Frames

Define a fixed global/base frame:

```text
{B}
```

Define a moving platform frame:

```text
{P}
```

For leg or motor `i`, define:

```text
b_i = motor base/home point in the global frame
R_i = rotation matrix from motor i local frame to the global frame
a_i = platform attachment point in the platform-local frame
v_i = slider direction in the global frame
L_i = fixed rod/link length
```

If the slider axis is local X, then:

```text
v_i = R_i [1, 0, 0]^T
```

If the slider axis is local Y or Z, replace `[1, 0, 0]^T` with the corresponding unit vector.

For the platform target pose, define:

```text
p = desired global position of the chosen platform reference point
R = desired rotation matrix from platform frame to global frame
r = chosen reference point in the platform-local frame
```

Usually `r = [0, 0, 0]^T` if the platform coordinate origin is the reference point.

## 3. Platform Attachment Point Motion

Because the platform is rigid, every point fixed on the platform moves by the same rigid-body transformation.

In matrix form, the platform pose is represented by a homogeneous transformation matrix:

```text
T =
[ R  p ]
[ 0  1 ]
```

where:

```text
R =
[ r11 r12 r13 ]
[ r21 r22 r23 ]
[ r31 r32 r33 ]

p =
[ px ]
[ py ]
[ pz ]
```

For a platform-local attachment point:

```text
a_i =
[ aix ]
[ aiy ]
[ aiz ]
```

and a platform-local reference point:

```text
r =
[ rx ]
[ ry ]
[ rz ]
```

the local vector from the reference point to the attachment point is:

```text
alpha_i = a_i - r
```

In homogeneous coordinates:

```text
alpha_i^h =
[ aix - rx ]
[ aiy - ry ]
[ aiz - rz ]
[    1     ]
```

Then the global position of the attachment point is:

```text
q_i^h = T alpha_i^h
```

That is:

```text
[ qix ]   [ r11 r12 r13 px ] [ aix - rx ]
[ qiy ] = [ r21 r22 r23 py ] [ aiy - ry ]
[ qiz ]   [ r31 r32 r33 pz ] [ aiz - rz ]
[  1  ]   [  0   0   0  1 ] [    1     ]
```

After multiplication:

```text
q_i = p + R(a_i - r)
```

For attachment point `a_i`, its target global position is:

```text
q_i = p + R(a_i - r)
```

where:

```text
q_i = target global position of platform attachment point i
p   = target global position of platform reference point
R   = target platform orientation
a_i = local coordinates of attachment point i
r   = local coordinates of the reference point
```

This equation is the core rigid-body kinematics equation.

## 3.1 Rotation Matrix Used in the GUI

The GUI uses yaw, pitch, and roll to build the platform rotation matrix:

```text
R = Rz(yaw) Ry(pitch) Rx(roll)
```

where:

```text
Rz(psi) =
[ cos(psi) -sin(psi) 0 ]
[ sin(psi)  cos(psi) 0 ]
[    0         0     1 ]
```

```text
Ry(theta) =
[  cos(theta) 0 sin(theta) ]
[      0      1     0      ]
[ -sin(theta) 0 cos(theta) ]
```

```text
Rx(phi) =
[ 1     0          0     ]
[ 0 cos(phi) -sin(phi) ]
[ 0 sin(phi)  cos(phi) ]
```

So the platform transform matrix is:

```text
T =
[ Rz Ry Rx   p ]
[    0       1 ]
```

The same form is used for each slider/motor local coordinate frame. If a slider's local frame rotation is `R_i`, and its selected local stroke axis is local X:

```text
e_x =
[1]
[0]
[0]
```

then the slider stroke direction in the global frame is:

```text
v_i = R_i e_x
```

If the selected local axis is Y or Z:

```text
v_i = R_i e_y
```

or:

```text
v_i = R_i e_z
```

## 4. Case A: Motor Can Command Local XYZ Displacement

If the motor endpoint can be directly positioned in local XYZ, the motor must move from its base/home point `b_i` to the target joint position `q_i`.

The required displacement in the global frame is:

```text
d_i^B = q_i - b_i
```

But the motor command is needed in the motor's own local coordinate system. Since `R_i` maps motor-local coordinates to global coordinates:

```text
d_i^B = R_i u_i
```

Therefore:

```text
u_i = R_i^T d_i^B
```

Substituting `d_i^B = q_i - b_i`:

```text
u_i = R_i^T(q_i - b_i)
```

And substituting the platform point equation:

```text
u_i = R_i^T( p + R(a_i - r) - b_i )
```

This is the analytical inverse kinematics solution for a motor that can command local XYZ displacement.

## 5. Case B: Slider Plus Fixed-Length Rod

The example mechanism is a moving platform connected by rods or links. In that model, the motor does not place the joint freely in XYZ. Instead, a slider moves along one known axis and a rod of fixed length connects the slider point to the platform attachment point.

Let the slider position be:

```text
m_i(s_i) = b_i + s_i v_i
```

where:

```text
s_i = scalar slider stroke
v_i = unit direction of the slider axis in the global frame
```

The fixed rod length constraint is:

```text
|| q_i - m_i(s_i) ||^2 = L_i^2
```

Substitute `m_i(s_i) = b_i + s_i v_i`:

```text
|| q_i - b_i - s_i v_i ||^2 = L_i^2
```

Let:

```text
c_i = q_i - b_i
```

Then:

```text
|| c_i - s_i v_i ||^2 = L_i^2
```

Expand:

```text
(c_i - s_i v_i)^T(c_i - s_i v_i) = L_i^2
```

Assuming `v_i` is a unit vector:

```text
s_i^2 - 2(v_i^T c_i)s_i + (c_i^T c_i - L_i^2) = 0
```

This is a quadratic equation in the unknown slider stroke `s_i`.

In standard quadratic form:

```text
A_i s_i^2 + B_i s_i + C_i = 0
```

where:

```text
A_i = v_i^T v_i
B_i = -2 v_i^T c_i
C_i = c_i^T c_i - L_i^2
```

Because `v_i` is a unit vector:

```text
A_i = 1
```

So:

```text
B_i = -2 v_i^T c_i
C_i = c_i^T c_i - L_i^2
```

The quadratic formula gives:

```text
s_i = (-B_i ± sqrt(B_i^2 - 4 A_i C_i)) / (2 A_i)
```

Substituting `A_i = 1`, `B_i = -2 v_i^T c_i`, and `C_i = c_i^T c_i - L_i^2`:

```text
s_i =
(2 v_i^T c_i ± sqrt(4(v_i^T c_i)^2 - 4(c_i^T c_i - L_i^2))) / 2
```

Therefore:

```text
s_i = v_i^T c_i ± sqrt( (v_i^T c_i)^2 - (c_i^T c_i - L_i^2) )
```

The analytical solution is:

```text
s_i = v_i^T c_i ± sqrt( (v_i^T c_i)^2 - (c_i^T c_i - L_i^2) )
```

Substitute back `c_i = q_i - b_i`:

```text
s_i = v_i^T(q_i - b_i) ± sqrt( [v_i^T(q_i - b_i)]^2 - ( ||q_i - b_i||^2 - L_i^2 ) )
```

And since:

```text
q_i = p + R(a_i - r)
```

the full inverse kinematics formula is:

```text
s_i =
v_i^T( p + R(a_i - r) - b_i )
± sqrt(
  [v_i^T( p + R(a_i - r) - b_i )]^2
  - ( ||p + R(a_i - r) - b_i||^2 - L_i^2 )
)
```

The two signs correspond to the two possible geometric intersections of a line and a sphere. In a real machine, the correct branch is selected from the physical assembly direction and stroke limits.

If the square-root term is negative, the requested platform pose is not reachable for that leg:

```text
[v_i^T c_i]^2 - (c_i^T c_i - L_i^2) < 0
```

## 5.1 Numeric Procedure Used by the GUI

For each link, the GUI computes the final number as follows.

First, it builds the platform transformation:

```text
T =
[ R  p ]
[ 0  1 ]
```

Then it computes the global platform joint:

```text
q_i = p + R(a_i - r)
```

For a slider link:

```text
c_i = q_i - b_i
```

The slider axis is computed from the slider local frame:

```text
v_i = R_i e_i
```

where `e_i` is one of:

```text
e_x = [1, 0, 0]^T
e_y = [0, 1, 0]^T
e_z = [0, 0, 1]^T
```

Then the quadratic coefficients are:

```text
A_i = v_i^T v_i
B_i = -2 v_i^T c_i
C_i = c_i^T c_i - L_i^2
```

The discriminant is:

```text
D_i = B_i^2 - 4 A_i C_i
```

If `D_i < 0`, the GUI reports `unreachable`.

If `D_i >= 0`, the two candidate strokes are:

```text
s_i^- = (-B_i - sqrt(D_i)) / (2 A_i)
```

```text
s_i^+ = (-B_i + sqrt(D_i)) / (2 A_i)
```

When `Solution branch = positive stroke`, the GUI chooses the smallest non-negative candidate:

```text
s_i = min({s_i^-, s_i^+} where s_i >= 0)
```

If neither candidate is non-negative, the GUI returns the minus branch by default and shows both roots, so the user can see that the selected slider axis points in the opposite direction.

After choosing `s_i`, the slider endpoint is:

```text
m_i = b_i + s_i v_i
```

The residual check is:

```text
residual_i = ||q_i - m_i|| - L_i
```

For a correct reachable solution, this should be close to zero.

For a fixed anchor link, there is no `s_i`. The GUI only computes:

```text
residual_i = ||q_i - f_i|| - L_i
```

where `f_i` is the fixed anchor point.

## 5.2 When Constraints Increase: The Matrix to Solve

There are two different problems.

### Problem 1: Known Target Pose, Solve Motor Commands

This is the current inverse-kinematics direction:

```text
given p, R -> compute q_i -> solve each slider stroke s_i
```

This is not one big linear matrix inverse. The GUI first uses the 4x4 pose matrix:

```text
T =
[ R  p ]
[ 0  1 ]
```

to compute:

```text
q_i = p + R(a_i - r)
```

Then each slider stroke is obtained from one scalar quadratic equation:

```text
A_i s_i^2 + B_i s_i + C_i = 0
```

### Problem 2: Unknown Platform Pose, Solve Pose From Constraints

If the platform pose is unknown, and we know several slider strokes or fixed anchors, then we must solve for:

```text
x =
[ px, py, pz, psi, theta, phi ]^T
```

where:

```text
psi   = yaw
theta = pitch
phi   = roll
```

The platform point is now a function of `x`:

```text
q_i(x) = p + R(psi, theta, phi)(a_i - r)
```

#### Fixed Anchor Constraint Row

For a fixed anchor link, the known fixed anchor is:

```text
f_i =
[ fix, fiy, fiz ]^T
```

The rod length is `L_i`. The residual equation is:

```text
F_i(x) = || q_i(x) - f_i ||^2 - L_i^2 = 0
```

This is one row in the nonlinear system.

#### Slider Constraint Row

For a slider, if the current/known stroke is `s_i`, then the slider endpoint is known:

```text
m_i = b_i + s_i v_i
```

where:

```text
b_i = slider base point
v_i = slider axis direction
```

The residual equation is:

```text
F_i(x) = || q_i(x) - m_i ||^2 - L_i^2 = 0
```

This has the same structure as the fixed anchor equation; only the endpoint changes from `f_i` to `m_i`.

#### Stacked Constraint Vector

For `n` links:

```text
F(x) =
[ F_1(x) ]
[ F_2(x) ]
[ F_3(x) ]
[  ...   ]
[ F_n(x) ]
```

The goal is:

```text
F(x) = 0
```

#### Jacobian Matrix

The matrix to solve is built from derivatives of each residual with respect to the unknown pose variables:

```text
J(x) =
[ dF_1/dpx  dF_1/dpy  dF_1/dpz  dF_1/dpsi  dF_1/dtheta  dF_1/dphi ]
[ dF_2/dpx  dF_2/dpy  dF_2/dpz  dF_2/dpsi  dF_2/dtheta  dF_2/dphi ]
[ dF_3/dpx  dF_3/dpy  dF_3/dpz  dF_3/dpsi  dF_3/dtheta  dF_3/dphi ]
[   ...       ...       ...        ...          ...          ...   ]
[ dF_n/dpx  dF_n/dpy  dF_n/dpz  dF_n/dpsi  dF_n/dtheta  dF_n/dphi ]
```

For each row, define:

```text
d_i(x) = q_i(x) - endpoint_i
```

where:

```text
endpoint_i = f_i              for a fixed anchor
endpoint_i = b_i + s_i v_i    for a slider with known stroke
```

Then:

```text
F_i(x) = d_i(x)^T d_i(x) - L_i^2
```

The translation derivatives are simple:

```text
dF_i/dpx = 2 d_i^T [1, 0, 0]^T
dF_i/dpy = 2 d_i^T [0, 1, 0]^T
dF_i/dpz = 2 d_i^T [0, 0, 1]^T
```

or:

```text
[ dF_i/dpx  dF_i/dpy  dF_i/dpz ] = 2 d_i^T
```

The rotation derivatives are:

```text
dF_i/dpsi   = 2 d_i^T (dR/dpsi)(a_i - r)
dF_i/dtheta = 2 d_i^T (dR/dtheta)(a_i - r)
dF_i/dphi   = 2 d_i^T (dR/dphi)(a_i - r)
```

Therefore, row `i` of the Jacobian is:

```text
J_i =
[
  2 d_i^T e_x,
  2 d_i^T e_y,
  2 d_i^T e_z,
  2 d_i^T (dR/dpsi)(a_i-r),
  2 d_i^T (dR/dtheta)(a_i-r),
  2 d_i^T (dR/dphi)(a_i-r)
]
```

where:

```text
e_x = [1,0,0]^T
e_y = [0,1,0]^T
e_z = [0,0,1]^T
```

#### Newton Matrix Solve

At iteration `k`, evaluate:

```text
F_k = F(x_k)
J_k = J(x_k)
```

If the number of independent constraints is exactly 6, solve:

```text
J_k Δx = -F_k
```

Then update:

```text
x_{k+1} = x_k + Δx
```

#### More Than 6 Constraints

If there are more than 6 constraints, the system is overdetermined. Then use the least-squares normal equation:

```text
J_k^T J_k Δx = -J_k^T F_k
```

The matrix being solved is:

```text
[             ] [ Δpx    ]   [             ]
[             ] [ Δpy    ]   [             ]
[   J_k^T J_k ] [ Δpz    ] = [ -J_k^T F_k ]
[             ] [ Δpsi   ]   [             ]
[             ] [ Δtheta ]   [             ]
[             ] [ Δphi   ]   [             ]
```

Explicitly:

```text
[ H11 H12 H13 H14 H15 H16 ] [ Δpx    ]   [ g1 ]
[ H21 H22 H23 H24 H25 H26 ] [ Δpy    ]   [ g2 ]
[ H31 H32 H33 H34 H35 H36 ] [ Δpz    ] = [ g3 ]
[ H41 H42 H43 H44 H45 H46 ] [ Δpsi   ]   [ g4 ]
[ H51 H52 H53 H54 H55 H56 ] [ Δtheta ]   [ g5 ]
[ H61 H62 H63 H64 H65 H66 ] [ Δphi   ]   [ g6 ]
```

where:

```text
H = J_k^T J_k
g = -J_k^T F_k
```

This is the matrix form that should be shown when solving platform pose from many constraints.

So:

- Current GUI inverse kinematics: 4x4 transform matrix + scalar quadratic equation per slider.
- Full pose reconstruction from many constraints: nonlinear residual vector `F(x)`, Jacobian matrix `J(x)`, and the solve `J Δx = -F` or `J^T J Δx = -J^T F`.

## 6. Final Formula for the Example Mechanism

For each leg `i = 1, 2, 3`:

```text
q_i = p + R(a_i - r)
```

```text
c_i = q_i - b_i
```

```text
s_i = v_i^T c_i ± sqrt( (v_i^T c_i)^2 - (c_i^T c_i - L_i^2) )
```

This gives the analytical inverse kinematics result: the required slider stroke for each motor.

## 7. Final Formula for a Full XYZ Motor

For each motor `i = 1, 2, 3`:

```text
q_i = p + R(a_i - r)
```

```text
u_i = R_i^T(q_i - b_i)
```

or directly:

```text
u_i = R_i^T( p + R(a_i - r) - b_i )
```

where `u_i = [u_ix, u_iy, u_iz]^T` is the required motor command in motor `i`'s local XYZ coordinate system.

## 8. If a Motor Only Moves Along One Axis Without a Rod

If a real actuator only moves along one local axis, for example local X, then it cannot generally realize an arbitrary 3D displacement.

Let the actuator axis in local coordinates be:

```text
e = [1, 0, 0]^T
```

The scalar actuator stroke is:

```text
s_i = e^T u_i
```

The unachievable perpendicular error is:

```text
epsilon_i = u_i - s_i e
```

If `||epsilon_i||` is not close to zero, the requested platform pose is not reachable by that single-axis actuator model.

## 9. Why This Solves the Moving-Platform Problem

The goal is an analytical inverse kinematics solution for a moving platform. The analytical solution comes from these facts:

1. The moving platform is a rigid body, so all platform attachment points follow the same transform.
2. Each platform attachment point has a target global position once the desired pose is known.
3. Each leg then becomes either a coordinate transformation problem or a line-sphere intersection problem.

For a full XYZ motor, the problem reduces to:

```text
desired platform pose -> target attachment points -> motor-local displacement commands
```

For a slider plus fixed-length rod, the problem reduces to:

```text
desired platform pose -> target attachment points -> slider stroke from a quadratic equation
```

No MATLAB, Mathematica, or numerical optimizer is required for this simplified analytical model. Those tools are useful for checking algebra, simulating a more detailed CAD mechanism, or handling extra constraints, but the core inverse kinematics can be written directly.

## 10. What the Web GUI Does

The GUI implements both formulas.

For an example slider and rod mechanism:

```text
s_i = v_i^T c_i ± sqrt( (v_i^T c_i)^2 - (c_i^T c_i - L_i^2) )
```

For a full local XYZ motor:

```text
u_i = R_i^T( p + R(a_i - r) - b_i )
```

It lets the user enter:

```text
target platform position p
target platform orientation R
platform reference point r
motor base/home points b_i
motor local frame orientations R_i
platform attachment points a_i
```

Then it outputs:

```text
s_1, s_2, s_3
```

These are the required slider strokes. It also shows the equivalent local XYZ displacement as a reference.
