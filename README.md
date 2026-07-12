# Moving Platform Inverse Kinematics (IK) Builder

Browser-based teaching and prototyping tool for the inverse kinematics of a rigid moving sample/platform connected to rods, fixed anchors, one-dimensional sliders, or simplified intermediate mechanisms.

> **Demo only — not for machine safety validation or direct machine control.**

- [English web app](https://tiktaalika.github.io/moving-platform-inverse-kinematics-builder/index-en.html)
- [Chinese web app](https://tiktaalika.github.io/moving-platform-inverse-kinematics-builder/)
- [中文说明](#中文说明)

## English

### Purpose

Given a current sample pose and a target sample pose, the app computes the actuator positions required by the configured geometric constraints:

```text
Current pose -> current actuator stroke
Target pose  -> target actuator stroke
Delta stroke = target stroke - current stroke
```

It also samples intermediate poses and checks whether every configured link can satisfy its geometric constraint at each sample.

### Coordinate model

The sample is a rigid body. Let:

- `p` be the global position of the sample reference point;
- `R` be the sample rotation matrix;
- `r` be the reference point in sample-local coordinates;
- `a_i` be attachment point `i` in sample-local coordinates;
- `q_i` be the same attachment point in global coordinates.

The rigid-body transformation is:

```text
q_i = p + R(a_i - r)
```

The GUI uses yaw, pitch, and roll with:

```text
R = Rz(yaw) Ry(pitch) Rx(roll)
```

`Attach local XYZ` is therefore sample-local. Slider bases and fixed anchors are global coordinates.

### Supported link types

#### 1D slider and fixed-length rod

For slider base `b_i`, global unit direction `v_i`, stroke `s_i`, and rod length `L_i`:

```text
m_i(s_i) = b_i + s_i v_i
||q_i - m_i(s_i)||² = L_i²
```

Let `c_i = q_i - b_i`. The stroke is obtained analytically:

```text
s_i = v_iᵀc_i ± sqrt((v_iᵀc_i)² - (c_iᵀc_i - L_i²))
```

A negative square-root term means that no real slider position reaches the requested attachment point. When two roots exist, the selected branch or the nearest previous stroke determines which root is used.

#### Fixed anchor

A fixed anchor has no actuator command. It is valid only when:

```text
residual_i = ||q_i - f_i|| - L_i ≈ 0
```

#### Simplified multi-link chain

For segment lengths `L_1 ... L_n`:

```text
Rmax = sum(L_j)
Rmin = max(0, Lmax - sum(other lengths))
Rmin <= ||q_i - m_i(s_i)|| <= Rmax
```

The app returns a feasible slider-stroke interval. It does **not** determine a unique pose for every internal hinge.

#### Translating rigid coupler

Rods in one coupler group share one slider stroke `s_g`:

```text
k_i(s_g) = b_g + s_g v_g + R_g d_i
e_i(s_g) = ||q_i - k_i(s_g)|| - L_i
F(s_g)   = [e_1, e_2, ...]ᵀ
(JᵀJ) Δs = -JᵀF
```

This is a one-variable Gauss-Newton solve. Group members must share the same base, orientation, and slider axis; only their local coupler offsets may differ.

### How to use the app

1. Enter sample dimensions and its local reference point.
2. Enter the current pose and target pose in global coordinates.
3. Add links and choose each link type.
4. Define sample-local attachment points and global motor/anchor geometry.
5. Check the IK result, both candidate roots, current/target stroke, and delta stroke.
6. Inspect the trajectory table and Verification Panel.
7. Use Play only to visualize the sampled motion; it does not change the calculation.

### Verification

For each sampled pose, the app substitutes the computed stroke back into the original constraint:

```text
m_i = b_i + s_i v_i
distance = ||q_i - m_i||
residual = distance - L_i
```

A platform pose passes only when **all** links pass simultaneously. Stroke limits and coupler configuration errors are checked as well. A small residual confirms numerical consistency with the configured model, not physical machine safety.

### Model assumptions and limitations

This tool is appropriate for teaching, analytical checking, and early mechanism prototyping under the following assumptions:

- the sample/platform is perfectly rigid;
- dimensions, coordinate frames, attachment points, and slider axes are exact;
- rods are rigid and joints are ideal;
- each simple slider link follows the model above;
- a coupler group is a fixed-orientation rigid body translating along one axis.

It does **not** provide:

- collision or interference detection;
- internal hinge positions for a general multi-link mechanism;
- hinge-axis, hinge-angle, or joint-limit validation;
- arbitrary rotating or nested intermediate-body kinematics;
- continuous-path proof between sampled poses;
- Jacobian condition-number or singularity analysis;
- backlash, compliance, tolerances, calibration errors, or sensor uncertainty;
- velocity, acceleration, force, torque, dynamics, or controller timing;
- fail-safe logic, safety integrity, or machine certification.

Increasing `Steps` checks more discrete poses but never proves that the continuous path is collision-free. The automatic lift search tests only a bounded lift-translate-lower strategy; `not found` does not prove that no feasible path exists.

Before using results on hardware, independently validate the complete CAD/multibody model, joint limits, collisions, loads, controls, emergency stops, and applicable safety requirements.

### References

The design was informed by the verification concepts of [Drake](https://github.com/RobotLocomotion/drake), [Pinocchio](https://github.com/stack-of-tasks/pinocchio), and [IKPy](https://github.com/Phylliade/ikpy). No source code was copied from these projects.

## 中文说明

这是一个用于教学、解析检查和机构早期设计的网页工具。给定 sample/platform 的当前位姿和目标位姿，程序按照用户配置的几何约束反求 actuator 位置：

```text
当前位姿 -> current actuator stroke
目标位姿 -> target actuator stroke
移动量   = target stroke - current stroke
```

> **Demo only — 仅用于教学与几何验证，不可用于机器安全认证或直接控制机器。**

### 坐标与刚体变换

- `p`：sample 参考点的全局位置；
- `R`：sample 的旋转矩阵；
- `r`：sample 局部坐标中的参考点；
- `a_i`：第 `i` 个 sample 连接点的局部坐标；
- `q_i`：该连接点变换后的全局坐标。

```text
q_i = p + R(a_i - r)
R = Rz(yaw) Ry(pitch) Rx(roll)
```

因此 `Attach local XYZ` 是 sample 局部坐标；slider base 和 fixed anchor 使用全局坐标。

### 支持的连杆模型

#### 一维 slider + 定长杆

```text
m_i(s_i) = b_i + s_i v_i
||q_i - m_i(s_i)||² = L_i²
s_i = v_iᵀc_i ± sqrt((v_iᵀc_i)² - (c_iᵀc_i - L_i²))
c_i = q_i - b_i
```

根号内为负表示目标点不可达；存在两个根时，程序按照选择的 branch 或离上一帧最近的 stroke 选解。

#### Fixed anchor

Fixed anchor 没有 actuator command，只检查：

```text
residual_i = ||q_i - f_i|| - L_i ≈ 0
```

#### 简化 multi-link chain

```text
Rmax = sum(L_j)
Rmin = max(0, Lmax - sum(other lengths))
Rmin <= ||q_i - m_i(s_i)|| <= Rmax
```

程序只给出 slider 的可行 stroke 区间，**不会**确定每个中间铰点的唯一位置和姿态。

#### 平移刚体 coupler

```text
k_i(s_g) = b_g + s_g v_g + R_g d_i
e_i(s_g) = ||q_i - k_i(s_g)|| - L_i
F(s_g)   = [e_1, e_2, ...]ᵀ
(JᵀJ) Δs = -JᵀF
```

这是只有一个共享未知 stroke 的 Gauss-Newton 求解。同一 coupler group 必须共用 base、姿态和 slider axis；各杆的 coupler local offset 可以不同。

### 使用步骤

1. 输入 sample 尺寸和局部参考点。
2. 输入全局 Current Pose 和 Target Pose。
3. 添加 link 并选择类型。
4. 填写 sample 局部连接点，以及全局 slider/anchor 几何。
5. 检查两个候选根、current stroke、target stroke 和 delta stroke。
6. 检查轨迹表和 Verification Panel。
7. Play 只用于显示采样运动过程，不会改变计算结果。

### Verification / 反代验证

程序把求出的 stroke 代回原约束：

```text
m_i = b_i + s_i v_i
distance = ||q_i - m_i||
residual = distance - L_i
```

所有 link 必须同时通过，整个平台位姿才通过。程序还检查 stroke limit 和 coupler configuration error。Residual 接近零只说明结果与当前配置的数学模型一致，不代表真实机器安全。

### 模型假设与局限

适用前提：

- sample/platform 是理想刚体；
- 尺寸、坐标系、连接点和 slider axis 输入准确；
- 连杆刚性、关节理想；
- 简单 slider 符合上述一维模型；
- coupler 是姿态固定、沿单轴平移的刚体。

当前程序**不包括**：

- 碰撞和空间干涉检测；
- 一般多连杆机构的内部铰点位置；
- hinge axis、hinge angle 和 joint limit 检查；
- 任意旋转或多层嵌套中间机构运动学；
- 采样点之间连续路径的严格证明；
- Jacobian 条件数和奇异性分析；
- 间隙、柔性、公差、标定误差和传感器不确定性；
- 速度、加速度、力、扭矩、动力学和控制器时序；
- fail-safe、安全完整性或机器认证。

增加 `Steps` 只能增加离散检查点，不能证明连续路径无碰撞。自动 Lift 只搜索有限范围内的“抬高—平移—下降”路径；`not found` 不代表不存在其他可行路径。

把结果用于硬件之前，必须另外验证完整 CAD/多体模型、关节限制、碰撞、载荷、控制系统、急停和适用的机器安全要求。

### 参考

项目参考了 [Drake](https://github.com/RobotLocomotion/drake)、[Pinocchio](https://github.com/stack-of-tasks/pinocchio) 和 [IKPy](https://github.com/Phylliade/ikpy) 的建模与验证思想，没有复制这些项目的源代码。

## License

MIT. See [LICENSE](LICENSE).
