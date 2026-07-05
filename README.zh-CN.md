# Moving Platform Inverse Kinematics (IK) Builder

这是一个单文件网页工具，用来教学和检查 moving sample/platform 的 inverse kinematics。平台可以连接刚性杆、fixed anchor、一维 slider，以及简化的 multi-link chain。

GitHub 仓库：

[https://github.com/tiktaalika/moving-platform-inverse-kinematics-builder](https://github.com/tiktaalika/moving-platform-inverse-kinematics-builder)

在线网页：

[https://tiktaalika.github.io/moving-platform-inverse-kinematics-builder/](https://tiktaalika.github.io/moving-platform-inverse-kinematics-builder/)

`IK` 是 `inverse kinematics` 的缩写。标题里同时保留完整写法和缩写，是为了让熟悉/不熟悉机器人术语的人都能看懂。

## 使用方式

这是一个静态网页工具。clone 或下载仓库之后，直接用浏览器打开 `index.html`。

## 它解决什么问题

给定 sample/platform 的目标位姿：

```text
p = [x, y, z]
R = yaw/pitch/roll 旋转矩阵
```

程序先计算 sample 上每个连接点的全局位置：

```text
q_i = p + R(a_i - r)
```

然后分别求每条 link：

- `1D slider`：用二次方程解析求 actuator stroke。
- `multi-link to slider`：用 chain reach 求可行 stroke 区间。
- `fixed anchor`：不输出 motor command，只检查 residual。

## 路径检查

程序会把当前位姿到目标位姿采样成多帧：

```text
Current -> middle frames -> Target
```

每一帧都重新计算所有 actuator command，并检查：

- 是否 reachable，
- residual 是否过大，
- stroke 是否超出 min/max，
- 通过“选离上一帧最近的根”来保持 branch 连续。

程序也可以自动搜索抬高路径：

```text
Current -> W1 -> W2 -> Target
```

其中：

```text
Z_high = max(Z_current, Z_target) + h
```

程序从 `h=0` 开始往上搜索，找到第一个可行的 lift height。

## Verification Panel / 反代检查

Verification Panel 会对当前动画帧做反代验证：

```text
m_i = b_i + s_i v_i
distance = ||q_i - m_i||
residual = distance - L_i
```

对于 multi-link chain，它检查：

```text
Rmin <= distance <= Rmax
```

这个面板是判断程序结果是否可信的主要依据。不要只看最终 stroke，要看反代后的 distance 和 residual。

## 现在还不是工业级仿真器

这个工具目前是几何 IK 和教学检查工具，不是完整动力学仿真器。它还没有包括：

- collision detection，
- multi-link chain 内部 hinge angle limit，
- Jacobian singularity / condition number check，
- velocity 和 acceleration limit，
- force/torque/dynamics。

如果以后要接近 Drake、Simscape Multibody 这类工业/研究级工具，这些是下一步应该加的功能。

## 参考与致谢

这个项目在设计时参考了几个开源机器人/IK 工具的功能边界和检查思路。这里参考的是建模和验证思想，没有复制这些项目的源代码。

- [Drake](https://github.com/RobotLocomotion/drake)：model-based robotics design and verification、constrained IK、differential IK，以及 joint/velocity/acceleration limits。
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio)：rigid-body dynamics、analytical derivatives、constraints、closed-loop mechanisms。
- [IKPy](https://github.com/Phylliade/ikpy)：纯 Python inverse kinematics library，适合 serial chain 和 URDF 模型。

当前网页工具刻意只实现这个 moving-platform 教学问题需要的、可逐项检查的几何层。collision check、Jacobian singularity check 等更工业级的功能，后续应该明确加进去，而不是藏在黑箱求解器里。
