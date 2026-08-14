# my_cartesian_impedance_controller_cmp_dq

A Cartesian impedance controller for `ros2_control` whose pose error is a
**dual-quaternion screw displacement**, with a **feedforward wrench input**
summed into the control law.

```
F_des = screw_stiffness * error_screw  -  damping * velocity  +  F_ff
tau   = J^T * F_des  +  tau_nullspace  +  coriolis
```

Translation and rotation are driven as one coupled screw motion rather than as
two independent error terms.

Kinematics and dynamics (Jacobian, Coriolis) come from **Pinocchio**, computed
from a URDF at runtime. There is no dependency on libfranka or any
Franka-specific dynamics library, so the control logic itself is robot-agnostic
— see [Porting to another arm](#porting-to-another-arm).

This is a variant of
[`my_cartesian_impedance_controller`](https://github.com/BastienMourin/franka_ros2_shared/tree/my_cartesian_impedance_controller).
For a side-by-side of the two, see [COMPARISON.md](COMPARISON.md).

---

## Dependencies

Beyond the usual `ros2_control` stack and Pinocchio, this package needs one
extra thing: [`dq_operations`](https://github.com/BastienMourin/dq_operations),
for the dual-quaternion maths.

It is deliberately **not** listed in this workspace's `franka.repos`. That file
pulls in `libfranka` and `franka_description`, which you will be dropping if you
are running a non-Franka arm — `dq_operations` is robot-independent and should
outlive it. Clone it into your workspace's `src/` directly:

```bash
git clone https://github.com/BastienMourin/dq_operations.git src/dq_operations

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to my_cartesian_impedance_controller_cmp_dq
```

> `dq_operations` is currently a **private** repository. If the clone fails with
> a permission error, ask Bastien Mourin for access.

It is a plain `ament_cmake` package that depends only on `rclcpp`, `Eigen3` and
`orocos_kdl`, so it drops into any colcon workspace.

The controller uses only four functions from it — `rotationTranslationToDQ`,
`multiplyDQ`, `classicConjugateDQ` and `dqToScrewParameters`, all in `dq.cpp`.
The `orocos_kdl` dependency comes from three other libraries in that package
that the controller never touches. If it is ever in the way, those four
functions are self-contained enough to vendor into this package as an
Eigen-only header.

---

## Running it

The controller is declared in `franka_bringup/config/controllers.yaml` and can
be spawned like any other:

```bash
ros2 launch franka_bringup franka.launch.py robot_ip:=<ip>
ros2 control switch_controllers --activate my_cartesian_impedance_controller_cmp_dq
```

On activation the equilibrium pose is set to wherever the arm already is, so it
never commands a step. It then holds that pose until you publish a target.

### Topics

| topic | type | direction |
|---|---|---|
| `~/target_pose` | `geometry_msgs/PoseStamped` | in — equilibrium pose, in the robot base frame |
| `~/feedforward_wrench` | `geometry_msgs/WrenchStamped` | in — feedforward wrench, world-aligned at the EE |

Both are low-pass filtered on the way in (see `alpha`). Publish `target_pose`
at whatever rate your trajectory source runs at; the controller interpolates.

### Diagnostics

| topic | contents |
|---|---|
| `~/cartesian_error` | `error_screw` as a `WrenchStamped` — `force` is the linear part, `torque` the angular |
| `~/wrench_cmd` | the commanded wrench **before** the Cartesian clamp. Compare its norms against `max_force` / `max_torque` to see whether saturation is binding |
| `~/tau_cmd` | 20 doubles: `[0..6]` tau after the slew limiter, `[7..13]` tau written to the hardware, `[14..19]` the live damping diagonal. A difference between the first two blocks means the per-joint clamp bound |
| `~/dq_state` | 34 doubles: `[0..7]` `dq_curr`, `[8..15]` `dq_des` before the shortest-path flip, `[16..23]` `dq_error` after it, `[24]` `theta` before, `[25]` `theta` after, `[26]` 1.0 if the flip fired, `[27]` `d_e`, `[28..30]` `l_e`, `[31..33]` `m_e` |

These publish from the 1 kHz loop without a real-time publisher. It has not
caused trouble in practice, but if you are chasing jitter, this is a thing you
can turn off.

---

## Parameters

Set in `franka_bringup/config/controllers.yaml`. The gains can also be changed
live with `ros2 param set` — they feed filtered targets, so a change ramps in
over many cycles instead of landing in a single 1 ms step.

| parameter | default | meaning |
|---|---|---|
| `screw_stiffness` | `250.0` | the one stiffness, applied to all six screw-error components |
| `rot_damping_ratio` | `0.2` | rotational damping as a fraction of translational |
| `nullspace_stiffness` | `0.0` | pull toward the activation posture; `0.0` disables |
| `alpha` | `0.005` | per-cycle low-pass on the desired pose, stiffness and damping |
| `delta_tau_max` | `1.0` | torque slew limit, Nm per 1 ms cycle |
| `wrench_saturation` | `false` | apply the Cartesian wrench clamp |
| `max_force` | `25.0` | N, only used when `wrench_saturation` is true |
| `max_torque` | `25.0` | Nm, only used when `wrench_saturation` is true |
| `tau_limits` | FR3 limits | per-joint torque clamp, **always** applied |
| `arm_id` / `arm_prefix` | `fr3` / `""` | joint-name prefix, `<prefix>_<arm_id>_joint<N>` |

### What to tune

**`screw_stiffness` first.** This is the main knob. Higher tracks the target
more tightly; lower is more compliant. Damping is derived from it as

```
D_trans = 2*sqrt(screw_stiffness)
D_rot   = rot_damping_ratio * D_trans
```

so the damping ratio stays constant as you retune the stiffness — you do not
have to re-derive damping by hand. As a starting point, around `400` works well
while measuring contact forces, and around `50` when replaying with a large
feedforward wrench.

**`alpha` if tracking lags.** Smaller is smoother but adds delay. Raising it
reduces lag at the cost of larger transients when the target jumps; if you raise
it a long way, watch `~/wrench_cmd` for spikes.

**`rot_damping_ratio` only if the wrist chatters.** Rotation is the axis that
destabilises first: apparent inertia at the end-effector is a few kg
translationally but only ~0.015 kg·m² rotationally, so the discrete-time
stability limit `D*dt/Lambda < 2` binds on rotation long before translation.
The controller warns above `0.5`; values approaching `1.0` over-damp the wrist
and can trip a Cartesian reflex.

**`wrench_saturation` depends on your feedforward.** Leave it on for
pose-only work — it keeps a transient at high stiffness from exceeding the
robot's collision thresholds and aborting the motion. Turn it off when the
feedforward wrench is itself close to `max_torque`: the clamp scales the whole
block to preserve direction, so a large feedforward would drag the restoring
moment down with it. `tau_limits` stays active either way.

**`delta_tau_max`** was kept conservative for the FR3 to avoid reflex errors.
On a robot with less aggressive collision detection it can usually be relaxed,
which reduces phase lag in the torque command.

---

## Test scripts

Not installed by CMake — run them directly with the workspace sourced.

| script | what it does |
|---|---|
| `test_pose.py` | step the EE by a delta on one or more axes from its current pose |
| `test_impedance.py` | circular trajectory tracking, orientation held; ramps in so the first command never steps |
| `test_force.py` | publish a constant feedforward wrench and watch the response |
| `test_pose_and_force.py` | move to a target pose while applying a feedforward wrench, then zero the wrench and hold |

```bash
python3 scripts/test_pose.py --axis z --delta 0.10
python3 scripts/test_impedance.py --radius 0.05 --freq 0.15 --plane xz
python3 scripts/test_force.py --fz 5.0 --duration 3.0
```

---

## Porting to another arm

The control law is robot-agnostic; what is Franka-specific is the URDF, the
joint naming and the limits.

1. **URDF.** The package bundles `urdf/fr3.urdf` and loads it from its own share
   directory in `on_configure`. Replace it, or change the path there.
2. **End-effector frame.** `on_configure` looks up `fr3_hand_tcp` and falls back
   to the last frame in the model. Point it at your tool frame.
3. **Joint count and names.** `num_joints_` is 7; joint names are built as
   `<arm_prefix>_<arm_id>_joint<N>`. If your robot does not follow that pattern
   (UR does not), change `buildJointPrefix()` and the two interface
   configuration functions.
4. **`tau_limits`.** Set to your robot's joint torque limits.
5. **Retune.** `screw_stiffness`, `alpha` and `delta_tau_max` are all sized for
   the FR3. The FR3's collision detection is unusually eager, so the filtering
   and rate limits here are more conservative than most arms need.

You can drop `franka.repos`, `libfranka` and `franka_description` entirely —
nothing in this package links against them. `dq_operations` is the only
dependency that does not come from `rosdep`, and it is robot-independent.

A more detailed walkthrough of the same steps, written for a UR, is in the
[dq_dmp_ros2_shared](https://github.com/BastienMourin/dq_dmp_ros2_shared) docs.
