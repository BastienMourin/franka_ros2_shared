# `_cmp_dq` vs `my_cartesian_impedance_controller`

A practical comparison for anyone already running
[`my_cartesian_impedance_controller`](https://github.com/BastienMourin/franka_ros2_shared/tree/my_cartesian_impedance_controller)
and deciding whether to move to this one.

Both controllers have the same skeleton: a `ros2_control` effort controller,
Pinocchio for kinematics and dynamics from a URDF, `~/target_pose` as the
equilibrium-pose input, a nullspace term, a torque slew limiter, and
`tau = J^T*F + tau_nullspace + coriolis`. If you ported the simple one, you have
already done most of the work.

Four things differ.

---

## 1. A feedforward wrench input

The simple controller computes its wrench from the pose error alone. This one
adds an input on `~/feedforward_wrench` (`geometry_msgs/WrenchStamped`) that is
summed straight into the control law:

```
F_des = screw_stiffness * error_screw  -  damping * velocity  +  F_ff
                                                                 ^^^^
```

This is the reason the variant exists. It lets you command a contact force
independently of the pose error, instead of getting force only as a by-product
of deliberately commanding a target the robot cannot reach.

**Why this may matter for the sole-detachment problem.** With stiffness alone,
force and stored energy come together: the only way to get more force is a
larger pose error, and that error is a loaded spring. When the sole releases,
that energy goes into moving the arm, and stiffening further to get more force
makes the release more violent rather than less.

A feedforward wrench separates the two. You can hold a low stiffness, keep the
commanded pose close to where the arm actually is, and carry the contact force
on `F_ff` instead. The same applies to a second robot holding the shoe: it can
push back with a commanded force rather than being levered into a pose error,
which is what makes two stiff arms fight each other.

Two caveats worth being clear about:

- **This only helps while the commanded pose stays near the arm.** If you drive
  a target that the arm cannot follow, the pose error grows again and so does
  the stored energy, feedforward or not. The benefit comes from keeping the
  error small, not from the feedforward term by itself.
- **The feedforward does not stop on its own.** When the sole releases, `F_ff`
  keeps pushing until you stop commanding it, so a release strategy is still
  needed — ramp it down, or cut it on a velocity threshold. That is an easier
  failure to handle than a spring release, because you are acting on something
  you command rather than on energy already stored.

This also makes a lateral or pendulum-like peeling motion straightforward: you
drive the pose along the peel path at low stiffness and let `F_ff` carry the
normal force, rather than encoding both into one stiff position target.

## 2. Dual-quaternion screw error

The simple controller builds two independent errors:

```cpp
err_pos = p_des - p_curr;
err_rot = log3(R_des * R_curr^T);
error_6d << err_pos, err_rot;
```

This one composes the two poses as unit dual quaternions and decomposes the
result into screw parameters:

```cpp
dq_error = dq_des * conj(dq_curr);
dqToScrewParameters(dq_error, theta, d, l, m);
w_e = l*theta;            // angular
v_e = l*d + m*theta;      // linear
```

Translation and rotation are then driven as one coupled screw motion. It also
handles the shortest-path flip explicitly when the rotation error exceeds π.

In practice this is a convenience, not a requirement — it matters because our
DMP represents trajectories the same way. **If you are only after the
feedforward input, you can skip this part** and keep the simple controller's
`log3` error.

## 3. One scalar stiffness, damping derived from it

| | simple | `_cmp_dq` |
|---|---|---|
| stiffness | `trans_stiffness`, `rot_stiffness` | `screw_stiffness` (one scalar, all six components) |
| damping | `2*sqrt(trans)`, `2*sqrt(rot)` | `2*sqrt(K)` and `rot_damping_ratio * 2*sqrt(K)` |
| computed | once, in `on_configure` | every cycle, filtered |
| live tunable | no | yes, via `ros2 param set` |

The practical difference is that damping **follows** the stiffness here. In the
simple controller the two stiffnesses and their damping are fixed at configure
time, so changing stiffness means re-deriving damping and restarting. Here
the damping ratio `zeta = D / (2*sqrt(K*Lambda))` is independent of `K`, so you
can sweep `screw_stiffness` live and the damping ratio holds.

`rot_damping_ratio` exists because rotation destabilises first: apparent inertia
at the end-effector is a few kg translationally but only ~0.015 kg·m²
rotationally, so the discrete-time limit `D*dt/Lambda < 2` binds there long
before it binds on translation. Tying rotational damping to the same `2*sqrt(K)`
as translation over-damps the wrist and trips a Cartesian reflex. `0.2` is the
tested value on an FR3.

Gains also ramp rather than step: a `ros2 param set` feeds a filtered target, so
it eases in over many cycles instead of landing in one 1 ms cycle.

## 4. The safety guards are enabled

The simple controller has these written but commented out, so it runs with the
slew limiter only. Here they are live:

| guard | simple | `_cmp_dq` |
|---|---|---|
| torque slew limiter | on | on |
| Cartesian wrench clamp | commented out | `wrench_saturation`, direction-preserving |
| per-joint torque clamp | commented out | always on, `tau_limits` |
| soft-start ramp | commented out | not present |
| startup delay | commented out | not present |

The wrench clamp scales the force and torque blocks so the wrench keeps its
direction rather than clipping per axis. It earns its place at high stiffness:
without it, a transient can exceed the robot's collision thresholds and abort
the motion with a reflex.

It ships **disabled** (`wrench_saturation: false`) because our feedforward
wrench peaks near `max_torque`, and the clamp scales the whole block — a large
`F_ff` would drag the restoring moment down with it and distort the commanded
direction. Turn it on for pose-only work.

There are also four diagnostic topics the simple controller does not have
(`~/cartesian_error`, `~/wrench_cmd`, `~/tau_cmd`, `~/dq_state`), which is how
you tell whether either clamp is actually binding during a run. See the
[README](README.md).

---

## Worth patching in the simple controller either way

While debugging this variant we hit a numerical instability in the
desired-orientation filter, and **the same code pattern is in
`my_cartesian_impedance_controller`**:

```cpp
Eigen::Quaterniond q_current_des(M_des_.rotation());
Eigen::Quaterniond q_target_raw(M_des_raw_.rotation());
M_des_.rotation() = q_current_des.slerp(alpha_, q_target_raw).toRotationMatrix();
```

Every cycle round-trips matrix → quaternion → slerp → matrix, and nothing in
that loop renormalises: Eigen's `slerp` does not, and the filter state is read
back out of a rotation matrix each time rather than kept as a quaternion. So a
small norm error is never corrected, and it is fed back in on the next cycle.

The amplifier is a *stationary* target. As the commanded orientation stops
changing, the rotation between the two quaternions goes to zero, and `slerp`
divides by the sine of that angle. The closer the target is to standing still,
the more the existing error is magnified. The error therefore grows
geometrically: it sits at round-off level for a long time, then rises very
quickly over a handful of milliseconds, the filtered orientation stops being a
valid rotation, and the resulting bogus orientation error commands a large
wrench.

A controller holding a pose is exactly the stationary case, so a long hold is
the risky situation, not a fast trajectory.

The fix is to keep the filter state as a quaternion and renormalise it, never
reading it back out of the matrix:

```cpp
// member, not a local:
Eigen::Quaterniond q_des_filtered_;

Eigen::Quaterniond q_target_raw(M_des_raw_.rotation());
q_target_raw.normalize();
q_des_filtered_ = q_des_filtered_.slerp(alpha_, q_target_raw);
q_des_filtered_.normalize();                       // <- the important line
M_des_.rotation() = q_des_filtered_.toRotationMatrix();
```

Initialise `q_des_filtered_` from the current pose in `on_activate` and on the
first update. This controller does it that way; the simple one does not yet.

---

## Which to use

**Stay on the simple controller** if you only need pose tracking. It is smaller,
has one less dependency, and you already have it tuned.

**Move to this one** if you want to command contact force directly rather than
through a deliberate pose error — which, for pressing and peeling tasks, is
usually the better-behaved way to get force.

## How to adopt it

If you have already ported the simple controller to your own arm, **do not
re-port from this package** — you would have to redo the joint names, the DOF
count, the URDF and whatever else your platform needed. Keep your file and swap
in the control law instead.

1. Clone [`dq_operations`](https://github.com/BastienMourin/dq_operations) into
   your workspace `src/`, and add it to your `package.xml` and `CMakeLists.txt`
   (two lines each).
2. **Replace the error block.** `err_pos` / `log3(R_err)` becomes
   `dq_des * conj(dq_curr)` decomposed into screw parameters — section 4 of
   `update()`.
3. **Replace the impedance law.** `Kp.cwiseProduct(error) - Kd.cwiseProduct(v)`
   becomes `screw_stiffness * error_screw - d_diag * v + F_ff`, with damping
   derived from the stiffness — section 5.
4. **Add the `~/feedforward_wrench` subscriber**, its `F_ff_` member and mutex, and
   swap `trans_stiffness`/`rot_stiffness` for
   `screw_stiffness`/`rot_damping_ratio` in the parameter block.
5. **Apply the orientation-filter fix** below.

That is roughly 90 lines of edits, about 50 of which are the error and control
law themselves. Everything platform-specific stays outside the region you are
touching — in particular the dual-quaternion error works purely on Cartesian
poses, so it never sees the joint count and 6 vs 7 DOF does not enter it.

Skip the four diagnostic publishers (~55 lines) unless you want them, and note
that the simple controller already contains the wrench and per-joint torque
clamps, commented out — you can just uncomment them.

**A smaller first step.** Steps 1, 3 and 4 without the dual-quaternion error —
keeping your existing `log3` error — is around ten lines and gets you the
feedforward wrench on its own. The screw error is a convenience, not a
requirement, so this is a reasonable way to test whether commanding force
directly solves your problem before taking on the rest.
