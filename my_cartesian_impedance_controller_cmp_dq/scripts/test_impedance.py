#!/usr/bin/env python3
"""
Circular trajectory tracking test for the DQ Cartesian impedance controller.

The circle is built around the EE pose captured at start-up, and the angular
rate is ramped in, so the first command never steps away from where the robot
actually is. Orientation is held at the start orientation.

Usage (workspace must be sourced):
    python3 test_impedance.py
    python3 test_impedance.py --radius 0.05 --freq 0.15 --plane xz
    python3 test_impedance.py --duration 20 --prefix run2_
"""
import argparse
import sys
import time

import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
from scipy.spatial.transform import Rotation as R
import tf2_ros


CONTROLLER = "my_cartesian_impedance_controller_cmp_dq"

# Basis vectors spanning each circle plane: (u, v) with p = c + r*(cos*u + sin*v)
PLANES = {
    "yz": (np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])),
    "xz": (np.array([1.0, 0.0, 0.0]), np.array([0.0, 0.0, 1.0])),
    "xy": (np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])),
}


class ImpedanceTestNode(Node):
    def __init__(self, args):
        super().__init__("impedance_tester")

        self.duration = args.duration
        self.radius = args.radius
        self.freq = args.freq
        self.ramp = args.ramp
        self.prefix = args.prefix
        self.base = args.base
        self.ee = args.ee
        self.u, self.v = PLANES[args.plane]

        self._pub = self.create_publisher(
            PoseStamped, f"/{CONTROLLER}/target_pose", 10)
        self._sub_err = self.create_subscription(
            WrenchStamped, f"/{CONTROLLER}/cartesian_error",
            self._on_screw_error, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Captured on the first successful TF lookup.
        self.center = None
        self.start_quat = None

        self.times = []
        self.target_pos = []
        self.actual_pos = []
        self.pos_errors = []   # euclidean, m
        self.ori_errors = []   # angle between target and actual orientation, rad
        self.screw_lin = []    # controller's own DQ screw error, sampled
        self.screw_ang = []

        self._last_screw = np.zeros(6)

        self.start_time = None
        self.timer = self.create_timer(0.02, self.control_loop)  # 50 Hz

        self.get_logger().info(
            f"Waiting for TF {self.base}→{self.ee} to anchor the circle...")

    def _on_screw_error(self, msg: WrenchStamped):
        w = msg.wrench
        self._last_screw = np.array([
            w.force.x, w.force.y, w.force.z,
            w.torque.x, w.torque.y, w.torque.z,
        ])

    def get_actual_pose(self):
        """Return (position, quaternion xyzw) of the EE, or None if no TF yet."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.base, self.ee, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05))
        except tf2_ros.TransformException:
            return None

        tr, rot = t.transform.translation, t.transform.rotation
        return (np.array([tr.x, tr.y, tr.z]),
                np.array([rot.x, rot.y, rot.z, rot.w]))

    def _phase(self, t):
        """Angle with theta(0)=0 and dtheta/dt(0)=0, easing into rate omega."""
        omega = 2.0 * np.pi * self.freq
        if t < self.ramp:
            return omega * t * t / (2.0 * self.ramp)
        return omega * (t - self.ramp / 2.0)

    def control_loop(self):
        pose = self.get_actual_pose()
        if pose is None:
            return
        actual_pos, actual_quat = pose

        # Anchor the circle so that p(0) == the EE pose we started from.
        if self.center is None:
            self.center = actual_pos - self.radius * self.u
            self.start_quat = actual_quat
            self.start_time = time.time()
            self.get_logger().info(
                f"Anchored at {np.array2string(actual_pos, precision=4)}; "
                f"running {self.duration:.0f}s circle "
                f"(r={self.radius} m, f={self.freq} Hz)")
            return

        t = time.time() - self.start_time
        if t > self.duration:
            self.stop_and_plot()
            return

        theta = self._phase(t)
        target = self.center + self.radius * (np.cos(theta) * self.u +
                                              np.sin(theta) * self.v)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base
        msg.pose.position.x = float(target[0])
        msg.pose.position.y = float(target[1])
        msg.pose.position.z = float(target[2])
        msg.pose.orientation.x = float(self.start_quat[0])
        msg.pose.orientation.y = float(self.start_quat[1])
        msg.pose.orientation.z = float(self.start_quat[2])
        msg.pose.orientation.w = float(self.start_quat[3])
        self._pub.publish(msg)

        # Angle of the relative rotation target→actual.
        q_err = (R.from_quat(self.start_quat).inv() * R.from_quat(actual_quat))
        ori_err = np.linalg.norm(q_err.as_rotvec())

        self.times.append(t)
        self.target_pos.append(target)
        self.actual_pos.append(actual_pos)
        self.pos_errors.append(np.linalg.norm(target - actual_pos))
        self.ori_errors.append(ori_err)
        self.screw_lin.append(self._last_screw[:3].copy())
        self.screw_ang.append(self._last_screw[3:].copy())

    def stop_and_plot(self):
        self.timer.cancel()
        self.get_logger().info("Test completed. Generating plots...")

        times = np.array(self.times)
        targets = np.array(self.target_pos)
        actuals = np.array(self.actual_pos)
        errors = np.array(self.pos_errors)
        ori_errors = np.array(self.ori_errors)
        screw_lin = np.array(self.screw_lin)
        screw_ang = np.array(self.screw_ang)

        # Exclude the ramp-in from the metrics: it is not steady-state tracking.
        steady = times > self.ramp
        rmse = np.sqrt(np.mean(errors[steady] ** 2))
        max_err = np.max(errors[steady])
        ori_rmse = np.sqrt(np.mean(ori_errors[steady] ** 2))

        print("--- RESULTS (steady state, ramp excluded) ---")
        print(f"Position RMSE     : {rmse * 1000:.2f} mm")
        print(f"Position max error: {max_err * 1000:.2f} mm")
        print(f"Orientation RMSE  : {np.rad2deg(ori_rmse):.2f} deg")

        # --- FIGURE 1: XYZ tracking ---
        fig1, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
        y_min = min(targets.min(), actuals.min())
        y_max = max(targets.max(), actuals.max())
        for i, label in enumerate("XYZ"):
            axs[i].plot(times, targets[:, i], "r--", label="Desired", linewidth=2)
            axs[i].plot(times, actuals[:, i], "b-", label="Actual", linewidth=1.5)
            axs[i].axvspan(0, self.ramp, color="grey", alpha=0.15)
            axs[i].set_ylabel(f"Position {label} [m]")
            axs[i].set_ylim(y_min, y_max)
            axs[i].grid(True)
            axs[i].legend()
        axs[0].set_title(f"DQ Impedance Trajectory Tracking\nRMSE: {rmse * 1000:.1f} mm")
        axs[2].set_xlabel("Time [s]")
        fig1.savefig(f"{self.prefix}result_tracking_xyz.png")

        # --- FIGURE 2: position + orientation error ---
        fig2, (ax_p, ax_o) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        ax_p.plot(times, errors * 1000, "k-", linewidth=1.5)
        ax_p.fill_between(times, errors * 1000, color="red", alpha=0.1)
        ax_p.axhline(y=rmse * 1000, color="g", linestyle="--",
                     label=f"RMSE ({rmse * 1000:.1f} mm)")
        ax_p.axvspan(0, self.ramp, color="grey", alpha=0.15, label="ramp-in")
        ax_p.set_ylabel("Position error [mm]")
        ax_p.set_title("Tracking Error")
        ax_p.grid(True)
        ax_p.legend()

        ax_o.plot(times, np.rad2deg(ori_errors), "k-", linewidth=1.5)
        ax_o.fill_between(times, np.rad2deg(ori_errors), color="purple", alpha=0.1)
        ax_o.axvspan(0, self.ramp, color="grey", alpha=0.15)
        ax_o.set_ylabel("Orientation error [deg]")
        ax_o.set_xlabel("Time [s]")
        ax_o.grid(True)
        fig2.savefig(f"{self.prefix}result_error.png")

        # --- FIGURE 3: 3D view ---
        fig3 = plt.figure(figsize=(10, 8))
        ax = fig3.add_subplot(111, projection="3d")
        ax.plot(targets[:, 0], targets[:, 1], targets[:, 2], "r--", label="Target")
        ax.plot(actuals[:, 0], actuals[:, 1], actuals[:, 2], "b-", label="Actual")

        all_data = np.vstack((targets, actuals))
        mid = (all_data.max(axis=0) + all_data.min(axis=0)) / 2
        max_range = (all_data.max(axis=0) - all_data.min(axis=0)).max() / 2
        ax.set_xlim(mid[0] - max_range, mid[0] + max_range)
        ax.set_ylim(mid[1] - max_range, mid[1] + max_range)
        ax.set_zlim(mid[2] - max_range, mid[2] + max_range)
        ax.set_box_aspect([1, 1, 1])
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_zlabel("Z [m]")
        ax.set_title("Spatial Trajectory")
        ax.legend()
        fig3.savefig(f"{self.prefix}result_trajectory_3d.png")

        # --- FIGURE 4: controller's DQ screw error ---
        fig4, (ax_l, ax_a) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        for i, label in enumerate("xyz"):
            ax_l.plot(times, screw_lin[:, i] * 1000, label=f"v_{label}")
            ax_a.plot(times, np.rad2deg(screw_ang[:, i]), label=f"w_{label}")
        ax_l.set_ylabel("Linear screw error [mm]")
        ax_l.set_title(f"DQ screw error published on /{CONTROLLER}/cartesian_error")
        ax_l.grid(True)
        ax_l.legend()
        ax_a.set_ylabel("Angular screw error [deg]")
        ax_a.set_xlabel("Time [s]")
        ax_a.grid(True)
        ax_a.legend()
        fig4.savefig(f"{self.prefix}result_screw_error.png")

        self.get_logger().info(f"Plots saved ({self.prefix}result_*.png). Closing.")
        plt.show()
        raise SystemExit


def main():
    parser = argparse.ArgumentParser(
        description="Circle tracking test for the DQ Cartesian impedance controller.")
    parser.add_argument("--duration", type=float, default=15.0, help="Test duration [s]")
    parser.add_argument("--radius", type=float, default=0.1, help="Circle radius [m]")
    parser.add_argument("--freq", type=float, default=0.2, help="Circle frequency [Hz]")
    parser.add_argument("--ramp", type=float, default=2.0,
                        help="Angular-rate ramp-in time [s]")
    parser.add_argument("--plane", choices=list(PLANES), default="yz",
                        help="Plane of the circle (default: yz)")
    parser.add_argument("--base", default="fr3_link0", help="Base TF frame")
    parser.add_argument("--ee", default="fr3_hand_tcp",
                        help="EE TF frame (must match the controller's control frame)")
    parser.add_argument("--prefix", default="", help="Prefix for saved PNG filenames")
    args = parser.parse_args(sys.argv[1:])

    if args.ramp >= args.duration:
        parser.error("--ramp must be shorter than --duration")

    rclpy.init()
    node = ImpedanceTestNode(args)
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
