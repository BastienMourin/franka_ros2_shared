#!/usr/bin/env python3
"""
Combined test: move the EE to a target pose while applying a feedforward force.

1. Looks up current EE pose via TF and publishes target_pose (delta on Z).
2. Simultaneously publishes a constant feedforward force for --duration seconds,
   then zeroes it so the robot holds the new pose under pure impedance.

Usage:
    python3 test_pose_and_force.py
    python3 test_pose_and_force.py --delta 0.05 --fz 3.0 --duration 5.0
    python3 test_pose_and_force.py --delta 0.10 --fx 2.0 --fz 4.0 --duration 3.0
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, WrenchStamped
import tf2_ros


CONTROLLER = "my_cartesian_impedance_controller_cmp_dq"

_POSE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
_SENSOR_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class PoseAndForceTest(Node):
    def __init__(self, delta_z, fx, fy, fz, tx, ty, tz, duration, base, ee):
        super().__init__("pose_and_force_test")

        self._delta_z = delta_z
        self._fx, self._fy, self._fz = fx, fy, fz
        self._tx, self._ty, self._tz = tx, ty, tz
        self._duration = duration
        self._base = base
        self._ee = ee

        self._pose_sent = False
        self._force_start = None

        self._pub_pose = self.create_publisher(
            PoseStamped, f"/{CONTROLLER}/target_pose", _POSE_QOS)
        self._pub_force = self.create_publisher(
            WrenchStamped, f"/{CONTROLLER}/feedforward_wrench", _SENSOR_QOS)

        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)

        # Single timer: tries TF until pose is sent, then manages force loop.
        self._timer = self.create_timer(0.01, self._tick)

    def _tick(self):
        now = self.get_clock().now()

        # --- Phase 1: send pose once TF is available ---
        if not self._pose_sent:
            try:
                tf = self._tf_buf.lookup_transform(
                    self._base, self._ee,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1),
                )
            except tf2_ros.TransformException as exc:
                self.get_logger().info(
                    f"Waiting for TF {self._base}→{self._ee}: {exc}", once=True)
                return

            x = tf.transform.translation.x
            y = tf.transform.translation.y
            z = tf.transform.translation.z

            pose_msg = PoseStamped()
            pose_msg.header.stamp = now.to_msg()
            pose_msg.header.frame_id = self._base
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z + self._delta_z
            pose_msg.pose.orientation = tf.transform.rotation

            self._pub_pose.publish(pose_msg)
            self._pose_sent = True
            self._force_start = now

            self.get_logger().info(
                f"Target pose sent  : z {z:.4f} → {z + self._delta_z:.4f} m "
                f"(+{self._delta_z * 100:.1f} cm)"
            )
            self.get_logger().info(
                f"Feedforward force : F=[{self._fx}, {self._fy}, {self._fz}] N  "
                f"T=[{self._tx}, {self._ty}, {self._tz}] Nm  "
                f"for {self._duration:.1f} s"
            )
            return

        # --- Phase 2: publish force until duration expires ---
        elapsed = (now - self._force_start).nanoseconds * 1e-9

        force_msg = WrenchStamped()
        force_msg.header.stamp = now.to_msg()
        force_msg.header.frame_id = self._base

        if elapsed < self._duration:
            force_msg.wrench.force.x  = self._fx
            force_msg.wrench.force.y  = self._fy
            force_msg.wrench.force.z  = self._fz
            force_msg.wrench.torque.x = self._tx
            force_msg.wrench.torque.y = self._ty
            force_msg.wrench.torque.z = self._tz
        else:
            # Zero the feedforward — robot now holds target pose via impedance only.
            self.get_logger().info(
                "Force duration elapsed — zeroed feedforward, "
                "robot holds target pose under impedance control."
            )
            self._timer.cancel()

        self._pub_force.publish(force_msg)


def main():
    parser = argparse.ArgumentParser(
        description="Send a target pose + feedforward force simultaneously."
    )
    parser.add_argument("--delta", type=float, default=0.10,
                        help="Z offset in metres (default: 0.10 = 10 cm)")
    parser.add_argument("--fx", type=float, default=0.0, help="Force X [N]")
    parser.add_argument("--fy", type=float, default=0.0, help="Force Y [N]")
    parser.add_argument("--fz", type=float, default=5.0, help="Force Z [N] (default 5)")
    parser.add_argument("--tx", type=float, default=0.0, help="Torque X [Nm]")
    parser.add_argument("--ty", type=float, default=0.0, help="Torque Y [Nm]")
    parser.add_argument("--tz", type=float, default=0.0, help="Torque Z [Nm]")
    parser.add_argument("--duration", type=float, default=3.0,
                        help="How long to apply the force [s] (default 3)")
    parser.add_argument("--base", default="fr3_link0", help="Base TF frame")
    parser.add_argument("--ee",   default="fr3_hand_tcp", help="EE TF frame")
    args = parser.parse_args(sys.argv[1:])

    rclpy.init()
    node = PoseAndForceTest(
        args.delta,
        args.fx, args.fy, args.fz,
        args.tx, args.ty, args.tz,
        args.duration,
        args.base, args.ee,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
