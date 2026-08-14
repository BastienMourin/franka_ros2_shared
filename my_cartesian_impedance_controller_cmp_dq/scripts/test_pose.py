#!/usr/bin/env python3
"""
Move the EE by a delta along one or more axes from its current pose.

Usage (workspace must be sourced):
    python3 test_pose.py                        # +10 cm on Z (default)
    python3 test_pose.py --axis x --delta 0.05  # +5 cm on X
    python3 test_pose.py --axis y --delta -0.03 # -3 cm on Y
    python3 test_pose.py --dx 0.02 --dy -0.01 --dz 0.05  # multi-axis
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
import tf2_ros


CONTROLLER = "my_cartesian_impedance_controller_cmp_dq"


class AxisMoveTest(Node):
    def __init__(self, dx: float, dy: float, dz: float,
                 base_frame: str, ee_frame: str):
        super().__init__("axis_move_test")
        self._dx   = dx
        self._dy   = dy
        self._dz   = dz
        self._base = base_frame
        self._ee   = ee_frame
        self._done = False

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(
            PoseStamped, f"/{CONTROLLER}/target_pose", qos)
        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)
        self._timer  = self.create_timer(0.2, self._try_send)

    def _try_send(self):
        if self._done:
            return
        try:
            tf = self._tf_buf.lookup_transform(
                self._base, self._ee,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except tf2_ros.TransformException as exc:
            self.get_logger().info(f"Waiting for TF {self._base}→{self._ee}: {exc}")
            return

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        z = tf.transform.translation.z

        tx, ty, tz = x + self._dx, y + self._dy, z + self._dz

        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base
        msg.pose.position.x = tx
        msg.pose.position.y = ty
        msg.pose.position.z = tz
        msg.pose.orientation = tf.transform.rotation

        self._pub.publish(msg)

        def fmt(cur, tgt, d, axis):
            return f"{axis}: {cur:.4f} → {tgt:.4f}  ({d:+.4f} m)" if d != 0.0 else \
                   f"{axis}: {cur:.4f} (unchanged)"

        self.get_logger().info(
            f"\nCurrent EE  : x={x:.4f}  y={y:.4f}  z={z:.4f}\n"
            f"  {fmt(x, tx, self._dx, 'X')}\n"
            f"  {fmt(y, ty, self._dy, 'Y')}\n"
            f"  {fmt(z, tz, self._dz, 'Z')}\n"
            f"Published to /{CONTROLLER}/target_pose"
        )
        self._done = True
        self._timer.cancel()


def main():
    parser = argparse.ArgumentParser(
        description="Move EE along one or more Cartesian axes.")

    # Convenience: single axis + delta
    parser.add_argument("--axis", choices=["x", "y", "z"], default="z",
                        help="Axis to move along when using --delta (default: z)")
    parser.add_argument("--delta", type=float, default=0.10,
                        help="Offset in metres along --axis (default: 0.10)")

    # Fine-grained: explicit per-axis deltas (override --axis/--delta)
    parser.add_argument("--dx", type=float, default=None, help="X offset [m]")
    parser.add_argument("--dy", type=float, default=None, help="Y offset [m]")
    parser.add_argument("--dz", type=float, default=None, help="Z offset [m]")

    parser.add_argument("--base", default="fr3_link0",
                        help="Base TF frame (default: fr3_link0)")
    parser.add_argument("--ee",   default="fr3_hand_tcp",
                        help="End-effector TF frame (default: fr3_hand_tcp)")

    args = parser.parse_args(sys.argv[1:])

    # If any explicit dx/dy/dz given, use those; else map --axis/--delta
    if any(v is not None for v in (args.dx, args.dy, args.dz)):
        dx = args.dx or 0.0
        dy = args.dy or 0.0
        dz = args.dz or 0.0
    else:
        dx = args.delta if args.axis == "x" else 0.0
        dy = args.delta if args.axis == "y" else 0.0
        dz = args.delta if args.axis == "z" else 0.0

    rclpy.init()
    node = AxisMoveTest(dx, dy, dz, args.base, args.ee)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
