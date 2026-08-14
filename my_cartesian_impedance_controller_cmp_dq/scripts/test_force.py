#!/usr/bin/env python3
"""
Publish a constant feedforward force/torque to the cartesian impedance controller.

The controller adds F_ff to its impedance output: F_des = Kp*error - Kd*v + F_ff.
Forces are in the world frame (fr3_link0 aligned).

Usage:
    python3 test_force.py --fz 5.0 --duration 3.0
    python3 test_force.py --fx 3.0 --fy 0 --fz 0 --tx 0 --ty 0 --tz 0 --duration 2.0
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import WrenchStamped


CONTROLLER = "my_cartesian_impedance_controller_cmp_dq"

# SensorDataQoS: BEST_EFFORT, VOLATILE, depth=5
_SENSOR_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class ForceTest(Node):
    def __init__(self, fx, fy, fz, tx, ty, tz, duration):
        super().__init__("force_test")
        self._fx, self._fy, self._fz = fx, fy, fz
        self._tx, self._ty, self._tz = tx, ty, tz
        self._duration = duration
        self._start_time = None

        self._pub = self.create_publisher(
            WrenchStamped,
            f"/{CONTROLLER}/feedforward_wrench",
            _SENSOR_QOS,
        )
        # Publish at 100 Hz for the duration, then send zero and stop.
        self._timer = self.create_timer(0.01, self._publish)

    def _publish(self):
        now = self.get_clock().now()

        if self._start_time is None:
            self._start_time = now
            self.get_logger().info(
                f"Applying force for {self._duration:.1f} s  →  "
                f"F=[{self._fx}, {self._fy}, {self._fz}] N  "
                f"T=[{self._tx}, {self._ty}, {self._tz}] Nm"
            )

        elapsed = (now - self._start_time).nanoseconds * 1e-9

        msg = WrenchStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "fr3_link0"

        if elapsed < self._duration:
            msg.wrench.force.x  = self._fx
            msg.wrench.force.y  = self._fy
            msg.wrench.force.z  = self._fz
            msg.wrench.torque.x = self._tx
            msg.wrench.torque.y = self._ty
            msg.wrench.torque.z = self._tz
        else:
            # Zero out force so the robot returns to impedance equilibrium
            self.get_logger().info("Duration elapsed — zeroing feedforward force.")
            self._timer.cancel()

        self._pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(
        description="Apply a constant feedforward Cartesian force/torque."
    )
    parser.add_argument("--fx", type=float, default=0.0, help="Force X [N]")
    parser.add_argument("--fy", type=float, default=0.0, help="Force Y [N]")
    parser.add_argument("--fz", type=float, default=5.0, help="Force Z [N] (default 5)")
    parser.add_argument("--tx", type=float, default=0.0, help="Torque X [Nm]")
    parser.add_argument("--ty", type=float, default=0.0, help="Torque Y [Nm]")
    parser.add_argument("--tz", type=float, default=0.0, help="Torque Z [Nm]")
    parser.add_argument("--duration", type=float, default=3.0,
                        help="How long to apply the force [s] (default 3)")
    args = parser.parse_args(sys.argv[1:])

    rclpy.init()
    node = ForceTest(
        args.fx, args.fy, args.fz,
        args.tx, args.ty, args.tz,
        args.duration,
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
