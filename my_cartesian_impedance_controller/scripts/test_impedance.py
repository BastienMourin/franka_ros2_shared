#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import time
import os

class ImpedanceTestNode(Node):
    def __init__(self):
        super().__init__('impedance_tester')

        # --- Test Configuration ---
        self.duration = 15.0       # Test duration (seconds)
        self.radius = 0.1          # Circle radius (m)
        self.center = np.array([0.3, 0.0, 0.5]) # Circle center (x, y, z)
        self.freq = 0.2            # Circle frequency (Hz)
        
        # --- Communication ---
        self.publisher_ = self.create_publisher(
            PoseStamped, 
            '/my_cartesian_impedance_controller/target_pose', 
            10)
        
        # To listen to the real robot position (via TF)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Data Storage ---
        self.times = []
        self.target_pos = [] # [x, y, z]
        self.actual_pos = [] # [x, y, z]
        self.errors = []     # Error norm

        self.start_time = time.time()
        self.timer = self.create_timer(0.02, self.control_loop) # 50 Hz
        
        self.get_logger().info("Starting trajectory test (Circle)...")

    def get_actual_pose(self):
        """Retrieve the actual end-effector position via TF"""
        try:
            # Look up transform from base (link0) to end-effector (fr3_link8 or fr3_hand_tcp)
            # Adapt 'fr3_link8' according to your URDF!
            t = self.tf_buffer.lookup_transform(
                'fr3_link0', 
                'fr3_link8', 
                rclpy.time.Time())
            
            return np.array([
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z
            ])
        except Exception as e:
            # self.get_logger().warn(f"TF non disponible: {e}")
            return None

    def control_loop(self):
        current_time = time.time() - self.start_time
        
        # 1. End of test?
        if current_time > self.duration:
            self.stop_and_plot()
            return

        # 2. Trajectory Generation (Circle in YZ plane)
        # x fixe, y = cos, z = sin
        x_des = self.center[0]
        y_des = self.center[1] + self.radius * np.cos(2 * np.pi * self.freq * current_time)
        z_des = self.center[2] + self.radius * np.sin(2 * np.pi * self.freq * current_time)
        
        # Fixed orientation (pointing down/forward)
        # Quaternion for x=1 (facing forward) or adjusted as needed
        # Here we keep the neutral orientation provided in your example
        ori_x, ori_y, ori_z, ori_w = 1.0, 0.0, 0.0, 0.0 

        # 3. Send command
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'fr3_link0'
        msg.pose.position.x = x_des
        msg.pose.position.y = y_des
        msg.pose.position.z = z_des
        msg.pose.orientation.x = ori_x
        msg.pose.orientation.y = ori_y
        msg.pose.orientation.z = ori_z
        msg.pose.orientation.w = ori_w
        
        self.publisher_.publish(msg)

        # 4. Data recording
        actual = self.get_actual_pose()
        
        if actual is not None:
            self.times.append(current_time)
            self.target_pos.append([x_des, y_des, z_des])
            self.actual_pos.append(actual)
            
            # Euclidean error calculation
            err = np.linalg.norm(np.array([x_des, y_des, z_des]) - actual)
            self.errors.append(err)

    def stop_and_plot(self):
        self.timer.cancel()
        self.get_logger().info("Test completed. Generating plots...")
        
        # Conversion en numpy arrays
        times = np.array(self.times)
        targets = np.array(self.target_pos)
        actuals = np.array(self.actual_pos)
        errors = np.array(self.errors)

        # Compute metrics
        rmse = np.sqrt(np.mean(errors**2))
        max_err = np.max(errors)
        
        print(f"--- RESULTS ---")
        print(f"RMSE (Root Mean Square Error): {rmse*1000:.2f} mm")
        print(f"Max Error: {max_err*1000:.2f} mm")

        # --- FIGURE 1: XYZ Trajectory Tracking ---
        fig1, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
        labels = ['X', 'Y', 'Z']

        # Common limits
        y_min = min(targets.min(), actuals.min())
        y_max = max(targets.max(), actuals.max())

        for i in range(3):
            axs[i].plot(times, targets[:, i], 'r--', label='Desired', linewidth=2)
            axs[i].plot(times, actuals[:, i], 'b-', label='Actual', linewidth=1.5)
            axs[i].set_ylabel(f'Position {labels[i]} [m]')
            axs[i].set_ylim(y_min, y_max)  # identical scale
            axs[i].grid(True)
            axs[i].legend()
            
        axs[0].set_title(f'Trajectory Tracking (Impedance)\nRMSE: {rmse*1000:.1f}mm')
        axs[2].set_xlabel('Time [s]')
        plt.savefig('result_tracking_xyz.png')

        # --- FIGURE 2: Error over time ---
        plt.figure(figsize=(10, 6))
        plt.plot(times, errors * 1000, 'k-', linewidth=1.5)
        plt.fill_between(times, errors * 1000, color='red', alpha=0.1)
        plt.title('Euclidean Position Error')
        plt.ylabel('Error [mm]')
        plt.xlabel('Time [s]')
        plt.grid(True)
        plt.axhline(y=rmse*1000, color='g', linestyle='--', label=f'RMSE ({rmse*1000:.1f}mm)')
        plt.legend()
        plt.savefig('result_error.png')
    
        # --- FIGURE 3: 3D View (Spatial Trajectory) ---
        fig3 = plt.figure(figsize=(10, 8))
        ax = fig3.add_subplot(111, projection='3d')

        ax.plot(targets[:,0], targets[:,1], targets[:,2], 'r--', label='Target')
        ax.plot(actuals[:,0], actuals[:,1], actuals[:,2], 'b-', label='Actual')

        # === IDENTICAL SCALE ===
        all_data = np.vstack((targets, actuals))

        x_min, y_min, z_min = all_data.min(axis=0)
        x_max, y_max, z_max = all_data.max(axis=0)

        x_mid = (x_max + x_min) / 2
        y_mid = (y_max + y_min) / 2
        z_mid = (z_max + z_min) / 2

        max_range = max(
            x_max - x_min,
            y_max - y_min,
            z_max - z_min
        ) / 2

        ax.set_xlim(x_mid - max_range, x_mid + max_range)
        ax.set_ylim(y_mid - max_range, y_mid + max_range)
        ax.set_zlim(z_mid - max_range, z_mid + max_range)

        ax.set_box_aspect([1, 1, 1])  # ← crucial

        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.set_title('Spatial Trajectory')
        ax.legend()

        plt.savefig('result_trajectory_3d.png')

        self.get_logger().info("Plots saved (result_*.png). Closing.")
        plt.show()
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = ImpedanceTestNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
