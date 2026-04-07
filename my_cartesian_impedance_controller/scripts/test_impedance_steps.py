#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import numpy as np
import matplotlib.pyplot as plt
import time

class ImpedanceStepTestNode(Node):
    def __init__(self):
        super().__init__('impedance_step_tester')

        # --- Waypoint Definitions (Step Commands) ---
        # Format: [x, y, z, wait_time_in_seconds]
        # Start from a neutral position, make 5 to 10 cm jumps, 
        # and wait 4 seconds each time to observe stabilization.
        self.waypoints = [
            [0.40,  0.00,  0.50,  4.0], # 1. Initial Position (Rest)
            [0.50,  0.00,  0.50,  4.0], # 2. Step in X (+10 cm)
            [0.50,  0.10,  0.50,  4.0], # 3. Step in Y (+10 cm)
            [0.50,  0.10,  0.40,  4.0], # 4. Step in Z (-10 cm)
            [0.40, -0.10,  0.50,  5.0], # 5. Combined step (Return + Diagonal)
        ]
        
        self.current_wp_index = 0
        self.time_at_last_wp = time.time()
        
        # Automatic total duration calculation
        self.duration = sum([wp[3] for wp in self.waypoints])
        
        # --- ROS Communication ---
        self.publisher_ = self.create_publisher(
            PoseStamped, 
            '/my_cartesian_impedance_controller/target_pose', 
            10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Data Storage ---
        self.times = []
        self.target_pos = [] 
        self.actual_pos = [] 

        self.start_time = time.time()
        self.timer = self.create_timer(0.02, self.control_loop) # 50 Hz
        
        self.get_logger().info(f"Starting step test (Expected duration: {self.duration}s)...")

    def get_actual_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('fr3_link0', 'fr3_link8', rclpy.time.Time())
            return np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        except Exception:
            return None

    def control_loop(self):
        current_time = time.time() - self.start_time
        time_since_last_wp = time.time() - self.time_at_last_wp
        
        # 1. End of test?
        if self.current_wp_index >= len(self.waypoints):
            self.stop_and_plot()
            return

        # 2. State machine management (Switch to next waypoint)
        current_wp = self.waypoints[self.current_wp_index]
        wait_time = current_wp[3]
        
        if time_since_last_wp >= wait_time:
            self.current_wp_index += 1
            self.time_at_last_wp = time.time()
            if self.current_wp_index >= len(self.waypoints):
                self.stop_and_plot()
                return
            current_wp = self.waypoints[self.current_wp_index]
            self.get_logger().info(f"-> New step: X={current_wp[0]:.2f}, Y={current_wp[1]:.2f}, Z={current_wp[2]:.2f}")

        # Current target
        x_des, y_des, z_des = current_wp[0], current_wp[1], current_wp[2]

        # 3. Send command
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'fr3_link0'
        msg.pose.position.x = float(x_des)
        msg.pose.position.y = float(y_des)
        msg.pose.position.z = float(z_des)
        # Fixed orientation pointing down/forward
        msg.pose.orientation.x = 1.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 0.0
        
        self.publisher_.publish(msg)

        # 4. Data recording
        actual = self.get_actual_pose()
        if actual is not None:
            self.times.append(current_time)
            self.target_pos.append([x_des, y_des, z_des])
            self.actual_pos.append(actual)

    def stop_and_plot(self):
        self.timer.cancel()
        self.get_logger().info("Test completed. Generating step response plot...")
        
        times = np.array(self.times)
        targets = np.array(self.target_pos)
        actuals = np.array(self.actual_pos)

        # --- FIGURE: Step Response ---
        fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        labels = ['X', 'Y', 'Z']
        colors = ['tab:red', 'tab:green', 'tab:blue']

        for i in range(3):
            # Dashed line for setpoint (perfect step)
            axs[i].plot(times, targets[:, i], color='black', linestyle='--', linewidth=2, label='Setpoint (Step)')
            # Solid line for robot response
            axs[i].plot(times, actuals[:, i], color=colors[i], linewidth=2.5, label='Robot Response')
            
            # Aesthetics
            axs[i].set_ylabel(f'Position {labels[i]} [m]', fontweight='bold')
            axs[i].grid(True, linestyle=':', alpha=0.7)
            axs[i].legend(loc='upper right')
            
            # Smart Y-axis zoom to see transient response
            margin = 0.05
            axs[i].set_ylim(min(targets[:, i].min(), actuals[:, i].min()) - margin, 
                            max(targets[:, i].max(), actuals[:, i].max()) + margin)

        axs[0].set_title('Impedance Controller Step Response Analysis', fontsize=14, fontweight='bold')
        axs[2].set_xlabel('Time [s]', fontweight='bold')
        
        plt.tight_layout()
        plt.savefig('result_step_response.png', dpi=300)
        self.get_logger().info("Plot saved as 'result_step_response.png'.")
        plt.show()
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = ImpedanceStepTestNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()