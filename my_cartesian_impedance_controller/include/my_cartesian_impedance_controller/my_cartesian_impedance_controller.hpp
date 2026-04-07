#pragma once

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> // Changed Twist -> Pose
#include <realtime_tools/realtime_buffer.hpp>
#include <array>
#include <memory>
#include <vector>

// Pinocchio
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

using CallbackReturn = controller_interface::CallbackReturn;

namespace my_cartesian_impedance_controller {

class MyCartesianImpedanceController : public controller_interface::ControllerInterface {
public:
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    
    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    // --- Multi-robot identification ---
    std::string arm_id_     = "fr3";  // e.g. "fr3"
    std::string arm_prefix_ = "";     // e.g. "robot1" -> joint prefix = "robot1_fr3"
    std::string buildJointPrefix() const;

    // --- Pinocchio & Robot ---
    pinocchio::Model model_;
    pinocchio::Data data_;
    int num_joints_ = 7;
    int ee_frame_id_ = -1;

    // --- State variables (Pre-allocated to avoid malloc) ---
    Eigen::VectorXd q_;      
    Eigen::VectorXd v_;      
    Eigen::VectorXd tau_J_;       // Raw computed torque
    Eigen::VectorXd tau_null_;    // Nullspace torque
    Eigen::VectorXd tau_des_;     // Desired torque before safety checks
    Eigen::VectorXd coriolis_;    // Coriolis term
    Eigen::VectorXd tau_cmd_;     // Final sent torque (filtered)
    Eigen::MatrixXd J_;           // Jacobian
    Eigen::MatrixXd J_transpose_pinv_;
    Eigen::MatrixXd nullspace_projector_;
    Eigen::MatrixXd identity7_;

    // --- Computation variables (Pre-allocated) ---
    Eigen::Vector3d err_pos_;
    Eigen::Matrix3d R_err_;
    Eigen::Vector3d err_rot_;
    Eigen::VectorXd error_6d_;
    Eigen::VectorXd v_curr_cart_;
    Eigen::VectorXd F_des_;       // Desired Cartesian force

    // --- Target ---
    pinocchio::SE3 M_des_;        // Desired pose
    pinocchio::SE3 M_curr_;       // Current pose

    // --- Gains ---
    Eigen::VectorXd Kp_;
    Eigen::VectorXd Ki_;
    Eigen::VectorXd Kd_;
    double nullspace_stiffness = 20.0;
    Eigen::VectorXd q_d_nullspace_;

    // --- Integrator ---
    Eigen::VectorXd error_sum_; // Error integral
    Eigen::VectorXd error_sum_clamp_; // To limit the integrator (anti-windup)

    // --- Safety & Filtering ---
    double startup_weight_ = 0.0;
    bool is_first_update_ = true;
    Eigen::VectorXd tau_prev_;    // For the Slew Rate Limiter (previous torque)

    // --- Parameters (existing values, configurable via YAML) ---
    double trans_stiffness_ = 200.0;
    double rot_stiffness_ = 25.0;
    double alpha_ = 0.15;
    int safety_cycles_ = 3000;
    double startup_increment_ = 0.0005;
    double delta_tau_max_ = 1.0;
    double max_force_ = 50.0;
    double max_torque_ = 15.0;
    std::array<double, 7> tau_limits_ = {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};

    // --- ROS 2 ---
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_target_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::PoseStamped>> input_target_;
};

} // namespace