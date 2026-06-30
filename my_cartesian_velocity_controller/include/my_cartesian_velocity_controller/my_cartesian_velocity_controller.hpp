#pragma once

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include <string>

// Pinocchio
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <std_msgs/msg/string.hpp>

using CallbackReturn = controller_interface::CallbackReturn;

namespace my_cartesian_velocity_controller {

class MyCartesianVelocityController : public controller_interface::ControllerInterface {
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
    std::string arm_id_    = "fr3";
    std::string arm_prefix_ = "";
    std::string buildJointPrefix() const;

    // --- EMA smoothing (0 < alpha <= 1; lower = slower/smoother) ---
    double alpha_ = 0.005;

    // --- Rotation input scaling (multiplies incoming angular velocity) ---
    double rot_scale_ = 1.0;

    // --- Pinocchio ---
    pinocchio::Model model_;
    pinocchio::Data data_;
    Eigen::VectorXd q_;
    Eigen::VectorXd v_des_;
    Eigen::MatrixXd J_;
    Eigen::VectorXd q_init_;
    bool model_loaded_ = false;
    std::string urdf_string_;
    bool is_first_update_ = true;
    int num_joints_      = 7;
    Eigen::VectorXd q_dot_prev_;
    
    // Real-time communication (commands received via Topic)
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_command_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> input_cmd_;
};

} // namespace