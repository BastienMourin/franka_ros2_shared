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
    std::string arm_id_     = "fr3";  // e.g. "fr3"
    std::string arm_prefix_ = "";     // e.g. "robot1" -> joint prefix = "robot1_fr3"
    std::string buildJointPrefix() const;

    // --- Pinocchio ---
    pinocchio::Model model_;
    pinocchio::Data data_;
    Eigen::VectorXd q_;      // Current joint position
    Eigen::VectorXd v_des_;  // Desired Cartesian velocity (6 dimensions)
    Eigen::MatrixXd J_;      //  Jacobian
    Eigen::VectorXd q_init_;   // Initial joint position
    bool model_loaded_ = false;
    std::string urdf_string_;
    double startup_weight_ = 0.0;
    bool is_first_update_ = true;
    int num_joints_      = 7;
    
    // Real-time communication (commands received via Topic)
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_command_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> input_cmd_;
};

} // namespace