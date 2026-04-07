#include "my_cartesian_velocity_controller/my_cartesian_velocity_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

namespace my_cartesian_velocity_controller {

// ==========================================================
// Helper: build joint prefix from arm_prefix_ / arm_id_
// ==========================================================
std::string MyCartesianVelocityController::buildJointPrefix() const
{
    if (!arm_prefix_.empty()) {
        return arm_prefix_ + "_" + arm_id_;
    }
    // Fallback: try node namespace
    auto node = const_cast<MyCartesianVelocityController*>(this)->get_node();
    std::string ns = node ? node->get_namespace() : std::string("/");
    if (!ns.empty() && ns != "/") {
        if (ns.front() == '/') ns.erase(0, 1);
        return ns + "_" + arm_id_;
    }
    return arm_id_;  // e.g. "fr3"
}

// ==========================================================
// Interface configurations
// ==========================================================
controller_interface::InterfaceConfiguration
MyCartesianVelocityController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    const std::string prefix = buildJointPrefix();
    for (int i = 1; i <= num_joints_; ++i)
        config.names.push_back(prefix + "_joint" + std::to_string(i) + "/velocity");
    return config;
}

controller_interface::InterfaceConfiguration
MyCartesianVelocityController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    const std::string prefix = buildJointPrefix();
    for (int i = 1; i <= num_joints_; ++i)
        config.names.push_back(prefix + "_joint" + std::to_string(i) + "/position");
    return config;
}

// ==========================================================
// on_init: declare / read arm_id and arm_prefix parameters
// ==========================================================
CallbackReturn MyCartesianVelocityController::on_init()
{
    auto node = get_node();

    if (!node->has_parameter("arm_id"))
        node->declare_parameter<std::string>("arm_id", arm_id_);
    if (!node->has_parameter("arm_prefix"))
        node->declare_parameter<std::string>("arm_prefix", arm_prefix_);

    node->get_parameter("arm_id",     arm_id_);
    node->get_parameter("arm_prefix", arm_prefix_);

    RCLCPP_INFO(node->get_logger(),
                "on_init: arm_id='%s'  arm_prefix='%s'  joint_prefix='%s'",
                arm_id_.c_str(), arm_prefix_.c_str(), buildJointPrefix().c_str());

    return CallbackReturn::SUCCESS;
}

// ==========================================================
// on_configure
// ==========================================================
controller_interface::CallbackReturn MyCartesianVelocityController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_node()->get_logger(), "Configuration: Loading URDF from file...");

    try {
        std::string share_dir =
            ament_index_cpp::get_package_share_directory("my_cartesian_velocity_controller");
        std::string urdf_file_path = share_dir + "/urdf/fr3.urdf";

        RCLCPP_INFO(get_node()->get_logger(), "Target URDF path: %s", urdf_file_path.c_str());

        if (!std::filesystem::exists(urdf_file_path)) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "FATAL ERROR: File not found at: %s", urdf_file_path.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }

        pinocchio::urdf::buildModel(urdf_file_path, model_);
        data_ = pinocchio::Data(model_);

        RCLCPP_INFO(get_node()->get_logger(), "Success: Model loaded! Joints: %d", model_.nq);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "EXCEPTION during URDF loading: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    q_     = Eigen::VectorXd::Zero(model_.nq);
    v_des_ = Eigen::VectorXd::Zero(6);
    J_     = Eigen::MatrixXd::Zero(6, model_.nv);

    auto node = get_node();
    sub_command_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            input_cmd_.writeFromNonRT(msg);
        });

    return controller_interface::CallbackReturn::SUCCESS;
}

// ==========================================================
// on_activate
// ==========================================================
CallbackReturn MyCartesianVelocityController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    auto msg = std::make_shared<geometry_msgs::msg::Twist>();
    msg->linear.x = 0.0; msg->linear.y = 0.0; msg->linear.z = 0.0;
    msg->angular.x = 0.0; msg->angular.y = 0.0; msg->angular.z = 0.0;
    input_cmd_.writeFromNonRT(msg);

    for (int i = 0; i < num_joints_; ++i) {
        if (std::isnan(state_interfaces_[i].get_value())) {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Error: Joint %d position is NaN!", i);
            return CallbackReturn::ERROR;
        }
        q_[i] = state_interfaces_[i].get_value();
    }

    q_init_          = q_;
    startup_weight_  = 0.0;
    is_first_update_ = true;

    RCLCPP_INFO(get_node()->get_logger(),
                "Controller activated (joint prefix: '%s'). Initial position captured.",
                buildJointPrefix().c_str());
    return CallbackReturn::SUCCESS;
}

// ==========================================================
// on_deactivate
// ==========================================================
CallbackReturn MyCartesianVelocityController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    return CallbackReturn::SUCCESS;
}

// ==========================================================
// update  (1000 Hz real-time loop)
// ==========================================================
controller_interface::return_type MyCartesianVelocityController::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // ----------------------------------------------------------------
    // STEP 1: Safety Counter (3 s wait at startup)
    // ----------------------------------------------------------------
    static int safety_counter = 0;
    if (is_first_update_) {
        safety_counter   = 0;
        is_first_update_ = false;
    }
    if (safety_counter < 3000) {
        ++safety_counter;
        for (int i = 0; i < num_joints_; ++i)
            command_interfaces_[i].set_value(0.0);
        return controller_interface::return_type::OK;
    }

    // ----------------------------------------------------------------
    // STEP 2: Pinocchio Computation
    // ----------------------------------------------------------------
    for (int i = 0; i < num_joints_; ++i)
        q_[i] = state_interfaces_[i].get_value();

    auto current_cmd = input_cmd_.readFromRT();
    if (current_cmd && *current_cmd) {
        v_des_ << (*current_cmd)->linear.x,  (*current_cmd)->linear.y,  (*current_cmd)->linear.z,
                  (*current_cmd)->angular.x, (*current_cmd)->angular.y, (*current_cmd)->angular.z;
    } else {
        v_des_.setZero();
    }

    pinocchio::computeJointJacobians(model_, data_, q_);
    pinocchio::getJointJacobian(model_, data_, 8, pinocchio::LOCAL_WORLD_ALIGNED, J_);
    Eigen::MatrixXd J_arm = J_.block(0, 0, 6, 7);

    double lambda = 0.1;
    Eigen::MatrixXd J_pinv = J_arm.transpose() *
        (J_arm * J_arm.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(6, 6)).inverse();

    Eigen::VectorXd q_dot_target = J_pinv * v_des_;

    // ----------------------------------------------------------------
    // STEP 3: Slew Rate Limiter
    // ----------------------------------------------------------------
    static Eigen::VectorXd q_dot_prev = Eigen::VectorXd::Zero(num_joints_);
    double max_change = 0.0001;

    Eigen::VectorXd q_dot_safe = q_dot_target;
    for (int i = 0; i < num_joints_; ++i) {
        double diff = q_dot_target[i] - q_dot_prev[i];
        diff = std::max(std::min(diff, max_change), -max_change);
        q_dot_safe[i] = q_dot_prev[i] + diff;
        q_dot_prev[i] = q_dot_safe[i];
    }

    // ----------------------------------------------------------------
    // STEP 4: Send commands
    // ----------------------------------------------------------------
    double v_max_abs = 0.5;
    for (int i = 0; i < num_joints_; ++i) {
        q_dot_safe[i] = std::max(std::min(q_dot_safe[i], v_max_abs), -v_max_abs);
        command_interfaces_[i].set_value(q_dot_safe[i]);
    }

    return controller_interface::return_type::OK;
}

} // namespace my_cartesian_velocity_controller

PLUGINLIB_EXPORT_CLASS(
    my_cartesian_velocity_controller::MyCartesianVelocityController,
    controller_interface::ControllerInterface)