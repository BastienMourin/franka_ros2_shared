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
    if (!node->has_parameter("alpha"))
        node->declare_parameter<double>("alpha", alpha_);
    if (!node->has_parameter("rot_scale"))
        node->declare_parameter<double>("rot_scale", rot_scale_);

    node->get_parameter("arm_id",     arm_id_);
    node->get_parameter("arm_prefix", arm_prefix_);
    node->get_parameter("alpha",      alpha_);
    node->get_parameter("rot_scale",  rot_scale_);
    alpha_     = std::max(1e-4, std::min(alpha_, 1.0));
    rot_scale_ = std::max(0.0, std::min(rot_scale_, 1.0));

    RCLCPP_INFO(node->get_logger(),
                "on_init: arm_id='%s'  arm_prefix='%s'  joint_prefix='%s'  alpha=%.4f  rot_scale=%.2f",
                arm_id_.c_str(), arm_prefix_.c_str(), buildJointPrefix().c_str(), alpha_, rot_scale_);

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
    is_first_update_ = true;
    q_dot_prev_      = Eigen::VectorXd::Zero(num_joints_);

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
    // STEP 1: Startup hold (3 s zero-velocity)
    // Gives the hardware interface time to fully stabilize after activation
    // before any motion command is sent. q_dot_prev_ is zero at this point
    // (reset in on_activate), so the EMA starts from a clean state.
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
    // STEP 2: Cartesian → joint velocity via damped Jacobian pseudoinverse
    // Read current joint positions, build the 6×7 end-effector Jacobian
    // with Pinocchio, then solve:
    //   q_dot = J^T (J J^T + λ²I)^-1 · v_des
    // The damping term λ² prevents velocity blowup near singularities
    // where J J^T would otherwise be ill-conditioned.
    // Tune damping_param_pinv (= λ²): larger → safer near singularities
    // but reduced tracking accuracy; smaller → better tracking but riskier.
    // ----------------------------------------------------------------
    for (int i = 0; i < num_joints_; ++i)
        q_[i] = state_interfaces_[i].get_value();

    auto current_cmd = input_cmd_.readFromRT();
    if (current_cmd && *current_cmd) {
        // rot_scale_ scales the angular command at input level so rotation can be
        // attenuated independently of translation (e.g. to stay within reflex limits).
        v_des_ << (*current_cmd)->linear.x,
                  (*current_cmd)->linear.y,
                  (*current_cmd)->linear.z,
                  rot_scale_ * (*current_cmd)->angular.x,
                  rot_scale_ * (*current_cmd)->angular.y,
                  rot_scale_ * (*current_cmd)->angular.z;
    } else {
        v_des_.setZero();
    }

    pinocchio::computeJointJacobians(model_, data_, q_);
    pinocchio::getJointJacobian(model_, data_, 8, pinocchio::LOCAL_WORLD_ALIGNED, J_);
    Eigen::MatrixXd J_arm = J_.block(0, 0, 6, 7);

    double damping_param_pinv = 0.01;  // = λ² directly (not λ). Increase toward 0.25 if cartesian_reflex near singularities.
    Eigen::MatrixXd J_pinv = J_arm.transpose() *
        (J_arm * J_arm.transpose() + damping_param_pinv * Eigen::MatrixXd::Identity(6, 6)).inverse();

    Eigen::VectorXd q_dot_target = J_pinv * v_des_;

    // ----------------------------------------------------------------
    // STEP 3: EMA smoothing + per-joint acceleration and velocity hard caps
    //
    // EMA low-pass filter prevents step changes in the command from reaching
    // the robot instantly. Time constant: tau ≈ -0.001 / ln(1 - alpha)
    //   alpha=0.005 → tau ≈ 200 ms   (default, conservative)
    //   alpha=0.01  → tau ≈ 100 ms
    //   alpha=0.1   → tau ≈  10 ms   (nearly instant)
    //
    // Hard acceleration cap on top of EMA: EMA alone is not enough because
    // a worst-case ±v_max step at alpha=0.005 still produces up to 26 rad/s²
    // on joint 1, exceeding joint 2's datasheet limit of 7.5 rad/s².
    // The cap clamps the per-step velocity change to accel_limit × dt.
    //
    // FR3 datasheet hard limits applied here (joints 1–7):
    //   vel   [rad/s]:  2.62  2.62  2.62  2.62  5.26  4.18  5.26
    //   accel [rad/s²]: 15.0   7.5  10.0  12.5  15.0  20.0  20.0
    // ----------------------------------------------------------------
    static constexpr double kVelLim[7]   = {2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26};
    static constexpr double kAccelLim[7] = {15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0};
    constexpr double dt = 0.001;  // 1 kHz control loop

    Eigen::VectorXd q_dot_safe(num_joints_);
    for (int i = 0; i < num_joints_; ++i) {
        double ema    = alpha_ * q_dot_target[i] + (1.0 - alpha_) * q_dot_prev_[i];
        double delta  = ema - q_dot_prev_[i];
        double max_d  = kAccelLim[i] * dt;
        delta         = std::max(std::min(delta, max_d), -max_d);
        q_dot_safe[i] = q_dot_prev_[i] + delta;
        q_dot_safe[i] = std::max(std::min(q_dot_safe[i], kVelLim[i]), -kVelLim[i]);
        q_dot_prev_[i] = q_dot_safe[i];
    }

    // ----------------------------------------------------------------
    // STEP 4: Write commands to the hardware interface
    // ----------------------------------------------------------------
    for (int i = 0; i < num_joints_; ++i) {
        command_interfaces_[i].set_value(q_dot_safe[i]);
    }

    return controller_interface::return_type::OK;
}

} // namespace my_cartesian_velocity_controller

PLUGINLIB_EXPORT_CLASS(
    my_cartesian_velocity_controller::MyCartesianVelocityController,
    controller_interface::ControllerInterface)