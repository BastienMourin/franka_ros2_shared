#include "my_cartesian_impedance_controller/my_cartesian_impedance_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <algorithm>
#include <cmath>

namespace my_cartesian_impedance_controller {

// ==========================================================
// Helper: build "fr3" (or "<prefix>_<id>") from parameters
// ==========================================================
std::string MyCartesianImpedanceController::buildJointPrefix() const
{
    if (!arm_prefix_.empty()) {
        return arm_prefix_ + "_" + arm_id_;
    }
    // Fallback: try node namespace
    auto node = const_cast<MyCartesianImpedanceController*>(this)->get_node();
    std::string ns = node ? node->get_namespace() : std::string("/");
    if (!ns.empty() && ns != "/") {
        if (ns.front() == '/') ns.erase(0, 1);
        return ns + "_" + arm_id_;
    }
    return arm_id_;   // e.g. "fr3"
}

// ==========================================================
// Interface configurations
// ==========================================================
controller_interface::InterfaceConfiguration
MyCartesianImpedanceController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    const std::string prefix = buildJointPrefix();
    for (int i = 1; i <= num_joints_; ++i)
        config.names.push_back(prefix + "_joint" + std::to_string(i) + "/effort");
    return config;
}

controller_interface::InterfaceConfiguration MyCartesianImpedanceController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    const std::string prefix = buildJointPrefix();
    for (int i = 1; i <= num_joints_; ++i) {
        config.names.push_back(prefix + "_joint" + std::to_string(i) + "/position");
        config.names.push_back(prefix + "_joint" + std::to_string(i) + "/velocity");
    }
    return config;
}

// ==========================================================
// on_init: declare / read arm_id and arm_prefix parameters
// ==========================================================
CallbackReturn MyCartesianImpedanceController::on_init()
{
    auto node = get_node();

    if (!node->has_parameter("arm_id"))
        node->declare_parameter<std::string>("arm_id", arm_id_);
    if (!node->has_parameter("arm_prefix"))
        node->declare_parameter<std::string>("arm_prefix", arm_prefix_);
    if (!node->has_parameter("trans_stiffness"))
        node->declare_parameter<double>("trans_stiffness", trans_stiffness_);
    if (!node->has_parameter("rot_stiffness"))
        node->declare_parameter<double>("rot_stiffness", rot_stiffness_);
    if (!node->has_parameter("alpha"))
        node->declare_parameter<double>("alpha", alpha_);
    if (!node->has_parameter("safety_cycles"))
        node->declare_parameter<int>("safety_cycles", safety_cycles_);
    if (!node->has_parameter("startup_increment"))
        node->declare_parameter<double>("startup_increment", startup_increment_);
    if (!node->has_parameter("delta_tau_max"))
        node->declare_parameter<double>("delta_tau_max", delta_tau_max_);
    if (!node->has_parameter("max_force"))
        node->declare_parameter<double>("max_force", max_force_);
    if (!node->has_parameter("max_torque"))
        node->declare_parameter<double>("max_torque", max_torque_);
    if (!node->has_parameter("nullspace_stiffness"))
        node->declare_parameter<double>("nullspace_stiffness", nullspace_stiffness);
    // if (!node->has_parameter("tau_limits"))
    //     node->declare_parameter<std::vector<double>>(
    //         "tau_limits", std::vector<double>(tau_limits_.begin(), tau_limits_.end()));

    node->get_parameter("arm_id",     arm_id_);
    node->get_parameter("arm_prefix", arm_prefix_);
    node->get_parameter("trans_stiffness", trans_stiffness_);
    node->get_parameter("rot_stiffness", rot_stiffness_);
    node->get_parameter("alpha", alpha_);
    node->get_parameter("safety_cycles", safety_cycles_);
    node->get_parameter("startup_increment", startup_increment_);
    node->get_parameter("delta_tau_max", delta_tau_max_);
    node->get_parameter("max_force", max_force_);
    node->get_parameter("max_torque", max_torque_);
    node->get_parameter("nullspace_stiffness", nullspace_stiffness);
    if (nullspace_stiffness < 0.0) {
        RCLCPP_WARN(node->get_logger(),
                    "Parameter 'nullspace_stiffness' must be >= 0. Clamping to 0.");
        nullspace_stiffness = 0.0;
    }

    // std::vector<double> tau_limits_vec;
    // node->get_parameter("tau_limits", tau_limits_vec);
    // if (tau_limits_vec.size() == static_cast<size_t>(num_joints_)) {
    //     std::copy_n(tau_limits_vec.begin(), num_joints_, tau_limits_.begin());
    // } else {
    //     RCLCPP_WARN(node->get_logger(),
    //                 "Parameter 'tau_limits' must contain %d values. Using defaults.",
    //                 num_joints_);
    // }

    RCLCPP_INFO(node->get_logger(),
                "on_init: arm_id='%s'  arm_prefix='%s'  joint_prefix='%s'",
                arm_id_.c_str(), arm_prefix_.c_str(), buildJointPrefix().c_str());

    return CallbackReturn::SUCCESS;
}

// ==========================================================
// on_configure
// ==========================================================
CallbackReturn MyCartesianImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    auto node = get_node();
    RCLCPP_INFO(node->get_logger(), "Loading URDF model...");

    try {
        std::string share_dir =
            ament_index_cpp::get_package_share_directory("my_cartesian_impedance_controller");
        std::string urdf_file = share_dir + "/urdf/fr3.urdf";

        if (!std::filesystem::exists(urdf_file)) {
            RCLCPP_ERROR(node->get_logger(), "URDF not found: %s", urdf_file.c_str());
            return CallbackReturn::ERROR;
        }

        pinocchio::urdf::buildModel(urdf_file, model_);
        data_ = pinocchio::Data(model_);

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Pinocchio error: %s", e.what());
        return CallbackReturn::ERROR;
    }

    ee_frame_id_ = model_.existFrame("fr3_hand_tcp")
                       ? model_.getFrameId("fr3_hand_tcp")
                       : model_.nframes - 1;

    // --- Memory allocation ---
    q_       = Eigen::VectorXd::Zero(model_.nq);
    v_       = Eigen::VectorXd::Zero(model_.nv);
    tau_J_   = Eigen::VectorXd::Zero(model_.nv);
    tau_null_= Eigen::VectorXd::Zero(model_.nv);
    tau_des_ = Eigen::VectorXd::Zero(model_.nv);
    coriolis_= Eigen::VectorXd::Zero(model_.nv);
    tau_cmd_ = Eigen::VectorXd::Zero(model_.nv);
    tau_prev_= Eigen::VectorXd::Zero(model_.nv);
    J_       = Eigen::MatrixXd::Zero(6, model_.nv);
    J_transpose_pinv_ = Eigen::MatrixXd::Zero(6, model_.nv);
    nullspace_projector_ = Eigen::MatrixXd::Identity(model_.nv, model_.nv);
    identity7_ = Eigen::MatrixXd::Identity(model_.nv, model_.nv);
    q_d_nullspace_ = Eigen::VectorXd::Zero(model_.nq);

    err_pos_    = Eigen::Vector3d::Zero();
    err_rot_    = Eigen::Vector3d::Zero();
    R_err_      = Eigen::Matrix3d::Identity();
    error_6d_   = Eigen::VectorXd::Zero(6);
    v_curr_cart_= Eigen::VectorXd::Zero(6);
    F_des_      = Eigen::VectorXd::Zero(6);

    // --- Impedance (critical damping) ---
    Kp_          = Eigen::VectorXd::Zero(6);
    Kd_          = Eigen::VectorXd::Zero(6);
    error_sum_   = Eigen::VectorXd::Zero(6);
    error_sum_clamp_ = Eigen::VectorXd::Zero(6);

    Kp_ << trans_stiffness_, trans_stiffness_, trans_stiffness_,
           rot_stiffness_,   rot_stiffness_,   rot_stiffness_;
    const double trans_damping = 2.0 * std::sqrt(trans_stiffness_);
    const double rot_damping = 2.0 * std::sqrt(rot_stiffness_);
    Kd_ << trans_damping, trans_damping, trans_damping,
           rot_damping,   rot_damping,   rot_damping;

    // --- Subscriber (namespaced via ~/target_pose) ---
    sub_target_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/target_pose", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            input_target_.writeFromNonRT(msg);
        });

    return CallbackReturn::SUCCESS;
}

// ==========================================================
// on_activate
// ==========================================================
CallbackReturn MyCartesianImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    for (int i = 0; i < num_joints_; ++i) {
        q_[i] = state_interfaces_[2 * i].get_value();
        v_[i] = state_interfaces_[2 * i + 1].get_value();
    }

    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);
    M_des_ = data_.oMf[ee_frame_id_];

    input_target_.reset();
    startup_weight_ = 0.0;
    is_first_update_ = true;
    tau_prev_.setZero();
    q_d_nullspace_ = q_;

    RCLCPP_INFO(get_node()->get_logger(),
                "Impedance controller activated (joint prefix: '%s').",
                buildJointPrefix().c_str());
    return CallbackReturn::SUCCESS;
}

// ==========================================================
// on_deactivate
// ==========================================================
CallbackReturn MyCartesianImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_node()->get_logger(), "Impedance controller deactivated.");
    return CallbackReturn::SUCCESS;
}

// ==========================================================
// update  (1000 Hz real-time loop)
// ==========================================================
controller_interface::return_type MyCartesianImpedanceController::update(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
    (void)time;

    // --- 1. Read robot state ---
    for (int i = 0; i < num_joints_; ++i) {
        q_[i] = state_interfaces_[2 * i].get_value();
        v_[i] = state_interfaces_[2 * i + 1].get_value();
    }

    pinocchio::forwardKinematics(model_, data_, q_, v_);
    pinocchio::updateFramePlacements(model_, data_);
    pinocchio::computeJointJacobians(model_, data_, q_);
    pinocchio::getFrameJacobian(model_, data_, ee_frame_id_,
                                pinocchio::LOCAL_WORLD_ALIGNED, J_);
    pinocchio::computeCoriolisMatrix(model_, data_, q_, v_);

    M_curr_      = data_.oMf[ee_frame_id_];
    v_curr_cart_ = J_ * v_;
    coriolis_.noalias() = data_.C * v_;

    // --- 2. Initialization & safety delay ---
    // static int safety_counter = 0;
    static pinocchio::SE3 M_des_raw_;

    if (is_first_update_) {
        // safety_counter = 0;
        tau_prev_.setZero();
        M_des_     = M_curr_;
        M_des_raw_ = M_curr_;
        is_first_update_ = false;
    }

    // Startup delay disabled (ROS1-like behavior):
    // if (safety_counter < safety_cycles_) {
    //     ++safety_counter;
    //     for (int i = 0; i < num_joints_; ++i)
    //         command_interfaces_[i].set_value(0.0);
    //     return controller_interface::return_type::OK;
    // }

    // --- 3. ROS setpoint ---
    auto current_target_msg = input_target_.readFromRT();
    if (current_target_msg && *current_target_msg) {
        auto& msg = *current_target_msg;
        Eigen::Quaterniond quat(msg->pose.orientation.w, msg->pose.orientation.x,
                                msg->pose.orientation.y, msg->pose.orientation.z);
        Eigen::Vector3d trans(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        M_des_raw_ = pinocchio::SE3(quat, trans);
    }

    // Low-pass filter
    M_des_.translation() =
        (1.0 - alpha_) * M_des_.translation() + alpha_ * M_des_raw_.translation();
    Eigen::Quaterniond q_current_des(M_des_.rotation());
    Eigen::Quaterniond q_target_raw(M_des_raw_.rotation());
    M_des_.rotation() = q_current_des.slerp(alpha_, q_target_raw).toRotationMatrix();

    // --- 4. Spatial error ---
    err_pos_  = M_des_.translation() - M_curr_.translation();
    R_err_    = M_des_.rotation() * M_curr_.rotation().transpose();
    err_rot_  = pinocchio::log3(R_err_);
    error_6d_ << err_pos_, err_rot_;

    // --- 5. Impedance law (PD) ---
    F_des_.noalias() = Kp_.cwiseProduct(error_6d_) - Kd_.cwiseProduct(v_curr_cart_);

    // Cartesian force/torque saturation disabled (ROS1-like behavior)
    // for (int i = 0; i < 3; ++i) {
    //     F_des_[i]   = std::max(std::min(F_des_[i],   max_force_),  -max_force_);
    //     F_des_[i+3] = std::max(std::min(F_des_[i+3], max_torque_), -max_torque_);
    // }

    tau_J_.noalias() = J_.transpose() * F_des_;

    constexpr double pseudo_inverse_damping = 1e-6;
    Eigen::Matrix<double, 6, 6> jj_t_damped =
        J_ * J_.transpose() +
        pseudo_inverse_damping * Eigen::Matrix<double, 6, 6>::Identity();
    J_transpose_pinv_.noalias() = jj_t_damped.ldlt().solve(J_);

    nullspace_projector_.noalias() = identity7_ - J_.transpose() * J_transpose_pinv_;

    const double nullspace_damping = 2.0 * std::sqrt(std::max(nullspace_stiffness, 0.0));
    Eigen::VectorXd nullspace_control =
        nullspace_stiffness * (q_d_nullspace_ - q_) - nullspace_damping * v_;
    tau_null_.noalias() = nullspace_projector_ * nullspace_control;

    tau_des_ = tau_J_ + tau_null_ + coriolis_;


    // --- 6. Final safety checks ---

    // A. Soft start disabled (ROS1-like behavior)
    // if (startup_weight_ < 1.0) startup_weight_ += startup_increment_;
    // tau_cmd_ = tau_des_ * startup_weight_;
    tau_cmd_ = tau_des_;

    // B. Slew rate limiter
    for (int i = 0; i < num_joints_; ++i) {
        double diff = tau_cmd_[i] - tau_prev_[i];
        diff = std::max(std::min(diff, delta_tau_max_), -delta_tau_max_);
        tau_cmd_[i]  = tau_prev_[i] + diff;
        tau_prev_[i] = tau_cmd_[i];
    }

    // C. Per-joint absolute torque limits disabled (ROS1-like behavior)
    // for (int i = 0; i < num_joints_; ++i) {
    //     command_interfaces_[i].set_value(
    //         std::max(std::min(tau_cmd_[i], tau_limits_[i]), -tau_limits_[i]));
    // }
    for (int i = 0; i < num_joints_; ++i) {
        command_interfaces_[i].set_value(tau_cmd_[i]);
    }

    return controller_interface::return_type::OK;
}

} // namespace my_cartesian_impedance_controller

PLUGINLIB_EXPORT_CLASS(
    my_cartesian_impedance_controller::MyCartesianImpedanceController,
    controller_interface::ControllerInterface)