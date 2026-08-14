#include "my_cartesian_impedance_controller_cmp_dq/my_cartesian_impedance_controller_cmp_dq.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <filesystem>
#include <cmath>
#include <algorithm>

namespace my_cartesian_impedance_controller_cmp_dq {

// ==========================================================
// Helper
// ==========================================================
std::string MyCartesianImpedanceControllerCmpDQ::buildJointPrefix() const
{
    if (!arm_prefix_.empty())
        return arm_prefix_ + "_" + arm_id_;
    auto node = const_cast<MyCartesianImpedanceControllerCmpDQ*>(this)->get_node();
    std::string ns = node ? node->get_namespace() : std::string("/");
    if (!ns.empty() && ns != "/") {
        if (ns.front() == '/') ns.erase(0, 1);
        return ns + "_" + arm_id_;
    }
    return arm_id_;
}

// ==========================================================
// Interface configurations
// ==========================================================
controller_interface::InterfaceConfiguration
MyCartesianImpedanceControllerCmpDQ::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    const std::string prefix = buildJointPrefix();
    for (int i = 1; i <= num_joints_; ++i)
        cfg.names.push_back(prefix + "_joint" + std::to_string(i) + "/effort");
    return cfg;
}

controller_interface::InterfaceConfiguration
MyCartesianImpedanceControllerCmpDQ::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    const std::string prefix = buildJointPrefix();
    for (int i = 1; i <= num_joints_; ++i) {
        cfg.names.push_back(prefix + "_joint" + std::to_string(i) + "/position");
        cfg.names.push_back(prefix + "_joint" + std::to_string(i) + "/velocity");
    }
    return cfg;
}

// ==========================================================
// on_init: declare / read parameters
// ==========================================================
CallbackReturn MyCartesianImpedanceControllerCmpDQ::on_init()
{
    auto node = get_node();

    if (!node->has_parameter("arm_id"))
        node->declare_parameter<std::string>("arm_id", arm_id_);
    if (!node->has_parameter("arm_prefix"))
        node->declare_parameter<std::string>("arm_prefix", arm_prefix_);
    if (!node->has_parameter("screw_stiffness"))
        node->declare_parameter<double>("screw_stiffness", screw_stiffness_);
    if (!node->has_parameter("rot_damping_ratio"))
        node->declare_parameter<double>("rot_damping_ratio", rot_damping_ratio_);
    if (!node->has_parameter("alpha"))
        node->declare_parameter<double>("alpha", alpha_);
    if (!node->has_parameter("delta_tau_max"))
        node->declare_parameter<double>("delta_tau_max", delta_tau_max_);
    if (!node->has_parameter("nullspace_stiffness"))
        node->declare_parameter<double>("nullspace_stiffness", nullspace_stiffness_);
    if (!node->has_parameter("max_force"))
        node->declare_parameter<double>("max_force", max_force_);
    if (!node->has_parameter("max_torque"))
        node->declare_parameter<double>("max_torque", max_torque_);
    if (!node->has_parameter("tau_limits"))
        node->declare_parameter<std::vector<double>>(
            "tau_limits", std::vector<double>(tau_limits_.begin(), tau_limits_.end()));
    if (!node->has_parameter("wrench_saturation"))
        node->declare_parameter<bool>("wrench_saturation", wrench_saturation_);

    node->get_parameter("arm_id",              arm_id_);
    node->get_parameter("arm_prefix",          arm_prefix_);
    node->get_parameter("screw_stiffness",     screw_stiffness_);
    node->get_parameter("rot_damping_ratio",   rot_damping_ratio_);
    node->get_parameter("alpha",               alpha_);
    node->get_parameter("delta_tau_max",       delta_tau_max_);
    node->get_parameter("nullspace_stiffness", nullspace_stiffness_);
    node->get_parameter("max_force",           max_force_);
    node->get_parameter("max_torque",          max_torque_);
    node->get_parameter("wrench_saturation",   wrench_saturation_);

    rot_damping_ratio_ = std::max(rot_damping_ratio_, 0.0);
    if (rot_damping_ratio_ > 0.5) {
        RCLCPP_WARN(node->get_logger(),
            "rot_damping_ratio=%.3f is high. Rotational apparent inertia is "
            "roughly 300x smaller than translational, so the discrete-time "
            "limit D*dt/Lambda < 2 binds on rotation first. Values approaching "
            "1.0 over-damp the wrist and can trip a Cartesian reflex.",
            rot_damping_ratio_);
    }

    if (!wrench_saturation_) {
        RCLCPP_WARN(node->get_logger(),
            "wrench_saturation is disabled: the Cartesian clamps (max_force=%.1f N, "
            "max_torque=%.1f Nm) will NOT be applied. The per-joint tau_limits "
            "clamp remains active.", max_force_, max_torque_);
    }

    if (nullspace_stiffness_ < 0.0) {
        RCLCPP_WARN(node->get_logger(),
                    "Parameter 'nullspace_stiffness' must be >= 0. Clamping to 0.");
        nullspace_stiffness_ = 0.0;
    }
    if (screw_stiffness_ < 0.0) {
        RCLCPP_WARN(node->get_logger(),
                    "Parameter 'screw_stiffness' must be >= 0. Clamping to 0.");
        screw_stiffness_ = 0.0;
    }

    std::vector<double> tau_limits_vec;
    node->get_parameter("tau_limits", tau_limits_vec);
    if (tau_limits_vec.size() == static_cast<size_t>(num_joints_)) {
        for (int i = 0; i < num_joints_; ++i)
            tau_limits_[i] = std::abs(tau_limits_vec[i]);
    } else {
        RCLCPP_WARN(node->get_logger(),
                    "Parameter 'tau_limits' must contain %d values. Using defaults.",
                    num_joints_);
    }

    // The filtered values start at their targets so activation is never a step.
    screw_stiffness_target_      = screw_stiffness_;
    nullspace_stiffness_target_  = nullspace_stiffness_;

    on_set_parameters_callback_handle_ = node->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) {
            return onParameterUpdate(parameters);
        });

    RCLCPP_INFO(node->get_logger(),
                "on_init: arm_id='%s'  arm_prefix='%s'  joint_prefix='%s'",
                arm_id_.c_str(), arm_prefix_.c_str(), buildJointPrefix().c_str());

    return CallbackReturn::SUCCESS;
}

// ==========================================================
// Runtime parameter updates.
//
// The gains feed filtered TARGETS rather than the live values, so a
// `ros2 param set` ramps in over many cycles instead of landing in a single
// 1 ms cycle.
// ==========================================================
rcl_interfaces::msg::SetParametersResult
MyCartesianImpedanceControllerCmpDQ::onParameterUpdate(
    const std::vector<rclcpp::Parameter>& parameters)
{
    for (const auto& param : parameters) {
        const std::string& name = param.get_name();
        if (name == "screw_stiffness") {
            screw_stiffness_target_ = std::max(param.as_double(), 0.0);
        } else if (name == "rot_damping_ratio") {
            rot_damping_ratio_ = std::max(param.as_double(), 0.0);
        } else if (name == "nullspace_stiffness") {
            nullspace_stiffness_target_ = std::max(param.as_double(), 0.0);
        } else if (name == "alpha") {
            alpha_ = std::clamp(param.as_double(), 1e-4, 1.0);
        } else if (name == "max_force") {
            max_force_ = std::max(param.as_double(), 0.0);
        } else if (name == "max_torque") {
            max_torque_ = std::max(param.as_double(), 0.0);
        }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

// ==========================================================
// on_configure
// ==========================================================
CallbackReturn MyCartesianImpedanceControllerCmpDQ::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    auto node = get_node();
    RCLCPP_INFO(node->get_logger(), "Loading URDF model...");

    try {
        std::string share_dir = ament_index_cpp::get_package_share_directory(
            "my_cartesian_impedance_controller_cmp_dq");
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
    q_        = Eigen::VectorXd::Zero(model_.nq);
    v_        = Eigen::VectorXd::Zero(model_.nv);
    tau_J_    = Eigen::VectorXd::Zero(model_.nv);
    tau_null_ = Eigen::VectorXd::Zero(model_.nv);
    tau_des_  = Eigen::VectorXd::Zero(model_.nv);
    coriolis_ = Eigen::VectorXd::Zero(model_.nv);
    d_diag_   = Eigen::VectorXd::Zero(6);
    tau_cmd_  = Eigen::VectorXd::Zero(model_.nv);
    tau_prev_ = Eigen::VectorXd::Zero(model_.nv);
    J_        = Eigen::MatrixXd::Zero(6, model_.nv);
    J_transpose_pinv_    = Eigen::MatrixXd::Zero(6, model_.nv);
    nullspace_projector_ = Eigen::MatrixXd::Identity(model_.nv, model_.nv);
    identity7_           = Eigen::MatrixXd::Identity(model_.nv, model_.nv);
    q_d_nullspace_       = Eigen::VectorXd::Zero(model_.nq);

    error_screw_ = Eigen::VectorXd::Zero(6);
    v_curr_cart_ = Eigen::VectorXd::Zero(6);
    F_des_       = Eigen::VectorXd::Zero(6);
    F_ff_        = Eigen::Matrix<double, 6, 1>::Zero();

    // --- Desired pose subscriber ---
    sub_target_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/target_pose", rclcpp::SystemDefaultsQoS(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            input_target_.writeFromNonRT(msg);
        });

    // --- Feedforward wrench subscriber ---
    sub_force_ff_ = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "~/feedforward_wrench", rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(ff_mutex_);
            F_ff_ << msg->wrench.force.x,  msg->wrench.force.y,  msg->wrench.force.z,
                     msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z;
        });

    // --- Diagnostic publishers (see README.md) ---
    pub_error_ = node->create_publisher<geometry_msgs::msg::WrenchStamped>(
        "~/cartesian_error", rclcpp::SystemDefaultsQoS());
    pub_wrench_ = node->create_publisher<geometry_msgs::msg::WrenchStamped>(
        "~/wrench_cmd", rclcpp::SystemDefaultsQoS());
    pub_tau_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "~/tau_cmd", rclcpp::SystemDefaultsQoS());
    pub_dq_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "~/dq_state", rclcpp::SystemDefaultsQoS());

    return CallbackReturn::SUCCESS;
}

// ==========================================================
// on_activate
// ==========================================================
CallbackReturn MyCartesianImpedanceControllerCmpDQ::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    for (int i = 0; i < num_joints_; ++i) {
        q_[i] = state_interfaces_[2 * i].get_value();
        v_[i] = state_interfaces_[2 * i + 1].get_value();
    }

    // The equilibrium pose starts at wherever the arm already is, so activation
    // never commands a step.
    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);
    M_des_     = data_.oMf[ee_frame_id_];
    M_des_raw_ = M_des_;
    q_des_filtered_ = Eigen::Quaterniond(M_des_.rotation());
    q_des_filtered_.normalize();

    input_target_.reset();
    is_first_update_ = true;
    // Let the damping snap to its correct value on the first cycle rather than
    // ramping up from zero, which would leave the arm undamped at activation.
    d_diag_initialised_ = false;
    tau_prev_.setZero();
    q_d_nullspace_ = q_;
    F_ff_.setZero();

    RCLCPP_INFO(get_node()->get_logger(),
                "DQ impedance controller activated (joint prefix: '%s').",
                buildJointPrefix().c_str());
    return CallbackReturn::SUCCESS;
}

// ==========================================================
// on_deactivate
// ==========================================================
CallbackReturn MyCartesianImpedanceControllerCmpDQ::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    RCLCPP_INFO(get_node()->get_logger(), "DQ impedance controller deactivated.");
    return CallbackReturn::SUCCESS;
}

// ==========================================================
// update  (1000 Hz real-time loop)
// ==========================================================
controller_interface::return_type MyCartesianImpedanceControllerCmpDQ::update(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // --- 1. Read joint state & kinematics ---
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

    // --- 2. First cycle: anchor the setpoint to the current pose ---
    if (is_first_update_) {
        tau_prev_.setZero();
        M_des_     = M_curr_;
        M_des_raw_ = M_curr_;
        q_des_filtered_ = Eigen::Quaterniond(M_curr_.rotation());
        q_des_filtered_.normalize();
        is_first_update_ = false;
    }

    // --- 3. ROS setpoint ---
    auto current_target_msg = input_target_.readFromRT();
    if (current_target_msg && *current_target_msg) {
        auto& msg = *current_target_msg;
        Eigen::Quaterniond quat(msg->pose.orientation.w, msg->pose.orientation.x,
                                msg->pose.orientation.y, msg->pose.orientation.z);
        Eigen::Vector3d trans(msg->pose.position.x,
                              msg->pose.position.y,
                              msg->pose.position.z);
        M_des_raw_ = pinocchio::SE3(quat.normalized(), trans);
    }

    // Low-pass filter on the desired position.
    M_des_.translation() =
        (1.0 - alpha_) * M_des_.translation() + alpha_ * M_des_raw_.translation();

    // Low-pass filter on the desired orientation. The state lives in
    // q_des_filtered_ and is never read back out of M_des_.rotation() -- see
    // the header for why that round trip is numerically unstable. Eigen's slerp
    // does not renormalise, so we do.
    Eigen::Quaterniond q_target_raw(M_des_raw_.rotation());
    q_target_raw.normalize();
    q_des_filtered_ = q_des_filtered_.slerp(alpha_, q_target_raw);
    q_des_filtered_.normalize();
    M_des_.rotation() = q_des_filtered_.toRotationMatrix();

    // --- 4. Dual-quaternion screw error ---
    // Build unit dual quaternions for the current and desired poses.
    Eigen::Quaterniond q_curr(M_curr_.rotation());
    Eigen::Vector3d    p_curr = M_curr_.translation();
    orient_curr_ << q_curr.w(), q_curr.x(), q_curr.y(), q_curr.z();
    trans_curr_  << 0.0, p_curr.x(), p_curr.y(), p_curr.z();
    dq_curr_ = DQ::rotationTranslationToDQ(orient_curr_, trans_curr_);

    // Orientation comes from the quaternion state directly, not from
    // M_des_.rotation() -- same reason as the filter above.
    const Eigen::Quaterniond& q_des = q_des_filtered_;
    Eigen::Vector3d    p_des = M_des_.translation();
    orient_des_ << q_des.w(), q_des.x(), q_des.y(), q_des.z();
    trans_des_  << 0.0, p_des.x(), p_des.y(), p_des.z();
    dq_des_ = DQ::rotationTranslationToDQ(orient_des_, trans_des_);

    // Screw displacement error: err = des * conj(curr), a world-frame (left)
    // composition, so its screw axis and moment are expressed in the base frame.
    dq_error_ = DQ::multiplyDQ(dq_des_, DQ::classicConjugateDQ(dq_curr_));
    DQ::dqToScrewParameters(dq_error_, theta_e_, d_e_, l_e_, m_e_);

    // Snapshot for the ~/dq_state diagnostic, before the shortest-path flip can
    // mutate dq_des_ or theta_e_.
    const Eigen::Matrix<double, 8, 1> dq_des_pre = dq_des_;
    const double theta_e_pre = theta_e_;

    // Flip the desired DQ if the rotation exceeds pi (shortest path).
    if (theta_e_ > M_PI) {
        dq_des_   = -dq_des_;
        dq_error_ = DQ::multiplyDQ(dq_des_, DQ::classicConjugateDQ(dq_curr_));
        DQ::dqToScrewParameters(dq_error_, theta_e_, d_e_, l_e_, m_e_);
    }

    // Screw twist components: angular w = l*theta, linear v = l*d + m*theta.
    // des*conj(curr) is the transform curr->des, so these already point toward
    // the target; no negation is needed.
    const Eigen::Vector3d w_e = (l_e_ * theta_e_).transpose();
    const Eigen::Vector3d v_e = (l_e_ * d_e_ + m_e_ * theta_e_).transpose();

    // Note on frames: v_e/w_e form the SPATIAL screw twist -- m_e_ is the moment
    // of the screw axis about the base-frame origin, so v_e is the velocity of
    // the point at the origin rather than of the end-effector. J_, v_curr_cart_
    // and F_ff_ are all LOCAL_WORLD_ALIGNED (reference point at the EE,
    // world-aligned axes). The reference points therefore differ by the lever
    // arm from the base origin. It is left as-is here because that is what was
    // validated on hardware for this task; if you see reach-dependent behaviour
    // on your robot, this is the first place to look.
    error_screw_ << v_e, w_e;

    // --- 5. Impedance law (screw PD + feedforward wrench) ---
    // One scalar stiffness restores all six screw components.
    screw_stiffness_ =
        alpha_ * screw_stiffness_target_ + (1.0 - alpha_) * screw_stiffness_;

    // Damping derived from the stiffness, so it tracks it:
    //     D_trans = 2*sqrt(K)
    //     D_rot   = rot_damping_ratio * D_trans
    const double trans_damping = 2.0 * std::sqrt(screw_stiffness_);
    const double rot_damping   = rot_damping_ratio_ * trans_damping;
    Eigen::VectorXd d_target(6);
    d_target << trans_damping, trans_damping, trans_damping,
                rot_damping,   rot_damping,   rot_damping;

    // Ramp toward the target instead of stepping, so a runtime parameter change
    // does not land in a single 1 ms cycle.
    if (!d_diag_initialised_) {
        d_diag_ = d_target;
        d_diag_initialised_ = true;
    } else {
        d_diag_ = alpha_ * d_target + (1.0 - alpha_) * d_diag_;
    }

    {
        std::lock_guard<std::mutex> lock(ff_mutex_);
        F_des_ = screw_stiffness_ * error_screw_
               - d_diag_.cwiseProduct(v_curr_cart_) + F_ff_;
    }

    // Snapshot the wrench BEFORE any saturation, for the diagnostic publisher.
    const Eigen::Matrix<double, 6, 1> F_raw = F_des_;

    // Cartesian wrench saturation, each block scaled so the wrench keeps its
    // direction. See the header for when to disable it.
    if (wrench_saturation_) {
        const double f_norm = F_des_.head<3>().norm();
        if (f_norm > max_force_)
            F_des_.head<3>() *= max_force_ / f_norm;
        const double t_norm = F_des_.tail<3>().norm();
        if (t_norm > max_torque_)
            F_des_.tail<3>() *= max_torque_ / t_norm;
    }

    tau_J_.noalias() = J_.transpose() * F_des_;

    // --- 6. Nullspace control ---
    constexpr double pseudo_inverse_damping = 1e-6;
    Eigen::Matrix<double, 6, 6> jj_t_damped =
        J_ * J_.transpose() +
        pseudo_inverse_damping * Eigen::Matrix<double, 6, 6>::Identity();
    J_transpose_pinv_.noalias() = jj_t_damped.ldlt().solve(J_);

    nullspace_projector_.noalias() = identity7_ - J_.transpose() * J_transpose_pinv_;

    nullspace_stiffness_ =
        alpha_ * nullspace_stiffness_target_ + (1.0 - alpha_) * nullspace_stiffness_;
    const double nullspace_damping = 2.0 * std::sqrt(std::max(nullspace_stiffness_, 0.0));
    tau_null_.noalias() = nullspace_projector_ *
                          (nullspace_stiffness_ * (q_d_nullspace_ - q_)
                           - nullspace_damping * v_);

    tau_des_ = tau_J_ + tau_null_ + coriolis_;

    // --- 7. Slew rate limiter ---
    tau_cmd_ = tau_des_;
    for (int i = 0; i < num_joints_; ++i) {
        double diff = tau_cmd_[i] - tau_prev_[i];
        diff = std::max(std::min(diff, delta_tau_max_), -delta_tau_max_);
        tau_cmd_[i]  = tau_prev_[i] + diff;
        tau_prev_[i] = tau_cmd_[i];
    }

    // --- 8. Per-joint torque clamp (always applied) ---
    std::array<double, 7> tau_out{};
    for (int i = 0; i < num_joints_; ++i) {
        tau_out[i] = std::clamp(tau_cmd_[i], -tau_limits_[i], tau_limits_[i]);
        command_interfaces_[i].set_value(tau_out[i]);
    }

    // --- 9. Diagnostics ---
    geometry_msgs::msg::WrenchStamped err_msg;
    err_msg.header.stamp    = get_node()->now();
    err_msg.header.frame_id = buildJointPrefix() + "_link0";
    err_msg.wrench.force.x  = error_screw_[0];
    err_msg.wrench.force.y  = error_screw_[1];
    err_msg.wrench.force.z  = error_screw_[2];
    err_msg.wrench.torque.x = error_screw_[3];
    err_msg.wrench.torque.y = error_screw_[4];
    err_msg.wrench.torque.z = error_screw_[5];
    pub_error_->publish(err_msg);

    // Commanded wrench BEFORE the Cartesian clamp. Comparing its norms against
    // max_force_ / max_torque_ shows whether that saturation is binding.
    geometry_msgs::msg::WrenchStamped w_msg;
    w_msg.header          = err_msg.header;
    w_msg.wrench.force.x  = F_raw[0];
    w_msg.wrench.force.y  = F_raw[1];
    w_msg.wrench.force.z  = F_raw[2];
    w_msg.wrench.torque.x = F_raw[3];
    w_msg.wrench.torque.y = F_raw[4];
    w_msg.wrench.torque.z = F_raw[5];
    pub_wrench_->publish(w_msg);

    // [0..6]   tau after the slew limiter, before the per-joint clamp
    // [7..13]  tau actually written to the hardware
    // [14..19] the live damping diagonal
    std_msgs::msg::Float64MultiArray tau_msg;
    tau_msg.data.resize(20);
    for (int i = 0; i < num_joints_; ++i) {
        tau_msg.data[i]     = tau_cmd_[i];
        tau_msg.data[i + 7] = tau_out[i];
    }
    for (int i = 0; i < 6; ++i)
        tau_msg.data[14 + i] = d_diag_[i];
    pub_tau_->publish(tau_msg);

    // Full DQ error state; error_screw_ is a pure function of these.
    std_msgs::msg::Float64MultiArray dq_msg;
    dq_msg.data.resize(34);
    for (int i = 0; i < 8; ++i) {
        dq_msg.data[i]      = dq_curr_[i];
        dq_msg.data[8 + i]  = dq_des_pre[i];
        dq_msg.data[16 + i] = dq_error_[i];
    }
    dq_msg.data[24] = theta_e_pre;
    dq_msg.data[25] = theta_e_;
    dq_msg.data[26] = (theta_e_pre > M_PI) ? 1.0 : 0.0;
    dq_msg.data[27] = d_e_;
    for (int i = 0; i < 3; ++i) {
        dq_msg.data[28 + i] = l_e_[i];
        dq_msg.data[31 + i] = m_e_[i];
    }
    pub_dq_->publish(dq_msg);

    return controller_interface::return_type::OK;
}

}  // namespace my_cartesian_impedance_controller_cmp_dq

PLUGINLIB_EXPORT_CLASS(
    my_cartesian_impedance_controller_cmp_dq::MyCartesianImpedanceControllerCmpDQ,
    controller_interface::ControllerInterface)
