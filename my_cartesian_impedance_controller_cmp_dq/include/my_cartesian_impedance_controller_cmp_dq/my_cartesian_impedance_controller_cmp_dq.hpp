#pragma once

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <array>
#include <memory>
#include <mutex>
#include <string>
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

#include <dq_operations/dq.h>

using CallbackReturn = controller_interface::CallbackReturn;

namespace my_cartesian_impedance_controller_cmp_dq {

// Cartesian impedance controller whose pose error is a dual-quaternion screw
// displacement, with an additional feedforward wrench input.
//
//     F_des = screw_stiffness * error_screw - damping * velocity + F_ff
//     tau   = J^T * F_des + tau_nullspace + coriolis
//
// Translation and rotation are driven as one coupled screw motion rather than
// as two independent error terms. Kinematics and dynamics come from Pinocchio
// and the URDF, so nothing here is Franka-specific except the bundled URDF and
// the default joint-torque limits.
//
// See README.md for usage and for what to tune.
class MyCartesianImpedanceControllerCmpDQ : public controller_interface::ControllerInterface {
public:
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::return_type update(const rclcpp::Time& time,
                                             const rclcpp::Duration& period) override;

private:
    // --- Multi-robot identification ---
    std::string arm_id_     = "fr3";  // e.g. "fr3"
    std::string arm_prefix_ = "";     // e.g. "robot1" -> joint prefix = "robot1_fr3"
    std::string buildJointPrefix() const;

    // --- Pinocchio & Robot ---
    pinocchio::Model model_;
    pinocchio::Data  data_;
    int num_joints_  = 7;
    int ee_frame_id_ = -1;

    // --- State variables (pre-allocated to avoid malloc in the RT loop) ---
    Eigen::VectorXd q_;
    Eigen::VectorXd v_;
    Eigen::VectorXd tau_J_;       // Task-space torque
    Eigen::VectorXd tau_null_;    // Nullspace torque
    Eigen::VectorXd tau_des_;     // Desired torque before safety checks
    Eigen::VectorXd coriolis_;    // Coriolis term
    Eigen::VectorXd tau_cmd_;     // Final sent torque
    Eigen::MatrixXd J_;           // Jacobian (LOCAL_WORLD_ALIGNED)
    Eigen::MatrixXd J_transpose_pinv_;
    Eigen::MatrixXd nullspace_projector_;
    Eigen::MatrixXd identity7_;

    // --- DQ screw error computation (pre-allocated) ---
    Eigen::RowVector4d          orient_curr_, trans_curr_;
    Eigen::RowVector4d          orient_des_,  trans_des_;
    Eigen::Matrix<double, 8, 1> dq_curr_, dq_des_, dq_error_;
    Eigen::RowVector3d          l_e_, m_e_;   // screw axis and moment
    double theta_e_ = 0.0;                    // screw angle
    double d_e_     = 0.0;                    // screw translation
    Eigen::VectorXd error_screw_;             // 6D [linear; angular], see update()
    Eigen::VectorXd v_curr_cart_;
    Eigen::VectorXd F_des_;                   // Desired Cartesian wrench

    // --- Feedforward wrench ---
    Eigen::Matrix<double, 6, 1> F_ff_;
    std::mutex ff_mutex_;

    // --- Target ---
    pinocchio::SE3 M_des_;      // Desired pose (low-pass filtered)
    pinocchio::SE3 M_des_raw_;  // Latest commanded pose, unfiltered

    // Filtered desired ORIENTATION. This quaternion is the authoritative filter
    // state: it is renormalised every cycle, and M_des_.rotation() is only ever
    // written FROM it, never read back into it.
    //
    // Do not be tempted to keep the state in M_des_.rotation() instead. That
    // makes each cycle a matrix -> quaternion -> slerp -> matrix round trip;
    // Eigen's slerp does not renormalise, so the norm error squares every
    // millisecond. It sits at machine epsilon for a long time and then diverges
    // abruptly.
    Eigen::Quaterniond q_des_filtered_ = Eigen::Quaterniond::Identity();
    pinocchio::SE3 M_curr_;     // Current pose

    // --- Nullspace ---
    // Pulls the arm toward the joint configuration recorded at activation,
    // without disturbing the Cartesian task. 0.0 disables it.
    double nullspace_stiffness_        = 20.0;
    double nullspace_stiffness_target_ = 20.0;
    Eigen::VectorXd q_d_nullspace_;

    bool is_first_update_ = true;
    Eigen::VectorXd tau_prev_;    // For the slew rate limiter

    // --- Parameters (configurable via YAML, see README.md) ---
    //
    // ONE scalar stiffness drives the restoring term on all six screw-error
    // components. Damping is derived from it:
    //
    //     D_trans = 2*sqrt(screw_stiffness)
    //     D_rot   = rot_damping_ratio * D_trans
    //
    // Because damping follows the stiffness, the damping ratio
    // zeta = D / (2*sqrt(K*Lambda)) is independent of K, so retuning the
    // stiffness preserves it automatically. This is the main parameter to tune.
    double screw_stiffness_        = 250.0;
    double screw_stiffness_target_ = 250.0;

    // Rotational damping as a fraction of translational: literally
    // D_rot / D_trans.
    //
    // Rotation needs its own knob because it is the axis that goes unstable
    // first. On an FR3 the apparent inertia at the end-effector is 4.3-9.9 kg
    // translationally but only ~0.015 kg m^2 rotationally, ~300x smaller, so
    // the discrete-time stability limit D*dt/Lambda < 2 binds on rotation long
    // before it binds on translation. Raising this toward 1.0 (rotational
    // damping equal to translational) over-damps the wrist and trips a
    // Cartesian reflex; the controller warns above 0.5.
    double rot_damping_ratio_      = 0.2;     // [-] D_rot / D_trans

    // Low-pass filter coefficient, applied per cycle to the desired pose, the
    // stiffness and the damping. Smaller is smoother but lags more.
    double alpha_                  = 0.005;

    // Damping is filtered with alpha_ too, so a runtime `ros2 param set` ramps
    // in over many cycles instead of stepping in a single 1 ms cycle.
    Eigen::VectorXd d_diag_;                  // 6, current filtered damping
    bool d_diag_initialised_ = false;

    double delta_tau_max_          = 1.0;     // [Nm per cycle] torque slew limit

    // Cartesian wrench saturation. Each block is scaled so the wrench keeps its
    // direction. Guards against a transient at high stiffness exceeding the
    // robot's collision thresholds and aborting the motion with a reflex.
    //
    // Disable it (wrench_saturation: false) when the feedforward wrench itself
    // is legitimately large: the clamp scales the WHOLE block, so a feedforward
    // torque near max_torque drags the restoring moment down with it and
    // distorts the commanded direction. The per-joint tau_limits clamp below
    // stays active either way.
    bool   wrench_saturation_      = true;
    double max_force_              = 20.0;    // [N]
    double max_torque_             = 10.0;    // [Nm]

    // Per-joint torque clamp, always applied. The slew limiter bounds only how
    // fast the command changes, not how large it gets: at 1 Nm/ms it reaches
    // 87 Nm in 87 ms. Defaults are the FR3's hardware limits; change them for
    // another arm.
    std::array<double, 7> tau_limits_ = {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        on_set_parameters_callback_handle_;
    rcl_interfaces::msg::SetParametersResult onParameterUpdate(
        const std::vector<rclcpp::Parameter>& parameters);

    // --- ROS 2 interfaces ---
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr   sub_target_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_force_ff_;

    // --- Diagnostics (see README.md for the layouts) ---
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr    pub_error_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr    pub_wrench_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr     pub_tau_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr     pub_dq_;

    realtime_tools::RealtimeBuffer<
        std::shared_ptr<geometry_msgs::msg::PoseStamped>> input_target_;
};

}  // namespace my_cartesian_impedance_controller_cmp_dq
