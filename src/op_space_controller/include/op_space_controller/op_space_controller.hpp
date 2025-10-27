#ifndef OP_SPACE_CONTROLLER__OP_SPACE_CONTROLLER_HPP_
#define OP_SPACE_CONTROLLER__OP_SPACE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
// #include "hardware_interface/loaned_command_interface.hpp"
// #include "hardware_interface/loaned_state_interface.hpp"
// #include "hardware_interface/loanable_command_interface.hpp" // Per LoanableCommandInterface
// #include "hardware_interface/loanable_state_interface.hpp"   // Per LoanableStateInterface

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "rclcpp/time.hpp"  //è sufficiente includere solo rclcpp/rclcpp.hpp

//#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "pinocchio/fwd.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include <pinocchio/spatial/se3.hpp>        // Per pinocchio::SE3
#include <pinocchio/spatial/motion.hpp>     // Per pinocchio::Motion
// #include "pinocchio/parsers/urdf.hpp"  //meglio nel cpp
// #include "pinocchio/algorithm/joint-configuration.hpp"   //meglio nel cpp

#include <Eigen/Dense> // Per Eigen::VectorXd

// visibility macros
#include "op_space_controller/visibility_control.h"

namespace op_space_controller
{

struct EEWaypoint
{
  pinocchio::SE3 pose_desired;       // Posa desiderata dell'end-effector
  pinocchio::Motion velocity_desired; // Velocità desiderata
  pinocchio::Motion acceleration_desired; // Accelerazione desiderata
  rclcpp::Duration time_from_start;

  EEWaypoint():
    pose_desired(pinocchio::SE3::Identity()),
    velocity_desired(pinocchio::Motion::Zero()),
    acceleration_desired(pinocchio::Motion::Zero()),
    time_from_start(0, 0) // Chiama un costruttore pubblico di rclcpp::Duration
  {}
};

class OP_SPACE_CONTROLLER_PUBLIC OpSpaceController : public controller_interface::ControllerInterface
{
public:
  OpSpaceController();

  ~OpSpaceController() override = default; // Distruttore - Buona pratica

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  // loaned interfaces  -- VECCHIO
  // std::vector<hardware_interface::LoanedStateInterface> joint_state_handles_;
  // std::vector<hardware_interface::LoanedCommandInterface> joint_command_handles_;

  // NUOVO 2.0 - 1.0: avevo LoanableCommandInterface e LoanableStateInterface, ma ora uso Loaned
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> state_if_position_handles_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> state_if_velocity_handles_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> cmd_if_effort_handles_;

  // parameters
  std::vector<std::string> joint_names_;
  std::string command_interface_type_;
  std::string urdf_file_path_;
  std::string base_link_name_;
  std::string ee_link_name_;
  std::string map_frame_name_;
  std::string trajectory_file_path_;

  // Guadagni del controllore (esempi)
  double trajectory_sample_dt_;
  double kp_linear_, kd_linear_, kp_angular_, kd_angular_;

  size_t num_joints_;

  // Pinocchio model and data
  pinocchio::Model model_;                  // Modello caricato da urdf_file_path_
  std::shared_ptr<pinocchio::Data> data_;   // Inizializzato con model_
  pinocchio::FrameIndex ee_frame_id_pin_;   // ID Pinocchio del link EE (da ee_link_name_)
  bool pinocchio_model_loaded_ = false;     // Flag di stato

  // TF2 for SLAM pose
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // trajectory waypoints
  std::vector<EEWaypoint> desired_trajectory_;
  bool trajectory_active_ = false;
  rclcpp::Time trajectory_start_time_; // Registra quando l'inseguimento della traiettoria inizia
  size_t current_trajectory_index_ = 0; // Indice al waypoint corrente nella desired_trajectory_

  // Stato interno del robot per i calcoli (vettori Eigen)
  Eigen::VectorXd q_;   // Posizioni joint attuali (dimensione model_.nq)
  Eigen::VectorXd dq_;  // Velocità joint attuali (dimensione model_.nv)
  Eigen::VectorXd tau_; // Coppie/sforzi calcolati da comandare (dimensione model_.nv)

  // helpers
  bool read_trajectory_from_file(const std::string & file_path);
  bool wait_for_transform(const std::string & target_frame, const std::string & source_frame, const rclcpp::Duration & timeout);
  // Potresti aggiungere altre funzioni helper qui, es:
  // Eigen::Isometry3d get_current_ee_pose_tf();
  // Eigen::Matrix<double, 6, Eigen::Dynamic> calculate_current_jacobian();
  // void calculate_control_law(...);


};

}  // namespace op_space_controller

#endif  // OP_SPACE_CONTROLLER__OP_SPACE_CONTROLLER_HPP_

