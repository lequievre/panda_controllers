#ifndef JOINT_POSITON_CONTROLLER_IP_H_
#define JOINT_POSITON_CONTROLLER_IP_H_

#include <array>
#include <string>
#include <vector>

#include <dynamic_reconfigure/server.h>
#include <franka_panda_ip_controllers/joint_position_controller_ip_paramsConfig.h>

#include <franka_core_msgs/JointCommand.h>
#include <franka_core_msgs/JointControllerStates.h>
#include <franka_core_msgs/JointLimits.h>

#include <mutex>
#include <franka_hw/trigger_rate.h>
#include <realtime_tools/realtime_publisher.h>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

// Boost
#include <boost/scoped_ptr.hpp>

// Eigen
#include <eigen3/Eigen/Dense>

// Float 64 MultiArray message
#include <std_msgs/Float64MultiArray.h>

// realtime tools
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <mutex>

// motion generator
#include <robot_motion_generation/CDDynamics.h>

namespace franka_panda_ip_controllers {

// NB joints of franka panda = 7
constexpr int NB_JOINTS = 7;

using MotionVector = Eigen::VectorXd;

class JointPositionControllerIP : public controller_interface::MultiInterfaceController<
                                          	franka_hw::FrankaModelInterface,
                                        	hardware_interface::PositionJointInterface,
                                        	franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;			// Start the controller
  void stopping(const ros::Time& time) override;	// Stop the controller
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  hardware_interface::PositionJointInterface* position_joint_interface_;
  std::vector<hardware_interface::JointHandle> position_joint_handles_;
  
  boost::scoped_ptr<motion::CDDynamics>   joint_cddynamics;
  motion::Vector motion_velocity_limits_;
  
  mutable std::mutex mutex_;
  
  // joint_cmd subscriber
  ros::Subscriber sub_command_;

  /*
  double filter_joint_pos_{0.3};
  double target_filter_joint_pos_{0.1};
  double filter_factor_{0.01};

  double param_change_filter_{0.005};
  */

  franka_core_msgs::JointLimits joint_limits_;
  
  /*
  // Dynamic reconfigure
  std::unique_ptr< dynamic_reconfigure::Server<franka_panda_ip_controllers::joint_position_controller_ip_paramsConfig> > dynamic_server_joint_controller_params_;
  ros::NodeHandle dynamic_reconfigure_joint_controller_params_node_;
  
  void jointPositionControllerIPParamCallback(franka_panda_ip_controllers::joint_position_controller_ip_paramsConfig& config,
                               uint32_t level);

  franka_hw::TriggerRate trigger_publish_;
  realtime_tools::RealtimePublisher<franka_core_msgs::JointControllerStates> publisher_controller_states_;
  */
  
  bool checkPositionLimits(const std::vector<double> & positions);
                                           
  void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg);
  
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
            
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> > realtime_obs_orientation_pub_;
  std_msgs::Float64MultiArray obs_orientation_msg_;
  
  realtime_tools::RealtimeBuffer<std::vector<double> >target_buffer_;
  
  MotionVector motion_target_;
  MotionVector motion_cmd_;
  MotionVector motion_current_position_;
  int cmd_flag_;  // flag set only to 1 when the controller receive a message to the command topic
            
};

}  // namespace franka_panda_ip_controllers

#endif
