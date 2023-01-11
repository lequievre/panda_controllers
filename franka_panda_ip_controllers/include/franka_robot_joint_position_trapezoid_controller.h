#ifndef FRANKA_ROBOT_JOINT_POSITON_TRAPEZOID_CONTROLLER_H_
#define FRANKA_ROBOT_JOINT_POSITON_TRAPEZOID_CONTROLLER_H_

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Dense>

// realtime tools
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// Float 64 MultiArray message
#include <std_msgs/Float64MultiArray.h>

#include <math.h>

namespace franka_panda_ip_controllers {

	// degree to radian, radian to degree converters
    constexpr double RAD_TO_DEG = 180.0/M_PI;
    constexpr double DEG_TO_RAD = M_PI/180.0;
    
    // NB joints of franka panda = 7
    constexpr int NB_JOINTS = 7;

    class JointPositionTrapezoidController: public controller_interface::MultiInterfaceController<
                                        franka_hw::FrankaModelInterface,
                                        hardware_interface::PositionJointInterface,
                                        franka_hw::FrankaStateInterface>
    {
        
        public:
            bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
            void starting(const ros::Time&) override;
            void update(const ros::Time&, const ros::Duration& period) override;  

            void commandCB(const sensor_msgs::JointStateConstPtr& msg); // callback function for the topic command
            
		private:
            std::vector<hardware_interface::JointHandle> position_joint_handles_;
            
            ros::Duration elapsed_time_;
           
            std::vector<double> current_joint_velocities_commands_;
            std::vector<double> joint_accelerations_;

            std::vector<double> joint_commands_;

            ros::Subscriber sub_command_;
            
            std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
            std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
            
            std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> > realtime_obs_orientation_pub_;
            std_msgs::Float64MultiArray obs_orientation_msg_;
            
            realtime_tools::RealtimeBuffer<std::vector<double> > desired_joint_positions_buffer_; // realtime buffer for the positions desired
            realtime_tools::RealtimeBuffer<std::vector<double> > desired_joint_velocities_buffer_; // realtime buffer for the velocities desired

    };
}

#endif


