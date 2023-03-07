/***************************************************************************

*
* @package: franka_ros_controllers
* @metapackage: franka_ros_interface
* @author: Saif Sidhik <sxs1412@bham.ac.uk>
*

**************************************************************************/

/***************************************************************************
* Copyright (c) 2019-2021, Saif Sidhik.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
**************************************************************************/
#include <franka_panda_ip_controllers/position_joint_position_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <Eigen/Dense>


namespace franka_panda_ip_controllers {


void PositionJointPositionController::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
{
		ROS_INFO("***** START PositionJointPositionController::commandCB ************");
		
		if (msg->data.size() != position_joint_handles_.size()) {
			ROS_ERROR_STREAM("Dimension of position command (" << msg->data.size() << ") does not match number of joints (" << position_joint_handles_.size() << ")! Not executing!");
			
			pos_d_buffer_.writeFromNonRT(*prev_pos_buffer_.readFromNonRT());
        		pos_d_target_buffer_.writeFromNonRT(*prev_pos_buffer_.readFromNonRT());
		}
		else if (checkPositionLimits(msg->data)) {
         		ROS_ERROR_STREAM("PositionJointPositionController: Commanded positions are beyond allowed position limits.");
         		pos_d_buffer_.writeFromNonRT(*prev_pos_buffer_.readFromNonRT());
        		pos_d_target_buffer_.writeFromNonRT(*prev_pos_buffer_.readFromNonRT());
         		}
         		
		else {
		 	pos_d_target_buffer_.writeFromNonRT(msg->data);
	 	}
	 	
	 	for (size_t i=0; i<7; ++i)
		{
			ROS_INFO_STREAM("commandCB : " << msg->data[i] << ", " );
		}
		   
		ROS_INFO("***** FINISH PositionJointPositionController::commandCB ************");
	}


bool PositionJointPositionController::init(hardware_interface::RobotHW* robot_hw,
                                          ros::NodeHandle& node_handle) {

	// subscribe to JointState command
	sub_command_ = node_handle.subscribe("command", 20, &PositionJointPositionController::commandCB, this, ros::TransportHints().reliable().tcpNoDelay());
		
	// Init RT buffers with initial 0.0 values
	initial_pos_buffer_.readFromNonRT()->resize(NB_JOINTS,0.0);
	prev_pos_buffer_.readFromNonRT()->resize(NB_JOINTS,0.0);
	pos_d_target_buffer_.readFromNonRT()->resize(NB_JOINTS,0.0);
	pos_d_buffer_.readFromNonRT()->resize(NB_JOINTS,0.0);


	std::vector<std::string> joint_names;
	if (!node_handle.getParam("joint_names",joint_names))
	{
		ROS_ERROR("Could not get joint_names into yaml config file.");
	}

	if (joint_names.size()!=NB_JOINTS)
	{
		ROS_ERROR_STREAM("Did not recieve " << NB_JOINTS << " joints, Franka robot is " << NB_JOINTS << " DOF robot!");
		return false;
	}
		
	std::string arm_id;
	if (!node_handle.getParam("arm_id", arm_id)) {
	    ROS_ERROR_STREAM("PositionJointPositionController: Could not read parameter arm_id");
	    return false;
	}

	auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
	if (model_interface == nullptr) {
	    ROS_ERROR_STREAM("PositionJointPositionController: Error getting model interface from hardware");
	    return false;
	}

	try 
	{
	    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
	}
	catch (hardware_interface::HardwareInterfaceException& ex) 
	{
		    ROS_ERROR_STREAM("PositionJointPositionController: Exception getting model handle from interface: " << ex.what());
		    return false;
	}

	auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
	if (state_interface == nullptr) {
	    ROS_ERROR_STREAM("PositionJointPositionController: Error getting state interface from hardware");
	    return false;
	}

	try 
	{
	    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
	}
	catch (hardware_interface::HardwareInterfaceException& ex) 
	{
	    ROS_ERROR_STREAM("PositionJointPositionController: Exception getting state handle from interface: " << ex.what());
	    return false;
	}

	position_joint_interface_ = robot_hw->get<hardware_interface::PositionJointInterface>();
	if (position_joint_interface_ == nullptr) {
	    ROS_ERROR_STREAM("PositionJointPositionController: Error getting position joint interface from hardware");
	    return false;
	}

	// publishers for current cartesian pose, velocity, orientation, orientation velocity
	realtime_obs_orientation_pub_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray>(node_handle, "current_observation_orientation", 4));

	obs_orientation_msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());  
	obs_orientation_msg_.layout.dim[0].size = 13;
	obs_orientation_msg_.layout.dim[0].stride = 1;
	obs_orientation_msg_.layout.dim[0].label = "observation_orientation";

	for (size_t i = 0; i < obs_orientation_msg_.layout.dim[0].size; ++i)
	    obs_orientation_msg_.data.push_back(0.0);


	if (!node_handle.getParam("joint_names", joint_limits_.joint_names)) {
		ROS_ERROR("PositionJointPositionController: Could not parse joint names");
	}
	
	if (joint_limits_.joint_names.size() != NB_JOINTS) {
		ROS_ERROR_STREAM("PositionJointPositionController: Wrong number of joint names, got "
		     << joint_limits_.joint_names.size() << " instead of " << NB_JOINTS << " names!");
	return false;
	}
	
	std::map<std::string, double> pos_limit_lower_map;
	std::map<std::string, double> pos_limit_upper_map;
	
	if (!node_handle.getParam("joint_config/joint_position_limit/lower", pos_limit_lower_map) ) {
		ROS_ERROR("PositionJointPositionController: Joint limits parameters not provided, aborting ");
		return false;
	}
	
	if (!node_handle.getParam("joint_config/joint_position_limit/upper", pos_limit_upper_map) ) {
		ROS_ERROR("PositionJointPositionController: Joint limits parameters not provided, aborting ");
		return false;
	}

	for (size_t i = 0; i < joint_limits_.joint_names.size(); ++i){
		if (pos_limit_lower_map.find(joint_limits_.joint_names[i]) != pos_limit_lower_map.end())
		{
			joint_limits_.position_lower.push_back(pos_limit_lower_map[joint_limits_.joint_names[i]]);
		}
		else
		{
			ROS_ERROR("PositionJointPositionController: Unable to find lower position limit values for joint %s...",
			       joint_limits_.joint_names[i].c_str());
		}
		if (pos_limit_upper_map.find(joint_limits_.joint_names[i]) != pos_limit_upper_map.end())
		{
			joint_limits_.position_upper.push_back(pos_limit_upper_map[joint_limits_.joint_names[i]]);
		}
		else
		{
			ROS_ERROR("PositionJointPositionController: Unable to find upper position limit  values for joint %s...",
			       joint_limits_.joint_names[i].c_str());
		}
	}  

	position_joint_handles_.resize(NB_JOINTS);
	for (size_t i = 0; i < NB_JOINTS; ++i) {
		try {
			position_joint_handles_[i] = position_joint_interface_->getHandle(joint_limits_.joint_names[i]);
		} catch (const hardware_interface::HardwareInterfaceException& e) {
			ROS_ERROR_STREAM(
		  	"PositionJointPositionController: Exception getting joint handles: " << e.what());
			return false;
		}
	}

	dynamic_reconfigure_joint_controller_params_node_ =
	ros::NodeHandle("position_joint_position_controller/arm/controller_parameters_config");

	dynamic_server_joint_controller_params_ = std::make_unique<
	dynamic_reconfigure::Server<franka_panda_ip_controllers::joint_controller_paramsConfig>>(
	dynamic_reconfigure_joint_controller_params_node_);

	dynamic_server_joint_controller_params_->setCallback(
	boost::bind(&PositionJointPositionController::jointControllerParamCallback, this, _1, _2));

	return true;
}

void PositionJointPositionController::starting(const ros::Time& /* time */) {


	std::vector<double> initial_pos_;
	initial_pos_.resize(NB_JOINTS);

	for (size_t i = 0; i < 7; ++i) {
		initial_pos_[i] = position_joint_handles_[i].getPosition();
	}
	
	
	
	pos_d_buffer_.writeFromNonRT(initial_pos_);
	prev_pos_buffer_.writeFromNonRT(initial_pos_);
	pos_d_target_buffer_.writeFromNonRT(initial_pos_);
	
	for (size_t i=0; i<7; ++i)
	{
	  ROS_INFO_STREAM("Starting : " << initial_pos_[i] << ", " );
    }
    
}

void PositionJointPositionController::update(const ros::Time& time,
                                            const ros::Duration& period) {
      
    
	std::vector<double> pos_d = *pos_d_buffer_.readFromRT();
	std::vector<double> prev_pos = *prev_pos_buffer_.readFromRT();
	std::vector<double> & pos_d_target = *pos_d_target_buffer_.readFromRT();
  
	for (size_t i = 0; i < NB_JOINTS; ++i) {
		position_joint_handles_[i].setCommand(pos_d[i]);	
	}
	
	double filter_val = filter_joint_pos_ * filter_factor_;
	
	for (size_t i = 0; i < NB_JOINTS; ++i) {
		prev_pos[i] = position_joint_handles_[i].getPosition();
		pos_d[i] = filter_val * pos_d_target[i] + (1.0 - filter_val) * pos_d[i];
	}
	
	
	pos_d_buffer_.writeFromNonRT(pos_d);
	prev_pos_buffer_.writeFromNonRT(prev_pos);
	
  
	// update parameters changed online either through dynamic reconfigure or through the interactive
	// target by filtering
	filter_joint_pos_ = param_change_filter_ * target_filter_joint_pos_ + (1.0 - param_change_filter_) * filter_joint_pos_;
  
	// get state variables
	franka::RobotState robot_state = state_handle_->getRobotState();
	std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
		
	// convert to Eigen
	Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
	Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
	Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
        
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.linear());
        
        /*
        * NE_T_EE -> base
        * O_T_EE -> doigt gauche
        * F_T_NE -> link_0
        * F_T_EE -> likn_0
        */
        
        Eigen::Matrix<double, 6, 1> cartesian_velocity = jacobian * dq;
        
        obs_orientation_msg_.data.clear();
        
        obs_orientation_msg_.data.push_back(position[0]);
        obs_orientation_msg_.data.push_back(position[1]);
        obs_orientation_msg_.data.push_back(position[2]);
        
        obs_orientation_msg_.data.push_back(cartesian_velocity[0]);
        obs_orientation_msg_.data.push_back(cartesian_velocity[1]);
        obs_orientation_msg_.data.push_back(cartesian_velocity[2]);
        
        obs_orientation_msg_.data.push_back(orientation.x());
        obs_orientation_msg_.data.push_back(orientation.y());
        obs_orientation_msg_.data.push_back(orientation.z());
        obs_orientation_msg_.data.push_back(orientation.w());


        obs_orientation_msg_.data.push_back(cartesian_velocity[3]);
        obs_orientation_msg_.data.push_back(cartesian_velocity[4]);
        obs_orientation_msg_.data.push_back(cartesian_velocity[5]);
        
        if (realtime_obs_orientation_pub_->trylock())
        {
			 realtime_obs_orientation_pub_->msg_  = obs_orientation_msg_;
			 realtime_obs_orientation_pub_->unlockAndPublish();
        }
   
}

bool PositionJointPositionController::checkPositionLimits(const std::vector<double> & positions)
{
	// bool retval = true;
	for (size_t i = 0;  i < 7; ++i){
    		if (!((positions[i] <= joint_limits_.position_upper[i]) && (positions[i] >= joint_limits_.position_lower[i]))){
      			return true;
		}
	}

  	return false;
}

void PositionJointPositionController::jointControllerParamCallback(franka_panda_ip_controllers::joint_controller_paramsConfig& config,
                               uint32_t level){
	target_filter_joint_pos_ = config.target_filter_joint_pos_delta;
	filter_joint_pos_ = config.filter_joint_pos_delta;
	filter_factor_ = config.filter_factor_delta;
	param_change_filter_ = config.param_change_filter_delta;
}

}  // namespace franka_ros_controllers

PLUGINLIB_EXPORT_CLASS(franka_panda_ip_controllers::PositionJointPositionController,
                       controller_interface::ControllerBase)
