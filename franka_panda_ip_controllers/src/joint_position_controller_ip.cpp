#include <franka_panda_ip_controllers/joint_position_controller_ip.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <Eigen/Dense>


namespace franka_panda_ip_controllers {


void JointPositionControllerIP::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
{
		ROS_INFO("***** START JointPositionControllerIP::commandCB ************");
		
		if (msg->data.size() != position_joint_handles_.size()) {
			ROS_ERROR_STREAM("Dimension of position command (" << msg->data.size() << ") does not match number of joints (" << position_joint_handles_.size() << ")! Not executing!");
			cmd_flag_ = 0;
		}
		else if (checkPositionLimits(msg->data)) {
			ROS_ERROR_STREAM("JointPositionControllerIP: Commanded positions are beyond allowed position limits.");	
			cmd_flag_ = 0;
        }		
		else {
		 	target_buffer_.writeFromNonRT(msg->data);
		 	cmd_flag_ = 1; // set this flag to 1 to run the update method
	 	}
	 	
	 	for (size_t i=0; i<NB_JOINTS; ++i)
		{
			ROS_INFO_STREAM("commandCB : " << msg->data[i] << ", " );
		}
		   
		ROS_INFO("***** FINISH JointPositionControllerIP::commandCB ************");
	}


bool JointPositionControllerIP::init(hardware_interface::RobotHW* robot_hw,
                                          ros::NodeHandle& node_handle) {

    ROS_INFO("Begin JointPositionControllerIP::init !");
    
	// subscribe to JointState command
	sub_command_ = node_handle.subscribe("command", 20, &JointPositionControllerIP::commandCB, this, ros::TransportHints().reliable().tcpNoDelay());

	target_buffer_.readFromNonRT()->resize(NB_JOINTS,0.0);
	
	motion_target_.resize(NB_JOINTS);
	motion_target_.setZero();
	
	motion_cmd_.resize(NB_JOINTS);
	motion_cmd_.setZero();
	
	motion_current_position_.resize(NB_JOINTS);
	motion_current_position_.setZero();
	
	cmd_flag_ = 0;  // set this flag to 0 to not run the update method

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
	    ROS_ERROR_STREAM("JointPositionControllerIP: Could not read parameter arm_id");
	    return false;
	}

	auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
	if (model_interface == nullptr) {
	    ROS_ERROR_STREAM("JointPositionControllerIP: Error getting model interface from hardware");
	    return false;
	}

	try 
	{
	    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
	}
	catch (hardware_interface::HardwareInterfaceException& ex) 
	{
		    ROS_ERROR_STREAM("JointPositionControllerIP: Exception getting model handle from interface: " << ex.what());
		    return false;
	}

	auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
	if (state_interface == nullptr) {
	    ROS_ERROR_STREAM("JointPositionControllerIP: Error getting state interface from hardware");
	    return false;
	}

	try 
	{
	    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
	}
	catch (hardware_interface::HardwareInterfaceException& ex) 
	{
	    ROS_ERROR_STREAM("JointPositionControllerIP: Exception getting state handle from interface: " << ex.what());
	    return false;
	}

	position_joint_interface_ = robot_hw->get<hardware_interface::PositionJointInterface>();
	if (position_joint_interface_ == nullptr) {
	    ROS_ERROR_STREAM("JointPositionControllerIP: Error getting position joint interface from hardware");
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
		ROS_ERROR("JointPositionControllerIP: Could not parse joint names");
	}
	
	if (joint_limits_.joint_names.size() != NB_JOINTS) {
		ROS_ERROR_STREAM("JointPositionControllerIP: Wrong number of joint names, got "
		     << joint_limits_.joint_names.size() << " instead of " << NB_JOINTS << " names!");
	return false;
	}
	
	std::map<std::string, double> pos_limit_lower_map;
	std::map<std::string, double> pos_limit_upper_map;
	std::map<std::string, double> velocity_limit_map;
	
	if (!node_handle.getParam("joint_config/joint_position_limit/lower", pos_limit_lower_map) ) {
		ROS_ERROR("JointPositionControllerIP: Joint limits parameters not provided, aborting ");
		return false;
	}
	
	if (!node_handle.getParam("joint_config/joint_position_limit/upper", pos_limit_upper_map) ) {
		ROS_ERROR("JointPositionControllerIP: Joint limits parameters not provided, aborting ");
		return false;
	}
	
	if (!node_handle.getParam("joint_config/joint_velocity_limit", velocity_limit_map))
	{
		ROS_ERROR("JointPositionControllerIP: Failed to find joint velocity limits on the param server. Aborting controller init");
		return false;
	}
	
	motion_velocity_limits_.resize(joint_limits_.joint_names.size());
	
	for (size_t i = 0; i < joint_limits_.joint_names.size(); ++i){
		if (pos_limit_lower_map.find(joint_limits_.joint_names[i]) != pos_limit_lower_map.end())
		{
			joint_limits_.position_lower.push_back(pos_limit_lower_map[joint_limits_.joint_names[i]]);
		}
		else
		{
			ROS_ERROR("JointPositionControllerIP: Unable to find lower position limit values for joint %s...",
			       joint_limits_.joint_names[i].c_str());
		}
		if (pos_limit_upper_map.find(joint_limits_.joint_names[i]) != pos_limit_upper_map.end())
		{
			joint_limits_.position_upper.push_back(pos_limit_upper_map[joint_limits_.joint_names[i]]);
		}
		else
		{
			ROS_ERROR("JointPositionControllerIP: Unable to find upper position limit  values for joint %s...",
			       joint_limits_.joint_names[i].c_str());
		}
		if (velocity_limit_map.find(joint_limits_.joint_names[i]) != velocity_limit_map.end())
		{
			joint_limits_.velocity.push_back(velocity_limit_map[joint_limits_.joint_names[i]]);
			motion_velocity_limits_(i) = velocity_limit_map[joint_limits_.joint_names[i]];
			//motion_velocity_limits_(i) = 0.5;
		}
		else
		{
			ROS_ERROR("JointPositionControllerIP: Unable to find velocity limit values for joint %s...",
					   joint_limits_.joint_names[i].c_str());
		}
	}  

	position_joint_handles_.resize(NB_JOINTS);
	for (size_t i = 0; i < NB_JOINTS; ++i) {
		try {
			position_joint_handles_[i] = position_joint_interface_->getHandle(joint_limits_.joint_names[i]);
		} catch (const hardware_interface::HardwareInterfaceException& e) {
			ROS_ERROR_STREAM("JointPositionControllerIP: Exception getting joint handles: " << e.what());
			return false;
		}
	}
	
	// CDD Dynamics
	joint_cddynamics.reset(new motion::CDDynamics(position_joint_handles_.size(),1e-6,1.0));
	joint_cddynamics->SetVelocityLimits(motion_velocity_limits_);
	
	
	// Set rqt reconfigure server
	dynamic_reconfigure_joint_controller_params_node_ =
	ros::NodeHandle("joint_position_controller_ip/arm/controller_parameters_config");

	dynamic_server_joint_controller_params_ = std::make_unique<
	dynamic_reconfigure::Server<franka_panda_ip_controllers::joint_position_controller_ip_paramsConfig>>(
	dynamic_reconfigure_joint_controller_params_node_);

	dynamic_server_joint_controller_params_->setCallback(
	boost::bind(&JointPositionControllerIP::jointPositionControllerIPParamCallback, this, _1, _2));
	
	
	ROS_INFO("End JointPositionControllerIP::init !");
	
	return true;
}

void JointPositionControllerIP::starting(const ros::Time& /* time */) {

    ROS_INFO("***** Begin JointPositionControllerIP::starting ************");
    std::vector<double> initial_pos;
	initial_pos.resize(NB_JOINTS);
	
	for (size_t i=0; i<NB_JOINTS; i++)
	{
		initial_pos[i] = position_joint_handles_[i].getPosition();
		motion_target_(i) = initial_pos[i];
		motion_current_position_(i) = initial_pos[i];
	}
	
	target_buffer_.writeFromNonRT(initial_pos);
	
	joint_cddynamics->SetState(motion_current_position_);
	joint_cddynamics->SetVelocityLimits(motion_velocity_limits_);
	joint_cddynamics->SetDt(0.001);
	joint_cddynamics->SetTarget(motion_target_);
	
	for (size_t i=0; i<NB_JOINTS; ++i)
	{
	  ROS_INFO_STREAM("Starting : " << initial_pos[i] << ", " );
    }
    
	cmd_flag_ = 0;  // set this flag to 0 to not run the update method
	
	ROS_INFO("***** End JointPositionControllerIP::starting ************");
    
}

void JointPositionControllerIP::stopping(const ros::Time& time)
{
	cmd_flag_ = 0;  // set this flag to 0 to not run the update method
}

void JointPositionControllerIP::update(const ros::Time& time,
                                            const ros::Duration& period) {
      
    std::vector<double> & vector_target = *target_buffer_.readFromRT();
	
	for (size_t i=0; i<NB_JOINTS; i++)
	{
		motion_target_(i) = vector_target[i];
		motion_current_position_(i) = position_joint_handles_[i].getPosition();	
	}
	
	if (cmd_flag_) 
	{
            joint_cddynamics->SetState(motion_current_position_);
            cmd_flag_ = 0;
	}
		
	joint_cddynamics->SetVelocityLimits(motion_velocity_limits_);
	joint_cddynamics->SetDt(0.001);
	joint_cddynamics->SetTarget(motion_target_);
	joint_cddynamics->Update();
	joint_cddynamics->GetState(motion_cmd_);
  
	
	for (size_t i = 0; i < NB_JOINTS; ++i) {
		position_joint_handles_[i].setCommand(motion_cmd_(i));
	}
	
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

bool JointPositionControllerIP::checkPositionLimits(const std::vector<double> & positions)
{
	// bool retval = true;
	for (size_t i = 0;  i < 7; ++i){
    		if (!((positions[i] <= joint_limits_.position_upper[i]) && (positions[i] >= joint_limits_.position_lower[i]))){
      			return true;
		}
	}

  	return false;
}


void JointPositionControllerIP::jointPositionControllerIPParamCallback(franka_panda_ip_controllers::joint_position_controller_ip_paramsConfig& config,
                               uint32_t level){
                               
    ROS_INFO_STREAM("Change factor velocity : " << config.factor_velocity);
                
    joint_cddynamics->SetWn(config.factor_velocity);
                               
	/*target_filter_joint_pos_ = config.target_filter_joint_pos_delta;
	filter_joint_pos_ = config.filter_joint_pos_delta;
	filter_factor_ = config.filter_factor_delta;
	param_change_filter_ = config.param_change_filter_delta;*/
}


}  // namespace franka_panda_ip_controllers

PLUGINLIB_EXPORT_CLASS(franka_panda_ip_controllers::JointPositionControllerIP,
                       controller_interface::ControllerBase)
