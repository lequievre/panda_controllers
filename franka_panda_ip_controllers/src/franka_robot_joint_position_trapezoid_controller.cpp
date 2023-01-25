#include <franka_robot_joint_position_trapezoid_controller.h>

#include <pluginlib/class_list_macros.h>
#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include<algorithm>

namespace franka_panda_ip_controllers
{
	
	void JointPositionTrapezoidController::commandCB(const sensor_msgs::JointStateConstPtr& msg)
    {
		ROS_INFO("***** START JointPositionTrapezoidController::commandCB ************");
		
		if (msg->position.size() == NB_JOINTS && msg->velocity.size() == NB_JOINTS)
		{
		
		    desired_joint_positions_buffer_.writeFromNonRT(msg->position);
		    desired_joint_velocities_buffer_.writeFromNonRT(msg->velocity);
		    
		    /*
		    std::vector<double> joint_velocities_in_radian;
		    for_each(begin(msg->velocity), end(msg->velocity),
                [&](auto joint_velocity_in_degree) {
                    joint_velocities_in_radian.push_back(joint_velocity_in_degree*DEG_TO_RAD);
                });
                
            desired_joint_velocities_buffer_.writeFromNonRT(joint_velocities_in_radian);
		    */
		    
		}
		else
		{
		
		    ROS_ERROR_STREAM("Dimension of position command (" << msg->position.size() << ")  and velocity command (" <<  msg->velocity.size() << ") does not match number of joints (" << NB_JOINTS << ")! Not executing!");
			return; 
		}
		
		ROS_INFO("***** FINISH JointPositionTrapezoidController::commandCB ************");
	}

	bool JointPositionTrapezoidController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle){
	
	
	    ROS_INFO_STREAM(" ******* \n Begin init JointPositionTrapezoidController ! \n *******");
	    
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
            ROS_ERROR_STREAM("JointPositionTrapezoidController: Could not read parameter arm_id");
            return false;
        }


        auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr) {
            ROS_ERROR_STREAM("JointPositionTrapezoidController: Error getting model interface from hardware");
            return false;
        }
        
        try 
        {
            model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
        }
        catch (hardware_interface::HardwareInterfaceException& ex) 
        {
                    ROS_ERROR_STREAM("JointPositionTrapezoidController: Exception getting model handle from interface: " << ex.what());
                    return false;
        }

        auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr) {
            ROS_ERROR_STREAM("JointPositionTrapezoidController: Error getting state interface from hardware");
            return false;
        }
        
        try 
        {
            state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
        }
        catch (hardware_interface::HardwareInterfaceException& ex) 
        {
            ROS_ERROR_STREAM("JointPositionTrapezoidController: Exception getting state handle from interface: " << ex.what());
            return false;
        }
        
        auto* position_joint_interface = robot_hw->get<hardware_interface::PositionJointInterface>();
        if (position_joint_interface == nullptr) {
            ROS_ERROR_STREAM("JointPositionTrapezoidController: Error getting position joint interface from hardware");
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

		 // Init commands buffer with initial 0.0 values
        desired_joint_positions_buffer_.writeFromNonRT(std::vector<double>(NB_JOINTS, 0.0));
        desired_joint_velocities_buffer_.writeFromNonRT(std::vector<double>(NB_JOINTS, 0.0));
      
		// Set joint commands size equal to joint number
		joint_commands_.resize(NB_JOINTS);
		std::fill(joint_commands_.begin(), joint_commands_.end(), 0.0);
		
		// Set Velocity limits size equal to joint number
		current_joint_velocities_commands_.resize(NB_JOINTS);
		std::fill(current_joint_velocities_commands_.begin(), current_joint_velocities_commands_.end(), 0.0);
		
		// Original acceleration limits from Franka-Emika. They are divided by d, in order to make controller smooth. Second joint affected by the movement of the other joints which is relatively smaller than others.
		double d = 50.0;
		joint_accelerations_ = {15.0/d,10.0/d,10.0/d,12.5/d,15.0/d,20.0/d,20.0/d}; // rad/s^-2

        // subscribe to JointState command
        sub_command_ = node_handle.subscribe("command", 20, &JointPositionTrapezoidController::commandCB, this, ros::TransportHints().reliable().tcpNoDelay());

		position_joint_handles_.resize(NB_JOINTS);
		for (size_t i = 0; i < NB_JOINTS; ++i)
		{
			try{
				position_joint_handles_[i] = position_joint_interface->getHandle(joint_names[i]);
			}									
			catch(const hardware_interface::HardwareInterfaceException& e){
				ROS_ERROR_STREAM("JointPositionTrapezoidController: Could not get joint handles: " << e.what());
				return false;
			}
		}

		 ROS_INFO_STREAM(" ******* \n End init JointPositionTrapezoidController ! \n *******");
		
		return true;
	}

	void JointPositionTrapezoidController::starting(const ros::Time&) {

        ROS_INFO_STREAM(" ******* \n Begin starting JointPositionTrapezoidController ! \n *******");
        
	    //Set the first command to current joint positions
		for (size_t i = 0; i < NB_JOINTS; ++i) {
			joint_commands_[i] = position_joint_handles_[i].getPosition();
		}
		desired_joint_positions_buffer_.writeFromNonRT(joint_commands_);
		
		elapsed_time_ = ros::Duration(0.0);
		
		ROS_INFO_STREAM(" ******* \n End starting JointPositionTrapezoidController ! \n *******");
	}

	void JointPositionTrapezoidController::update(const ros::Time&, const ros::Duration& period) {
		
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
        
        // Read topic command CB buffer
		std::vector<double> & desired_joint_positions = *desired_joint_positions_buffer_.readFromRT();
		std::vector<double> & desired_joint_velocities = *desired_joint_velocities_buffer_.readFromRT();
		
		//Total time passed since controller started in ROS::time
		elapsed_time_ += period;
		
		//Total time passed since controller started converted to the double.
		double seconds_passed = elapsed_time_.toSec();
		
		//Set distance to goal point to 0 in order to check with each new loop. 	
		double distance_to_goal_point = 0.0;

		//For each joint check if they are at goal position or move them to the goal position.
		for (size_t joint_id = 0; joint_id < NB_JOINTS; ++joint_id)
		{
			//Distance between goal position and joint's current position. 
			distance_to_goal_point = desired_joint_positions[joint_id] - position_joint_handles_[joint_id].getPosition();

			//Since joints are turning both -/+ sides, we need it as a multiplier. 
			int direction = std::signbit(distance_to_goal_point)==1?-1:1;

			//If distance_to_goal_point is more than this value keep joints working. Less than 0.001 causes problems becareful with oscillations.
			double goal_distance_check_value = 0.002;				

			if (std::fabs(distance_to_goal_point)>goal_distance_check_value || seconds_passed <= 0.001) // Seconds_passed prevents oscillations at the first call of the controller. DO NOT DELETE!
			{				
				double time_needed_to_stop_with_current_vel_and_acc = position_joint_handles_[joint_id].getVelocity() / joint_accelerations_[joint_id]; 
				
				double distance_needed_to_stop_with_current_vel = (joint_accelerations_[joint_id]*time_needed_to_stop_with_current_vel_and_acc*time_needed_to_stop_with_current_vel_and_acc)/2.0;				
				
				//Deceleration
				if (
						(
							( direction == -1 && (position_joint_handles_[joint_id].getPosition() < desired_joint_positions[joint_id] + distance_needed_to_stop_with_current_vel) ) 
							|| 
							( direction == 1 && (position_joint_handles_[joint_id].getPosition() > desired_joint_positions[joint_id] - distance_needed_to_stop_with_current_vel) )
						) 
						&& 
						std::fabs(position_joint_handles_[joint_id].getVelocity())>(0.1*DEG_TO_RAD)
					)
				{	
					current_joint_velocities_commands_[joint_id] -= (direction) * (joint_accelerations_[joint_id]*period.toSec());					
				}
				//Acceleration
				else if ( std::fabs(desired_joint_velocities[joint_id] - std::fabs(position_joint_handles_[joint_id].getVelocity())) > (1.0*DEG_TO_RAD) )
				{
					current_joint_velocities_commands_[joint_id] += (direction) * joint_accelerations_[joint_id]*period.toSec();				
				}
				//Constant Velocity
				else{					
					current_joint_velocities_commands_[joint_id] = (direction) * desired_joint_velocities[joint_id];
				}

				joint_commands_[joint_id] += current_joint_velocities_commands_[joint_id]*period.toSec();				

				position_joint_handles_[joint_id].setCommand(joint_commands_[joint_id]);
			}
		}
	}
	
}

// Implementation name_of_your_controller_package::NameOfYourControllerClass,
PLUGINLIB_EXPORT_CLASS(franka_panda_ip_controllers::JointPositionTrapezoidController,
	controller_interface::ControllerBase)
