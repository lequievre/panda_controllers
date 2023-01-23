/*
 *  Laurent LEQUIEVRE
 *  Research Engineer, CNRS (France)
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
 *  rosservice call ip_collision_behavior/set_high_values
 *  rosservice call ip_collision_behavior/set_low_values
 *  rosservice call ip_collision_behavior/set_maximum_values
 */

#include "ros/ros.h"
#include <std_srvs/Trigger.h>
#include <franka_msgs/SetFullCollisionBehavior.h>

class collisionBehaviorIPServer {

    public :
    
      collisionBehaviorIPServer()
      {
          panda_collision_behavior_client_ = nh_.serviceClient<franka_msgs::SetFullCollisionBehavior>("/franka_control/set_full_collision_behavior");
          high_values_service_ = nh_.advertiseService("ip_collision_behavior/set_high_values", &collisionBehaviorIPServer::high_values_cb, this);
          low_values_service_ = nh_.advertiseService("ip_collision_behavior/set_low_values", &collisionBehaviorIPServer::low_values_cb, this);
          maximum_values_service_ = nh_.advertiseService("ip_collision_behavior/set_maximum_values", &collisionBehaviorIPServer::maximum_values_cb, this);
		
          ROS_WARN("collisionBehaviorIPServer Server Created !");
      }

      ~collisionBehaviorIPServer()
      {
          ROS_WARN("collisionBehaviorIPServer Server Destructed !");
      }

      void startServer() { ros::spin(); }
    
    private:

      ros::NodeHandle nh_;

      ros::ServiceClient panda_collision_behavior_client_;
      ros::ServiceServer high_values_service_;
      ros::ServiceServer low_values_service_;
      ros::ServiceServer maximum_values_service_;

      bool low_values_cb(std_srvs::Trigger::Request &req,std_srvs::Trigger::Response &res)
      {
          franka_msgs::SetFullCollisionBehavior srv;
          srv.request.lower_torque_thresholds_acceleration = {{-30, -30, -30, -30, -10, -10, -10}};
          srv.request.upper_torque_thresholds_acceleration = {{30, 30, 30, 30, 10, 10, 10}};
          srv.request.lower_torque_thresholds_nominal = {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}};
          srv.request.upper_torque_thresholds_nominal = {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};
          srv.request.lower_force_thresholds_acceleration = {{-30, -30, -30,  -10, -10, -10}};
          srv.request.upper_force_thresholds_acceleration  = {{30, 30, 30, 10, 10, 10}};
          srv.request.lower_force_thresholds_nominal = {{100.0, 100.0, 100.0, 7.5, 7.5, 7.5}};
          srv.request.upper_force_thresholds_nominal = {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};

          panda_collision_behavior_client_.call(srv);

          ROS_INFO("Low Values Set");
          return true;
      }

      bool high_values_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
      {
          franka_msgs::SetFullCollisionBehavior srv;
          srv.request.lower_torque_thresholds_acceleration = {{-80, -80, -80, -80, -30, -30, -30}};
          srv.request.upper_torque_thresholds_acceleration = {{80, 80, 80, 80, 30, 30, 30}};
          srv.request.lower_torque_thresholds_nominal = {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}};
          srv.request.upper_torque_thresholds_nominal = {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};
          srv.request.lower_force_thresholds_acceleration = {{-80, -80, -80,  -30, -30, -30}};
          srv.request.upper_force_thresholds_acceleration  = {{80, 80, 80, 30, 30, 30}};
          srv.request.lower_force_thresholds_nominal = {{100.0, 100.0, 100.0, 7.5, 7.5, 7.5}};
          srv.request.upper_force_thresholds_nominal = {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};

          panda_collision_behavior_client_.call(srv);

          ROS_INFO("High Values Set");

          return true;
      }
    
      bool maximum_values_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
      {
          franka_msgs::SetFullCollisionBehavior srv;
          srv.request.lower_torque_thresholds_acceleration = {{-30, -30, -30, -30, -10, -10, -10}};
          srv.request.upper_torque_thresholds_acceleration = {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};
          srv.request.lower_torque_thresholds_nominal = {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}};
          srv.request.upper_torque_thresholds_nominal = {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};
          srv.request.lower_force_thresholds_acceleration = {{-30, -30, -30,  -10, -10, -10}};
          srv.request.upper_force_thresholds_acceleration  = {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};
          srv.request.lower_force_thresholds_nominal = {{100.0, 100.0, 100.0, 7.5, 7.5, 7.5}};
          srv.request.upper_force_thresholds_nominal = {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}};

          panda_collision_behavior_client_.call(srv);

          ROS_INFO("Maximum Values Set");

          return true;
      }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "collisionBehaviorIPServer server init !");

    collisionBehaviorIPServer ip_collision_server;

    ip_collision_server.startServer();

    return 0;
}
