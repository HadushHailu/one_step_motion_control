#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <one_step_motor_control/OsmcAction.h>
#include <geometry_msgs/PoseStamped.h>

class OsmcServer
{
private:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<one_step_motor_control::OsmcAction> as_;
	std::string action_name_;
	one_step_motor_control::OsmcFeedback feedback_;
	one_step_motor_control::OsmcResult result_;
    
	ros::Subscriber sub_;

public:
    
  OsmcServer(std::string name) : 
    as_(nh_, name, boost::bind(&OsmcServer::executeCB, this, _1),false),
    action_name_(name)
  {
    as_.start();
  }

  ~OsmcServer(void)
  {
  }

 void executeCB(const one_step_motor_control::OsmcGoalConstPtr &msg){

	 //helper variables
	 ros::Rate r(5);
	 bool sucess = true;

	 //get goal
	 geometry_msgs::PoseStamped goal;
	 goal = msg->target_pose;
	 ROS_INFO("Robot Position: (%f,%f)",goal.pose.position.x,goal.pose.position.y);
	 ROS_INFO("Robot Orientation: %f",goal.pose.orientation.z);
	as_.setSucceeded();
 }
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "osmc_serve");

  OsmcServer server_node ("osmc");
  ros::spin();

  return 0;
}

