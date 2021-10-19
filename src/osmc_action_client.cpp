#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <one_step_motion_control/OsmcAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>


typedef actionlib::SimpleActionClient<one_step_motion_control::OsmcAction> Client;

class OsmcClient
{
private:
	Client ac_;
	ros::NodeHandle nh_;
	ros::Subscriber sub_;
	std::string action_name_;
	
public:
  OsmcClient(std::string name) : action_name_(name),ac_(name, true)
  {
    
    sub_ = nh_.subscribe("/osmc_simple/goal", 100, &OsmcClient::goalMsgCallback,this);
  }

  void goalMsgCallback(const geometry_msgs::PoseStamped msg){
	
    ROS_INFO("Waiting for Osmc action server to start.");
    ac_.waitForServer();
    ROS_INFO("Osmc Action server started, sending goal.");

    one_step_motion_control::OsmcGoal goal;
    goal.target_pose = msg;
    ac_.sendGoal(goal,
                boost::bind(&OsmcClient::doneCb, this, _1, _2),
                Client::SimpleActiveCallback(),
                Client::SimpleFeedbackCallback());

   
  
  }


  void doneCb(const actionlib::SimpleClientGoalState& state,
              const one_step_motion_control::OsmcResultConstPtr& result)
  {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    //ROS_INFO("Answer: %i", result->sequence.back());
    //ros::shutdown();
  }
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "osmc_client");
  OsmcClient client_node("osmc");
  ros::spin();
  return 0;
}
