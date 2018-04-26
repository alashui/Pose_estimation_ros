#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/client/simple_action_client.h>
#include <localization_ros/hi_motionAction.h>

/*
#goal definition
float32 rotate_radius
float32 rotate_angle
float32 foward_dist
---
#result definition
float32[] ranges
float32[] angles
---
#feedback
float32 range
float32 angle
*/

using namespace localization_ros;
typedef actionlib::SimpleActionClient<hi_motionAction> Client;

class hi_motionActClient
{
  public:
	hi_motionActClient(std::string name) : ac_(name, true)
	{
		ROS_INFO("Waiting for action server to start.");
		ac_.waitForServer();
		ROS_INFO("Action server started, sending goal.");
	}

	void goalSet(float radius,float angle,float dist)
	{
		goal_.rotate_radius=radius;
		goal_.rotate_angle=angle*M_PI/180;
		goal_.foward_dist=dist;
	}
	void goalSend()
	{
		// Need boost::bind to pass in the 'this' pointer
		ac_.sendGoal(goal_,
				    boost::bind(&hi_motionActClient::doneCb, this, _1, _2),
				    boost::bind(&hi_motionActClient::activeCb,this),
				    boost::bind(&hi_motionActClient::feedbackCb,this, _1) );

	}

	void doneCb(const actionlib::SimpleClientGoalState& state,
		      const hi_motionResultConstPtr& result)
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		//ROS_INFO("Answer: %f", result->ranges.back());
		//ROS_INFO("Answer: %f", result->angles.back());
		ros::shutdown();
	}
	// Called once when the goal becomes active
	void activeCb()
	{
	    ROS_INFO("Goal just went active");
	}

	// Called every time feedback is received for the goal
	void feedbackCb(const hi_motionFeedbackConstPtr& feedback)
	{
	    //ROS_INFO("Got Feedback  %f", feedback->range);
	    //ROS_INFO("Got Feedback  %f", feedback->angle);
		ROS_INFO("Got Feedback  %s",feedback->state_motion.c_str());
	}

private:
	Client ac_;
	hi_motionGoal goal_;	
};

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_hi_motionAC");
  hi_motionActClient hi_motionAC("hi_motion");
  
  hi_motionAC.goalSet(0,360,1);
  hi_motionAC.goalSend();
  ros::spin();
  return 0;
}


