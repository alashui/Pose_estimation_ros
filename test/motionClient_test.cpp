#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib/client/simple_action_client.h>
#include <localization_ros/hi_motionAction.h>


#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <vector>
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
  private:
	Client ac_;
	hi_motionGoal goal_;

	ros::NodeHandle nh_;
	ros::Subscriber laser_scan_sub_;
	
	const  double FORWARD_SPEED_MPS = 0.2;
    const  double MIN_SCAN_ANGLE_RAD = -10.0/180*M_PI;
    const  double MAX_SCAN_ANGLE_RAD = +10.0/180*M_PI;
    const  float MIN_PROXIMITY_RANGE_M = 0.6;

	int collision_warning_count_;

  public:
  	bool goalSended_flag_ ;

  	tf::TransformListener listener_;
	tf::StampedTransform transform_;
	bool tf_base_enable_;

	std::string odom_frame_;
	std::string base_frame_;

	struct PoseLaserData 
	{
		float pose_x;
		float pose_y;
		double pose_theta;

		float laser_range;
	};
	std::vector<PoseLaserData> poseLaserDatas_vec_;

	hi_motionActClient(std::string name);
	bool goalSend(float radius,float angle,float dist);
	void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan);


	void doneCb(const actionlib::SimpleClientGoalState& state,
		        const hi_motionResultConstPtr& result )
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		goalSended_flag_=false;
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
	
	
};

hi_motionActClient::hi_motionActClient(std::string name) :ac_(name, true),
														  collision_warning_count_(0),
														  odom_frame_("/odom"),
														  tf_base_enable_(false),
														  goalSended_flag_ (false)
{
	ROS_INFO("Waiting for action server to start.");
	ac_.waitForServer();
	ROS_INFO("Action server started, sending goal.");

	//确定base_frame
	try
	{
		listener_.waitForTransform(odom_frame_, "/base_footprint", ros::Time(), ros::Duration(2.0) );
		base_frame_ = "/base_footprint";
		ROS_INFO("base_frame = /base_footprint");
		tf_base_enable_ = true;
	}
	catch (tf::TransformException & ex)
	{
		try
		{
			listener_.waitForTransform(odom_frame_, "/base_link", ros::Time(), ros::Duration(2.0) );
			base_frame_ = "/base_link";
			ROS_INFO("base_frame = /base_link");
			tf_base_enable_ = true;
		}
		catch (tf::TransformException ex)
		{
			ROS_INFO("Cannot find transform between /odom and /base_link or /base_footprint");
		}
	}

	laser_scan_sub_ = nh_.subscribe("/scan",1,&hi_motionActClient::laserScanCallback,this); 

}

bool hi_motionActClient::goalSend(float radius,float angle,float dist)
{
	if(!tf_base_enable_)
	{
		ROS_INFO("Cannot send goal because cannot find transform ...  ");
		return false;
	}
	poseLaserDatas_vec_.clear();//清除上一次任务记录的传感器信息	

	goal_.rotate_radius=radius;
	goal_.rotate_angle=angle*M_PI/180;
	goal_.foward_dist=dist;
	// Need boost::bind to pass in the 'this' pointer
	ac_.sendGoal(goal_,
				boost::bind(&hi_motionActClient::doneCb, this, _1, _2),
				boost::bind(&hi_motionActClient::activeCb,this),
				boost::bind(&hi_motionActClient::feedbackCb,this, _1) );

	goalSended_flag_= true;
	return true;
}
void hi_motionActClient::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	if(!goalSended_flag_) 
		return;
	int minIndex=ceil(( MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan-> angle_increment);
	int maxIndex = floor(( MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan-> angle_increment);

	float closestRange(0);
	for(int currIndex = minIndex ; currIndex <= maxIndex; currIndex++)
	{ 
		if(scan->ranges[currIndex]==scan->ranges[currIndex])//判断是否为nan
		{
			closestRange = scan->ranges[currIndex];  //为最近距离赋初值
			break;
		}
	}
	for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) 
	{
		if(scan->ranges[currIndex]==scan->ranges[currIndex])//判断是否为nan
			if (scan->ranges[currIndex] < closestRange) 
			{
				closestRange = scan->ranges[currIndex];
			}
	}

	//记录传感器数据及对应位姿
	PoseLaserData pose_laser;
	listener_.lookupTransform(odom_frame_, base_frame_, ros::Time(0), transform_);
	pose_laser.pose_x = transform_.getOrigin().x();
	pose_laser.pose_y = transform_.getOrigin().y();
	pose_laser.pose_theta = tf::getYaw(transform_.getRotation());
	pose_laser.laser_range = closestRange;
	poseLaserDatas_vec_.push_back(pose_laser);

	//避免碰撞
	ROS_INFO_STREAM("Closest range: " << closestRange);
	if (closestRange < MIN_PROXIMITY_RANGE_M) 
	{
		ROS_INFO("collision warning !");
		collision_warning_count_++;
		if(collision_warning_count_ > 3)
		{
			ac_.cancelGoal();			
		}

		//keepMoving = false;
	}

}


int main (int argc, char **argv)
{
	ros::init(argc, argv, "test_hi_motionAC");
	hi_motionActClient hi_motionAC("hi_motion");

	hi_motionAC.goalSend(0,360,0);
	ros::spin();

	for (auto poselaser: hi_motionAC.poseLaserDatas_vec_)
	{
		std::cout <<"pose_x: " << poselaser.pose_x <<std::endl;
		std::cout <<"pose_y: "<< poselaser.pose_y <<std::endl;
		std::cout <<"pose_theta: "<< poselaser.pose_theta <<std::endl;
		std::cout <<"laser_range: " << poselaser.laser_range <<std::endl;
	}

	return 0;
}


