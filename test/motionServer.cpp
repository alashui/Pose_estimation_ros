#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <localization_ros/hi_motionAction.h>

#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
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

class hi_motionActServ
{
private:
	//actionlib使用所需的相关变量
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<localization_ros::hi_motionAction> as_;
	std::string action_name_;

	localization_ros::hi_motionFeedback feedback_;
	localization_ros::hi_motionResult result_;
	
	//运动控制相关
	ros::Publisher cmdvel_pub_;//发布速度
	std::string topic_vel_;	 //发布速度的话题
	  
	const float linear_speed_ = 0.2;//linear speed : 0.2m/s	
	const float angular_speed_ = 0.3;//rotation speed : 0.5 rad/s
	const float MAX_linear_speed_ =0.5;
		
	const double angular_tolerance_ = 2.5*M_PI/180; //角度误差容忍

public:
    
	hi_motionActServ(std::string name) : 
	//as_(nh_, name, false),
	as_(nh_, name, boost::bind(&hi_motionActServ::goalCB, this, _1), false),
	action_name_(name)
	{
		as_.registerPreemptCallback(boost::bind(&hi_motionActServ::preemptCB, this));
		topic_vel_ = "/mobile_base/commands/velocity";
		cmdvel_pub_ = nh_.advertise<geometry_msgs::Twist>(topic_vel_, 1);
	}

	~hi_motionActServ(void)
	{
	}

	void Start();
	void goalCB(const localization_ros::hi_motionGoalConstPtr &goal);
	void preemptCB();
};

void hi_motionActServ::Start()
{
	as_.start();
	ROS_INFO("start over");
}
void hi_motionActServ::goalCB(const localization_ros::hi_motionGoalConstPtr &goal)
{
	ROS_INFO("goal:rotate %f degree with radius %f,and foward %f m",
				(goal->rotate_angle*180)/M_PI,goal->rotate_radius,goal->foward_dist);

	//signal(SIGINT, shutdown);
	ROS_INFO("move!!!...");

//任务执行过程
	tf::TransformListener listener;
	tf::StampedTransform transform;
	//Find out if the robot uses /base_link or /base_footprint
	std::string odom_frame = "/odom";
	std::string base_frame;
	try
	{
	listener.waitForTransform(odom_frame, "/base_footprint", ros::Time(), ros::Duration(2.0) );
	base_frame = "/base_footprint";
	ROS_INFO("base_frame = /base_footprint");
	}
	catch (tf::TransformException & ex)
	{
		try
		{
		listener.waitForTransform(odom_frame, "/base_link", ros::Time(), ros::Duration(2.0) );
		base_frame = "/base_link";
		ROS_INFO("base_frame = /base_link");
		}
		catch (tf::TransformException ex)
		{
			ROS_INFO("Cannot find transform between /odom and /base_link or /base_footprint");
			cmdvel_pub_.publish(geometry_msgs::Twist());
			ros::shutdown();
		}
	}

	double rate = 20;
	ros::Rate loopRate(rate);
	geometry_msgs::Twist speed;

	//旋转过程
	if(goal->rotate_radius >= 0)
	{
		speed.angular.z = angular_speed_; // 设置角速度，正为左转，负为右转
		speed.linear.x = goal->rotate_radius*angular_speed_; //r=v/w
		while(speed.linear.x > MAX_linear_speed_)  //超过最大线速度则调小角速度
		{
			speed.angular.z -= 0.01;
			speed.linear.x = goal->rotate_radius*speed.angular.z;
		}
		ROS_INFO("rotation...!");    
		double last_angle = fabs(tf::getYaw(transform.getRotation()));//Track the last angle measured   
		double turn_angle = 0;//Track how far we have turned
		while( (fabs(turn_angle + angular_tolerance_) < goal->rotate_angle) && (ros::ok()) )
		{
			if(!as_.isActive())break;//判断任务是否还要执行

			cmdvel_pub_.publish(speed);
			loopRate.sleep();
			// Get the current rotation
			listener.lookupTransform(odom_frame, base_frame, ros::Time(0), transform);
			double rotation = fabs(tf::getYaw(transform.getRotation()));

			//Compute the amount of rotation since the last loop
			double delta_angle = fabs(rotation - last_angle);

			//Add to the running total
			turn_angle += delta_angle;
			last_angle = rotation;
		}
		//Stop the robot before the rotation
		cmdvel_pub_.publish(geometry_msgs::Twist());
		ros::Duration(1).sleep(); // sleep for  a second
	}
	//直走过程
	if(goal->foward_dist > 0)
	{
		speed.linear.x=linear_speed_ ;
		speed.angular.z =0;
		ROS_INFO("go straight...!");

		listener.lookupTransform(odom_frame, base_frame, ros::Time(0), transform);
		float x_start = transform.getOrigin().x();
		float y_start = transform.getOrigin().y();
		// Keep track of the distance traveled
		float distance = 0;
		while( (distance < goal->foward_dist) && (ros::ok()) )
		{
			if(!as_.isActive())break;//判断任务是否还要执行	
			//Publish the Twist message and sleep 1 cycle
			cmdvel_pub_.publish(speed);
			loopRate.sleep();
			listener.lookupTransform(odom_frame, base_frame, ros::Time(0), transform);
			//Get the current position
			float x = transform.getOrigin().x();
			float y = transform.getOrigin().y();
			//Compute the Euclidean distance from the start
			distance = sqrt(pow((x - x_start), 2) +  pow((y - y_start), 2));
		}
		//Stop the robot before the rotation
		cmdvel_pub_.publish(geometry_msgs::Twist());
		ros::Duration(1).sleep(); // sleep for  a second	
	}

    ROS_INFO("move ended!");

/*
	ros::Rate r(1);
	for(int i= 0;i<ceil(goal->foward_dist);i++)
	{
		
			if(!as_.isActive())break;


		feedback_.range=i*goal->rotate_angle;
		feedback_.angle=i*goal->rotate_radius;
	
		as_.publishFeedback(feedback_);
	
		result_.ranges.push_back(feedback_.range);
		result_.angles.push_back(feedback_.angle);
		r.sleep();
		
		ROS_INFO("goalcb runing");
	}
*/

	if(as_.isActive())
		as_.setSucceeded(result_);

}

void hi_motionActServ::preemptCB()
{
	ROS_INFO("%s: Preempted", action_name_.c_str());
	//as_.setAborted(result_);
	if(as_.isActive())
		as_.setPreempted(result_);
}
	



int main(int argc, char** argv)
{
	ros::init(argc, argv, "hi_motionAS");

	hi_motionActServ hi_motionAS("hi_motion");
	hi_motionAS.Start();
	ros::spin();

	return 0;
}


