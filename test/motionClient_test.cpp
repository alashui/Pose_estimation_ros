#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <actionlib/client/simple_action_client.h>
#include <localization_ros/hi_motionAction.h>


#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <vector>

#include <fstream>

#include <signal.h>
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
	ros::Subscriber localizationState_sub_;
	
	const  double FORWARD_SPEED_MPS = 0.2;
    const  double MIN_SCAN_ANGLE_RAD = -5.0/180*M_PI;
    const  double MAX_SCAN_ANGLE_RAD = +5.0/180*M_PI;
    const  float MIN_PROXIMITY_RANGE_M = 0.6;

	int collision_warning_count_;




  public:
  	bool goalSended_flag_ ;
	bool actionServe_enable_;

  	tf::TransformListener listener_;
	tf::StampedTransform transform_;
	bool tf_base_enable_;

	std::string odom_frame_;
	std::string base_frame_;

	struct PoseLaserData 
	{
		int index;

		float pose_x;
		float pose_y;
		double pose_theta;

		float laser_range;
	};
	PoseLaserData pose_laser_;
	bool pose_laser_init_;
	std::vector<PoseLaserData> poseLaserDatas_vec_;

	//定位状态相关
	enum LocateState 
	{
		UNKNOW,FIND,LOST
	};
	LocateState  localization_state_;

	////

	hi_motionActClient(std::string name);
	bool goalSend(float radius,float angle,float dist);
	void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laser_scan);
	static double angleToTarget(double theta_current,double theta_target);//计算到目标角度需要转过的角度

	void selfLocalization();
	void explore();

	void state_callback(const std_msgs::String::ConstPtr& msg);

	void doneCb(const actionlib::SimpleClientGoalState& state,
		        const hi_motionResultConstPtr& result )
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		goalSended_flag_=false;
		//ROS_INFO("Answer: %f", result->ranges.back());
		//ROS_INFO("Answer: %f", result->angles.back());

		actionServe_enable_=true;

		//ros::shutdown();
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
		ROS_INFO("Got Feedback  %s",feedback->motion_state.c_str());
	}
	
	
};

hi_motionActClient::hi_motionActClient(std::string name) :ac_(name, true),
														  collision_warning_count_(0),
														  odom_frame_("/odom"),
														  tf_base_enable_(false),
														  goalSended_flag_ (false),
														  pose_laser_init_(false),
														  actionServe_enable_(true),
														  localization_state_(UNKNOW)
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
	localizationState_sub_=nh_.subscribe("localization_state", 1, 
									&hi_motionActClient::state_callback,this);
}

bool hi_motionActClient::goalSend(float radius,float angle,float dist)
{
	if(!tf_base_enable_)
	{
		ROS_INFO("Cannot send goal because cannot find transform ...  ");
		return false;
	}

	poseLaserDatas_vec_.clear();//清除上一次任务记录的传感器信息	
	pose_laser_init_=false;

	goal_.rotate_radius=radius;
	goal_.rotate_angle=angle;
	goal_.foward_dist=dist;
	// Need boost::bind to pass in the 'this' pointer
	ac_.sendGoal(goal_,
				boost::bind(&hi_motionActClient::doneCb, this, _1, _2),
				boost::bind(&hi_motionActClient::activeCb,this),
				boost::bind(&hi_motionActClient::feedbackCb,this, _1) );

	goalSended_flag_= true;
	actionServe_enable_ = false; 
	return true;
}
void hi_motionActClient::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	if(!goalSended_flag_) 
		return;

	//记录传感器数据及对应位姿
	PoseLaserData pose_laser;
	listener_.lookupTransform(odom_frame_, base_frame_, ros::Time(0), transform_);
	
	pose_laser.pose_x = transform_.getOrigin().x();
	pose_laser.pose_y = transform_.getOrigin().y();
	pose_laser.pose_theta = tf::getYaw(transform_.getRotation());
	
	if(	(!pose_laser_init_) ||  //第一次扫描必须记录,之后运动过一定距离才记录一次
		(fabs ( angleToTarget(pose_laser_.pose_theta , pose_laser.pose_theta) ) >= (M_PI * 10.0/180) ) ||
		( sqrt( pow((pose_laser.pose_x-pose_laser_.pose_x),2) +  
		   		pow((pose_laser.pose_y-pose_laser_.pose_y),2)   )   ) >= 0.1  	)
	{
		//找到该次扫描中最近的距离
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
		pose_laser.laser_range = closestRange;


		if(!pose_laser_init_)
		{ 
			pose_laser_init_=true;
			pose_laser_.index=0;
		}
		pose_laser.index = pose_laser_.index+1;
		pose_laser_ = pose_laser;
		poseLaserDatas_vec_.push_back(pose_laser_);

		//避免碰撞 ,连续三次小于允许最近接近距离则停下
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
		else collision_warning_count_=0;
	}
}
double hi_motionActClient::angleToTarget(double theta_current,double theta_target)//计算到目标角度需要转过的角度
{
	theta_current = (theta_current >= 0) ? theta_current : M_PI*2 + theta_current;
	theta_target = (theta_target >= 0) ? theta_target : M_PI*2 + theta_target;

	return (theta_target - theta_current);
}

void hi_motionActClient::selfLocalization()
{
	//首先原地转一圈
	goalSend(0,360.0*M_PI/180.0,0);
	while(!actionServe_enable_)//等待该任务执行完
	{
		if(localization_state_==FIND) return;
	}



	//统计分析周围环境数据
	std::vector<std::vector<PoseLaserData> > poselaser_range3_vec(3);

	for (auto poselaser: poseLaserDatas_vec_)
	{
		if( poselaser.laser_range >= 3)
		{
			poselaser_range3_vec[0].push_back(poselaser);
			poselaser_range3_vec[1].push_back(poselaser);
			poselaser_range3_vec[2].push_back(poselaser);
		}			
		else if( poselaser.laser_range >= 2.5)
		{
			poselaser_range3_vec[1].push_back(poselaser);
			poselaser_range3_vec[2].push_back(poselaser);
		}
		else if( poselaser.laser_range >= 2)
		{
			poselaser_range3_vec[2].push_back(poselaser);
		}
		
	}

	bool moved_flag(false);
	//如果周围空旷则走两个圆形,左侧一个,右侧一个,呈八字形
	for(int i=0;i<3;i++)
	{
		if( poselaser_range3_vec[i].size() == poseLaserDatas_vec_.size()) 
		{
			float radius_temp(0);
			switch(i)
			{
				case 0: radius_temp=0.9;break;   //所有扫描距离都大于3m
				case 1: radius_temp=0.65;break;  //所有扫描距离都大于2.5m
				case 2: radius_temp=0.4;break;   //所有扫描距离都大于2m
			}

			std::cout << "next(all clear): rotate 360 with radius " <<radius_temp <<std::endl;

			goalSend(radius_temp,360.0*M_PI/180.0,0);
			while(!actionServe_enable_)//等待该任务执行完
			{
				if(localization_state_==FIND) return;
			}
			
			goalSend(radius_temp,-360.0*M_PI/180.0,0);
			while(!actionServe_enable_)//等待该任务执行完
			{
				if(localization_state_==FIND) return;
			}

			moved_flag =true;
			break;
		}
	}
	if(moved_flag) return; 

	std::vector<std::vector<std::vector<int> > > index3_vecvecvec(3);//连续扫描到大于(3m/2.5m/2m) 的数据记为一组,记录索引
	std::vector<int > Maxsize_index3_vec(3);//确定含有数据最多的一组
	std::vector<int >index_vec3_Maxsize(3);//这一组有多少个数据

	for(int i=0;i<3;i++)
	{
		if(poselaser_range3_vec[i].size()>0)
		{						
			//找到每组连续扫描到大于(3m/2.5m/2m) 的数据,记录索引
			std::vector<int> index_vec;
			//std::vector<std::vector<int> > index_vecvec;

			int index_temp((* poselaser_range3_vec[i].begin()).index);
			
			for (int jj=0;jj<poselaser_range3_vec[i].size();jj++)
			{
				if(poselaser_range3_vec[i][jj].index==index_temp)
					index_vec.push_back( index_temp++ );
				else 
				{
					
					index3_vecvecvec[i].push_back(index_vec);
					index_vec.clear();
					index_temp=poselaser_range3_vec[i][jj].index;
					index_vec.push_back( index_temp++ );
				}
				
				if(jj==poselaser_range3_vec[i].size()-1)
				{
					index3_vecvecvec[i].push_back(index_vec);
					index_vec.clear();					
				}
				
			}
			//如果首组包含第一次扫描的数据，尾组包含最后一次扫描的数据则这两组为连续（360=0），合并为同一组
			if( (*index3_vecvecvec[i].begin())[0]==(*poseLaserDatas_vec_.begin()).index  &&
			    (*(index3_vecvecvec[i].end()-1))[0] == (*(poseLaserDatas_vec_.end()-1) ).index     )
			{
				for(auto vec_temp : (*index3_vecvecvec[i].begin()) )
				{
					(*(index3_vecvecvec[i].end()-1)).push_back(vec_temp);
				}
				
				index3_vecvecvec[i].erase(index3_vecvecvec[i].begin());
			}

			//int Maxsize_index(0); //确定含有数据最多的一组
			//int index_vec_Maxsize((*index3_vecvecvec[i].begin()).size());//这一组有多少个数据

			index_vec3_Maxsize[i] =  (*index3_vecvecvec[i].begin()).size();
			for (int kk =0;kk <index3_vecvecvec[i].size();kk++)
			{
				if(index3_vecvecvec[i][kk].size() > index_vec3_Maxsize[i])
				{
					index_vec3_Maxsize[i] = index3_vecvecvec[i][kk].size();
					Maxsize_index3_vec[i] = kk;
				}
					
			}

			//如果最多的这组数据有一半以上的数据,说明机器人周围有一半以上的方向在3m/2.5m/2m范围内无障碍物
			//此时控制机器人在合适的方向绕安全半径旋转一周
			if(index_vec3_Maxsize[i] > poseLaserDatas_vec_.size()/2)
			{
				//有一半方向空旷
				//找到起始索引,旋转到这个方向然后走个圆形
				double angle = angleToTarget( (*(poseLaserDatas_vec_.end()-1)).pose_theta,
								(poseLaserDatas_vec_[index3_vecvecvec[i][Maxsize_index3_vec[i]][0]-1]).pose_theta);
				if (fabs(angle)>0)
				{
					std::cout << "next(half): rotate " <<angle*180/M_PI <<" with radius 0 and ";
					goalSend(0,angle,0);   //先旋转到合适方向
					while(!actionServe_enable_)//等待该任务执行完
					{
						if(localization_state_==FIND) return;
					}
					
				}

				float radius_temp(0);
				switch(i)
				{
					case 0: radius_temp=0.9;break;   //所有扫描距离都大于3m
					case 1: radius_temp=0.65;break;  //所有扫描距离都大于2.5m
					case 2: radius_temp=0.4;break;   //所有扫描距离都大于2m
				}

				std::cout << "next(half): rotate 360 with radius " <<radius_temp <<std::endl;
				goalSend(0.9,360.0*M_PI/180.0,0);  //按合适半径走圆形
				while(!actionServe_enable_)//等待该任务执行完
				{
					if(localization_state_==FIND) return;
				}

				moved_flag =true;
				break;				
			}
		}
	}

	if(moved_flag) return; 

	for(int i=0;i<3;i++)
	{
		if(poselaser_range3_vec[i].size()>0)
		{			
			//机器人周围只有某些方向在3m/2.5m/2m范围内无障碍物,则朝无障碍物方向前进1m
			if (index_vec3_Maxsize[i] >= 3)
			{
				//找到中间索引,旋转到这个方向然后朝这个方向直行一米
				//int index_temp3();
				
				//double angle_current ((*(poseLaserDatas_vec_.end()-1)).pose_theta);
				//double angle_target  ((poseLaserDatas_vec_[index3_vecvecvec[i][Maxsize_index3_vec[i]][index_vec3_Maxsize[i]/2]-1]).pose_theta);
				
				//double angle = angleToTarget(angle_current,angle_target);
				
				double angle = angleToTarget( (*(poseLaserDatas_vec_.end()-1)).pose_theta,
								(poseLaserDatas_vec_[index3_vecvecvec[i][Maxsize_index3_vec[i]][index_vec3_Maxsize[i]/2]-1]).pose_theta);

				if (fabs(angle)>0)
				{
					angle = (angle<M_PI) ? angle : (angle-2*M_PI);

					std::cout << "next(once): rotate " <<angle *180/M_PI <<" with radius 0 and ";	
					goalSend(0,angle,0);   //先旋转到合适方向
					while(!actionServe_enable_)//等待该任务执行完
					{
						if(localization_state_==FIND) return;
					}
				    
				}

				float foward_distance(1.0);
				/*
				switch(i)
				{
					case 0: foward_distance = 2;break;   //所有扫描距离都大于3m
					case 1: foward_distance = 1.5;break;  //所有扫描距离都大于2.5m
					case 2: foward_distance = 1;break;   //所有扫描距离都大于2m
				}
				*/
				std::cout << "next(once): foward " << 1 <<" m ";
				goalSend(0,0,foward_distance);	//前进一米
				while(!actionServe_enable_)//等待该任务执行完
				{
					if(localization_state_==FIND) return;
				}

				moved_flag =true;
				break;
			}

		}
	}
}

void hi_motionActClient::state_callback(const std_msgs::String::ConstPtr& msg)
{
	std::cout << "state_callback:" << std::endl;
	std_msgs::String state_temp;
	std::stringstream ss;
	ss << "FIND " << std::endl;
	state_temp.data = ss.str();
	if(msg->data == state_temp.data)
	{
		localization_state_=FIND;
		std::cout << "localization succeed !" << std::endl;
		if(goalSended_flag_) ac_.cancelGoal();
		ros::shutdown();
	}
	else
	{
		localization_state_=LOST;
	}
}

void hi_motionActClient::explore()
{

}


void sighandler(int signum)
{
	ros::shutdown();
	printf("Catch a signal num is %d ...\n", signum);
	exit(1);
}
void spinThread()
{
  ros::spin();
  signal(SIGINT, sighandler);
}
int main (int argc, char **argv)
{
	ros::init(argc, argv, "test_hi_motionAC");
	hi_motionActClient hi_motionAC("hi_motion");

	boost::thread spin_thread(&spinThread);

	std::cout << "wait for localization state messege ..." << std::endl;
	while(hi_motionAC.localization_state_==hi_motionActClient::UNKNOW);//等待定位状态消息
	
	if(hi_motionAC.localization_state_==hi_motionActClient::FIND) return 0;
	
	for (int i=0;i<10;i++)
	{
		hi_motionAC.selfLocalization();
		if(hi_motionAC.localization_state_==hi_motionActClient::FIND) return 0;
	}
		

	ros::shutdown();
/*
	std::ofstream fout("./pose_laser3.txt");
	for (auto poselaser: hi_motionAC.poseLaserDatas_vec_)
	{
		fout << poselaser.pose_x <<"  "
			 << poselaser.pose_y <<"  "
			 << poselaser.pose_theta <<"  " << (poselaser.pose_theta/M_PI) *180 <<"  "
			 << poselaser.laser_range << std::endl;		
	}
	fout <<"num :  "<<hi_motionAC.poseLaserDatas_vec_.size()<<std::endl;
	fout.close();
*/

	return 0;

}


