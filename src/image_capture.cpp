#include "localization/image_capture.h"

namespace localization
{

Capturer::Capturer(std::string save_dir): saveCount_(0),it(nh),state_(false),
										  save_dir_(save_dir),Enable_(true),
										  odom_vx(0), odom_vy(0), odom_vth(0);
	  									  x(0), y(0) , th(0) , dd(0);	
										  rgb_sub( it, "/camera/rgb/image_raw", 1 ),
										  depth_sub( it, "/camera/depth/image_raw", 1 ),
										  sync( MySyncPolicy( 10 ), rgb_sub, depth_sub )
{
	last_time = ros::Time::now();
	sync.registerCallback( boost::bind( &Capturer::callback, this, _1, _2 ) );
	odom_sub = nh.subscribe<nav_msgs::Odometry>("/odom",10,odom_callback);
}

void Capturer::odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
	odom_vx_ =odom->twist.twist.linear.x;
	odom_vy_ =odom->twist.twist.linear.y;
	odom_vth_ =odom->twist.twist.angular.z;	
}

bool Capturer::captrue_check() //根据里程信息决定是否可以采集图像
{
		ros::Time current_time = ros::Time::now();

		//compute odometry in a typical way given the velocities of the robot
		float dt = (current_time - last_time).toSec();
		float delta_x = (odom_vx * cos(odom_th) - odom_vy * sin(odom_th)) * dt;
		float delta_y = (odom_vx * sin(odom_th) + odom_vy * cos(odom_th)) * dt;
		float delta_th = vth * dt;
		
		last_time = current_time;
		 
		x +=abs( delta_x);
		y +=abs( delta_y);
		th +=abs(delta_th);
		
		dd=sqrt(x*x + y*y);
		//cout <<"dd: "<<dd<<"   th: "<<th<<endl;
		if (dd >= 0.10 || th >= (float)(5.0/180.0) * Pi)  //运动距离超过0.1m或者角度超过5度
		{									   //则可捕获一张图像			
			x = 0;
			y = 0;
			th = 0;
			dd = 0;
			return true;
		}
		else
			return false;
}


void Capturer::callback(const ImageConstPtr& rgb_image, const ImageConstPtr& depth_image)
{
	if(Enable_)
	{
		if(saveCount_ == 0)
		    saveCount_ ++;
	
		if( captrue_check() )
		{
		    processImage(rgb_image, depth_image);
		    state_ = true;
		}			
	}
	else
		last_time = ros::Time::now();
	
}


void Capturer::processImage(const ImageConstPtr& rgb_image, const ImageConstPtr& depth_image)
{
	ROS_INFO("Saving images");
	// Process rgb image
	cv_bridge::CvImageConstPtr cv_ptr;
  	try
  	{
		cv_ptr = cv_bridge::toCvCopy(rgb_image, sensor_msgs::image_encodings::BGR8);				
		std::cout << "RGB " << cv_ptr->header.seq << std::endl;
		saveImage(cv_ptr, "rgb");
		
		color_ = cv_ptr->image;
	  //imshow("rgb", cv_ptr->image);
		//waitKey(10);
  	}		
  	catch (cv_bridge::Exception& e)
  	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
  	}

  	// Process depth image
  	try
  	{
   		cv_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
   		std::cout << "depth " << cv_ptr->header.seq << std::endl;
   		saveImage(cv_ptr, "depth");
   		
   		depth_ = cv_ptr->image;
   		//imshow("depth", cv_ptr->image);
   		//waitKey(10);
  	}
 	catch (cv_bridge::Exception& e)
 	{
   		ROS_ERROR("cv_bridge exception: %s", e.what());
   		return;
 	}		 				
	saveCount_ ++;
}

void Capturer::saveImage(cv_bridge::CvImageConstPtr cv_ptr, std::string imageType)
{
  	std::ostringstream s;
  	s << save_dir_ << "/" << imageType << "/" << imageType <<saveCount_ << ".png";
  	imwrite(s.str(), cv_ptr->image);
}


}
