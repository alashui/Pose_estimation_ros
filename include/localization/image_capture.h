#ifndef ImageCapture_H
#define ImageCapture_H

#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

namespace localization 
{
#define Pi 3.1415926 
using namespace sensor_msgs;
using namespace message_filters;

class Capturer
{
	private:
	  ros::NodeHandle nh;
	  
	  ros::Subscriber odom_sub;
	  float odom_vx, odom_vy, odom_vth;
	  float x, y , th, dd;
	  ros::Time last_time;
	  
	  image_transport::ImageTransport it;
	  typedef image_transport::SubscriberFilter ImageSubscriber;
	  ImageSubscriber rgb_sub;
	  ImageSubscriber depth_sub;

	  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
	  Synchronizer< MySyncPolicy > sync;
	  
	  
	public:
	  std::string save_dir_;
	  
	  int saveCount_;
	  bool state_;   //是否采集到图像
	  bool Enable_;	 //是否要采集
	  cv::Mat color_, depth_;

	  Capturer(std::string save_dir);
	  void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
	  bool captrue_check();
      void callback(const ImageConstPtr& rgb_image, const ImageConstPtr& depth_image);	    
      void processImage(const ImageConstPtr& rgb_image, const ImageConstPtr& depth_image);	    
	  void saveImage(cv_bridge::CvImageConstPtr cv_ptr, std::string imageType);
		
};

}
#endif // PoseEstimation_H

