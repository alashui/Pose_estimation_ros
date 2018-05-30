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

#include <tf/transform_listener.h>

namespace localization 
{
#define Pi 3.1415926 
using namespace sensor_msgs;
using namespace message_filters;

class Capturer
{
	private:
		ros::NodeHandle nh;

		image_transport::ImageTransport it;
		typedef image_transport::SubscriberFilter ImageSubscriber;
		ImageSubscriber rgb_sub;
		ImageSubscriber depth_sub;

		typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
		Synchronizer< MySyncPolicy > sync;

		float pose_x_, pose_y_, pose_theta_;//odom坐标系下位姿
		float pose_x_map_, pose_y_map_, pose_theta_map_;//地图坐标系下位姿	  
		tf::TransformListener listener_;
		tf::StampedTransform transform_;

		std::string map_frame_;
		std::string odom_frame_;
		std::string base_frame_;
		
		const float angle_MIN_INC=M_PI * 5.0/180; //最小采集间隔角度
		const float dist_MIN_INC=0.1;	//最小采集间隔距离
	  
	  	std::ofstream fout_;//记录每张图像的pose信息结果
	public:
		std::string save_dir_;

		int saveCount_;
		bool state_;   //是否采集到图像
		bool Enable_;	 //是否要采集
		cv::Mat color_, depth_;

		Capturer(std::string save_dir);
		~Capturer();

		bool captrue_check();
		void callback(const ImageConstPtr& rgb_image, const ImageConstPtr& depth_image);	    
		void processImage(const ImageConstPtr& rgb_image, const ImageConstPtr& depth_image);	    
		void saveImage(cv_bridge::CvImageConstPtr cv_ptr, std::string imageType);
		
};

}
#endif // PoseEstimation_H

