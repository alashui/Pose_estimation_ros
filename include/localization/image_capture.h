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

using namespace sensor_msgs;
using namespace message_filters;



float vx = 0.0;
float vy = 0.0;
float vth = 0.0;


void odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
	vx =odom->twist.twist.linear.x;
	vy =odom->twist.twist.linear.y;
	vth =odom->twist.twist.angular.z;	
}

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
	  
	  
	public:
	  std::string save_dir_;
	  int saveCount_;
	  bool state_;
	  bool Enable_;
	  cv::Mat color_, depth_;

		Capturer(std::string save_dir): saveCount_(0),it(nh),state_(false),Enable_(false),save_dir_(save_dir),	
					rgb_sub( it, "/camera/rgb/image_raw", 1 ),
					depth_sub( it, "/camera/depth/image_raw", 1 ),
					sync( MySyncPolicy( 10 ), rgb_sub, depth_sub )
	    {
			sync.registerCallback( boost::bind( &Capturer::callback, this, _1, _2 ) );
	    }

	    void callback(const ImageConstPtr& rgb_image, const ImageConstPtr& depth_image)
	    {
			if(saveCount_ == 0)
			{
			  saveCount_ ++;
			}
			
			if(Enable_)
			{
			  processImage(rgb_image, depth_image);
			  Enable_=false;
			  state_ = true;
			}	
	    }


	    void processImage(const ImageConstPtr& rgb_image, const ImageConstPtr& depth_image)
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

		void saveImage(cv_bridge::CvImageConstPtr cv_ptr, std::string imageType)
		{
			// Save rgb image to ~/follower_image/rgb
			// Save depth image to ~/follower_image/depth
		  	std::ostringstream s;
		  	s << save_dir_ << "/" << imageType << "/" << imageType <<saveCount_ << ".png";
		  	imwrite(s.str(), cv_ptr->image);
		}

};
