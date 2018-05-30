#include "localization/image_capture.h"

namespace localization
{

Capturer::Capturer(std::string save_dir): saveCount_(0),it(nh),state_(false),
										  save_dir_(save_dir),Enable_(true),
										 // odom_vx(0), odom_vy(0), odom_vth(0),
	  									 //x(0), y(0) , th(0) , dd(0),	
										  rgb_sub( it, "/camera/rgb/image_raw", 1 ),
										  depth_sub( it, "/camera/depth/image_raw", 1 ),
										  sync( MySyncPolicy( 10 ), rgb_sub, depth_sub ),
										  map_frame_("/map"),
										  odom_frame_("/odom"),
										  fout_(save_dir+"/image_pose_map.txt") 
{

	try
	{
		listener_.waitForTransform(map_frame_, "/base_footprint", ros::Time(), ros::Duration(2.0) );
		base_frame_ = "/base_footprint";
		ROS_INFO("base_frame = /base_footprint");
	}
	catch (tf::TransformException & ex)
	{
		try
		{
			listener_.waitForTransform(map_frame_, "/base_link", ros::Time(), ros::Duration(2.0) );
			base_frame_ = "/base_link";
			ROS_INFO("base_frame = /base_link");
		}
		catch (tf::TransformException ex)
		{
			ROS_INFO("Cannot find transform between /map and /base_link or /base_footprint");
		}
	}

	sync.registerCallback( boost::bind( &Capturer::callback, this, _1, _2 ) );
}

Capturer::~Capturer()
{
	fout_.close();
}

bool Capturer::captrue_check() //根据里程信息决定是否可以采集图像
{

	listener_.lookupTransform(odom_frame_, base_frame_, ros::Time(0), transform_);
	float pose_x_now = transform_.getOrigin().x();
	float pose_y_now = transform_.getOrigin().y();
	float pose_theta_now = tf::getYaw(transform_.getRotation());
	
	listener_.lookupTransform(map_frame_, base_frame_, ros::Time(0), transform_);
	pose_x_map_ = transform_.getOrigin().x();
	pose_y_map_ = transform_.getOrigin().y();
	pose_theta_map_ = tf::getYaw(transform_.getRotation());
	//运动过一定距离才采集一次
	bool flag_temp(false);
	if(	(fabs ( pose_theta_now - pose_theta_ ) >= angle_MIN_INC ) ||
		( sqrt( pow((pose_x_now - pose_x_),2) +  
   				pow((pose_y_now - pose_y_),2)   )  >= dist_MIN_INC ) )
   	{
		flag_temp = true;
		pose_x_ =pose_x_now ;
		pose_y_ = pose_y_now ;
		pose_theta_ = pose_theta_now;
	}

	return flag_temp;
	
}

void Capturer::callback(const ImageConstPtr& rgb_image, const ImageConstPtr& depth_image)
{

	if(Enable_)
	{
		if(saveCount_ == 0)
		{
		    saveCount_ ++;
		    
		    //记录位姿
		    listener_.lookupTransform(odom_frame_, base_frame_, ros::Time(0), transform_);
			pose_x_ = transform_.getOrigin().x();
			pose_y_ = transform_.getOrigin().y();
			pose_theta_ = tf::getYaw(transform_.getRotation());
			
			listener_.lookupTransform(map_frame_, base_frame_, ros::Time(0), transform_);
			pose_x_map_ = transform_.getOrigin().x();
			pose_y_map_ = transform_.getOrigin().y();
			pose_theta_map_ = tf::getYaw(transform_.getRotation());

		    processImage(rgb_image, depth_image);
		    state_ = true;
		}
		
		
		else if( captrue_check() )
		{
		    processImage(rgb_image, depth_image);
		    state_ = true;
		}			
	}	
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
 	
 	fout_ << saveCount_ <<" "<< pose_x_map_ <<" "<<pose_y_map_ <<" "<<pose_theta_map_<<std::endl;//保存每张图像的pose信息
 			 				
	saveCount_ ++;
	
}

void Capturer::saveImage(cv_bridge::CvImageConstPtr cv_ptr, std::string imageType)
{
  	std::ostringstream s;
  	s << save_dir_ << "/" << imageType << "/" << imageType <<saveCount_ << ".png";
  	imwrite(s.str(), cv_ptr->image);
}


}
