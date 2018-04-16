#ifndef MotionController_H
#define MotionController_H

#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl/common/common_headers.h>
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/radius_outlier_removal.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>   
#include <pcl/sample_consensus/model_types.h>   
#include <pcl/segmentation/sac_segmentation.h>   
#include <pcl/visualization/pcl_visualizer.h>

#include "geometry_msgs/Twist.h"

namespace localization 
{

enum DriveAction 
{
	FORWARD,LEFT,RIGHT,ROTATE
};

class Motion_controller
{
	private:
		ros::NodeHandle node;
		ros::Publisher velocity_pub;	 
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_receieved_;
		bool is_received; 
		
		DriveAction direction;
		DriveAction currentMOTION;
		ros::Time last_time;
		
		  	    
	public:
	    Motion_controller(ros::NodeHandle& nodehandle);
	    void pointCloud_callback(const sensor_msgs::PointCloud2& input);
	    void drive(const ros::TimerEvent& time);
	  	void computeRotationAngle(pcl::PointCloud<pcl::PointXYZ>::Ptr);
};

}
#endif 
