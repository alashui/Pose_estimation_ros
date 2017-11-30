#ifndef MotionController_H
#define MotionController_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Twist.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/f ilters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

namespace localization 
{

enum DriveAction 
{
	FORWARD,LEFT,RIGHT
};

class Motion_controller
{
	private:
	    ros::NodeHandle nh;
	    ros::subscriber pointcloud_sub; 
	    ros::advertise  velocity_pub;
	  
	

	    double CROP_XRADIUS,CROP_YMIN,CROP_YMAX,CROP_ZMIN,CROP_ZMAX;
	    DriveAction directionsPrimary;
	    DriveAction directionsSecondary;
	    std::list<int> heightRangeFrontSamples;
	    
	    
	    public:
	    Motion_controller(ros::NodeHandle& nodehandle);
	    void pcloud_callback(const sensor_msgs::PointCloud2& input);
	    void heightRange(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
	  
};

}
#endif 
