#include "localization/motion_controller.h"

namespace localization
{

Motion_controller::Motion_controller(ros::NodeHandle& nodehandle):
	nh(nodehandle),
	velocity_pub(nh.advertise<geometry_msgs::Twist>("/cmd_vel",1)),
	CROP_XRADIUS(0.2),CROP_YMIN(-0.07),CROP_YMAX(0.35),CROP_ZMIN(0.0),CROP_ZMAX(1.5)	
{
	pointcloud_sub=nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, 
														&Motion_controller::pcloud_callback,this);
}

void Motion_controller::pcloud_callback(const sensor_msgs::PointCloud2& input)    
{    
    pcl::PointCloud<pcl::PointXYZ> cloud;        
    pcl::fromROSMsg(input, cloud); // sensor_msgs::PointCloud2 ----> pcl::PointCloud<T>
    heightRange(cloud.makeShared());
}

void Motion_controller::heightRange(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{      
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_croped;
    //downsample    
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
    statFilter.setInputCloud(cloud);
    statFilter.setMeanK(10);
    statFilter.setStddevMulThresh(0.2);
    statFilter.filter(*cloud_filtered);
    
    pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
    voxelSampler.setInputCloud(cloud_filtered);
    voxelSampler.setLeafSize(0.04f, 0.04f, 0.04f);
    voxelSampler.filter(*cloud_downsampled);  
    
      
    //crop
    pcl::PassThrough<pcl::PointXYZ> crop;
    
    crop.setInputCloud(cloud_downsampled);
    crop.setFilterFieldName("x");
    crop.setFilterLimits(-CROP_XRADIUS,CROP_XRADIUS);
    crop.filter(*cloud_croped);
    
    crop.setInputCloud(cloud_croped);
    crop.setFilterFieldName("y");
    crop.setFilterLimits(CROP_YMIN,CROP_YMAX);
    crop.filter(*cloud_croped);
    
    crop.setInputCloud(cloud_croped);
    crop.setFilterFieldName("z")
    crop.setFilterLimits(CROP_ZMIN,CROP_ZMAX);
    crop.filter(*cloud_croped);
        

    if(currentMOTION!=FORWARD) heightRangeFrontSamples.clear(); //use straight snapshots while turning
	heightRangeFrontSamples.push_front(front->size());
	while(heightRangeFrontSamples.size()>(unsigned)HEIGHT_SAMPLES) heightRangeFrontSamples.pop_back(); //constrain our backlog

	//compute average number of points
	for(std::list<int>::iterator location=heightRangeFrontSamples.begin(); location!=heightRangeFrontSamples.end(); location++)
		averageObstacles+=*location;
	averageObstacles/=heightRangeFrontSamples.size();

	// DRIVE!
	if(averageObstacles>0) //something is in our way!
	{
		float centroidX=0;

		//compute the centroid of the detected points
		for(pcl::PointCloud<pcl::PointXYZ>::iterator point=front->begin(); point<front->end(); point++)
			centroidX+=point->x;
		centroidX/=front->size();

		if(HEIGHT_VERBOSE)
			ROS_INFO("HEIGHT RANGE :: Seeing %4d points in our way\n -> Centroid is at %.3f i", averageObstacles, centroidX);

		if(centroidX<0) //obstacle(s)'[s] centroid is off to left
			directionsPrimary=RIGHT;
		else //centroidX>=0
			directionsPrimary=LEFT;
	}
	else //nothing to see here
		directionsPrimary=FORWARD;

	//send our imagery to any connected visualizer
	//panorama.publish(*downsampled);
	//height.publish(*front);
                 
}    


}

