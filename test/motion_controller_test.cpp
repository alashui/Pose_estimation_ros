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
#include "std_msgs/String.h"

enum DriveAction 
{
	FORWARD,LEFT,RIGHT,ROTATE
};

enum LocateState 
{
	UNKNOW,FIND,LOST
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
		ros::Time rotate_time;
		double angular_rotated;
		
		LocateState  localization_state;
		  	    
	public:
	    Motion_controller(ros::NodeHandle& nodehandle);
	    void pointCloud_callback(const sensor_msgs::PointCloud2& input);
	    void state_callback(const std_msgs::String::ConstPtr& msg);
	    void drive(const ros::TimerEvent& time);
	  	bool planeExtract(pcl::PointCloud<pcl::PointXYZ>::Ptr);
};

Motion_controller::Motion_controller(ros::NodeHandle& nodehandle):
	node(nodehandle),
	velocity_pub(node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1)),
	
	cloud_receieved_(new pcl::PointCloud<pcl::PointXYZ>),
	is_received(false),
	direction(FORWARD),
	currentMOTION(FORWARD),
	localization_state(UNKNOW)
		
{
	last_time = ros::Time::now();
	ros::MultiThreadedSpinner threads(3);
	ros::Subscriber pointcloud_sub=node.subscribe("/camera/depth_registered/points", 1, 
									&Motion_controller::pointCloud_callback,this);
	ros::Subscriber localizationState_sub=node.subscribe("localization_state", 1, 
									&Motion_controller::state_callback,this);							
	ros::Timer drive=node.createTimer(ros::Duration(0.1), &Motion_controller::drive, this);
	threads.spin(); //blocks until the node is interrupted
}

void Motion_controller::state_callback(const std_msgs::String::ConstPtr& msg)
{
	std::cout << "state_callback:" << std::endl;
	std_msgs::String state_temp;
	std::stringstream ss;
	ss << "FIND " << endl;
	state_temp.data = ss.str();
	if(msg->data == state_temp.data)
	{
		localization_state=FIND;
	}
	else
	{
		localization_state=LOST;
	}
}
void Motion_controller::pointCloud_callback(const sensor_msgs::PointCloud2& input)    
{    
	pcl::fromROSMsg(input, *cloud_receieved_); 		
	is_received = true;
	
	ros::Time current_time = ros::Time::now();		
	float dt = (current_time - last_time).toSec();	
	std::cout << "time_pointCloud_callback:" << dt <<" s"<< std::endl;
	last_time = current_time;
}

void Motion_controller::drive(const ros::TimerEvent& time)
{   
	if(localization_state==UNKNOW)	//等待第一帧图像判断完
		return;
	
	if(localization_state==FIND)	//第一帧图像已经找到则完成初步定位
	{
		return;
	}
	
 
	double DRIVE_LINEARSPEED, DRIVE_ANGULARSPEED;
	bool DRIVE_MOVE, SHOW_VERBOSE;
			
	node.getParamCached("drive_linearspeed", DRIVE_LINEARSPEED);
	node.getParamCached("drive_angularspeed", DRIVE_ANGULARSPEED);
	node.getParamCached("drive_move", DRIVE_MOVE);
	node.getParamCached("show_verbose", SHOW_VERBOSE);	
	
	if(currentMOTION==ROTATE)		//原地旋转模式，直到旋转过360度
	{
		ros::Time rotate_time_now = ros::Time::now();
		float dt = (rotate_time_now - rotate_time).toSec();	
		angular_rotated += DRIVE_ANGULARSPEED * dt;
		
		if(angular_rotated >3.15)
		{
			direction=FORWARD;
			
		}
		else
		{
			direction=ROTATE;
		}
		DriveAction newMotion;
		geometry_msgs::Twist decision;

		//decide what to do, given the advice we've received
		newMotion=direction;

		//make our move
		switch(newMotion)
		{
			case LEFT:
				if(SHOW_VERBOSE) ROS_INFO("PILOT :: Turning %5s", "LEFT");
				decision.angular.z=DRIVE_ANGULARSPEED;
				break;
			case RIGHT:
				if(SHOW_VERBOSE) ROS_INFO("PILOT :: Turning %5s", "RIGHT");
				decision.angular.z=-DRIVE_ANGULARSPEED;
				break;
			case ROTATE:
				if(SHOW_VERBOSE) ROS_INFO("PILOT :: Turning %5s", "ROTATE");
				decision.angular.z=DRIVE_ANGULARSPEED;
				break;
			default:
				decision.linear.x=DRIVE_LINEARSPEED;
		}
		if(DRIVE_MOVE) velocity_pub.publish(decision);
		//tell the obstacle detectors what we've done
		currentMOTION=newMotion;
		
		ros::Time rotate_time = ros::Time::now();
	}  
	else if(is_received)
	{	 //接受到点云数据，下采样，剔除地面，切割，
		//利用剩余点的数量判断有无障碍物，以及障碍物位置
		//如果障碍物太近，则左（右）转避开，太远则继续前进，else （障碍物距离合适）则提取平面
		//如果提取到大的平面则视为墙面，进入原地旋转模式，else 左（右）转避开障碍物
		ros::Time last_time_p = ros::Time::now();
		is_received = false;
			
		double CROP_XRADIUS, CROP_YMIN, CROP_YMAX, CROP_ZMIN, CROP_ZMAX;
		double GROUND_CLOSEY, GROUND_CLOSEZ, GROUND_FARY, GROUND_FARZ;
		double GROUND_TOLERANCEFINE, GROUND_TOLERANCEROUGH,SIZE_DOWNSAMPLING;
		double DISTANCE_MIN,DISTANCE_MAX;
		int NUM_SAMPLES;

		node.getParamCached("size_downsampling", SIZE_DOWNSAMPLING);
		
		//parameters use to crop pointcloud
		node.getParamCached("crop_xradius", CROP_XRADIUS);
		node.getParamCached("crop_ymin", CROP_YMIN);
		node.getParamCached("crop_ymax", CROP_YMAX);
		node.getParamCached("crop_zmin", CROP_ZMIN);
		node.getParamCached("crop_zmax", CROP_ZMAX);
		
		node.getParamCached("num_samples", NUM_SAMPLES);
		
		//two points use to model plane of the ground
		node.getParamCached("ground_closey", GROUND_CLOSEY);
		node.getParamCached("ground_closez", GROUND_CLOSEZ);
		node.getParamCached("ground_fary", GROUND_FARY);
		node.getParamCached("ground_farz", GROUND_FARZ);
		
		node.getParamCached("ground_tolerancefine", GROUND_TOLERANCEFINE);
			
		node.getParamCached("distance_min",DISTANCE_MIN);
		node.getParamCached("distance_max",DISTANCE_MAX);
		
		//model the plane of the ground if the user changed its keypoints
		double GROUND_SLOPE, GROUND_YINTERCEPT; //parameters of the ground
		GROUND_SLOPE=(GROUND_FARY-GROUND_CLOSEY)/(GROUND_FARZ-GROUND_CLOSEZ);
		GROUND_YINTERCEPT=(GROUND_CLOSEY+GROUND_FARY)/2-GROUND_SLOPE*(GROUND_CLOSEZ+GROUND_FARZ)/2;

		
		pcl::VoxelGrid<pcl::PointXYZ> filter_downsample;
		pcl::PassThrough<pcl::PointXYZ> filter_crop;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_front(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacle(new pcl::PointCloud<pcl::PointXYZ>);
		
		int averageObstacles=0; //number of points in our way after averaging our readings
	
		//downsample cloud
		filter_downsample.setInputCloud(cloud_receieved_);
		if(SIZE_DOWNSAMPLING>=0) 
			filter_downsample.setLeafSize(  (float)SIZE_DOWNSAMPLING, 
											(float)SIZE_DOWNSAMPLING,
											(float)SIZE_DOWNSAMPLING   );
		filter_downsample.filter(*cloud_downsampled);
		//cloud_downsampled_pub.publish(*cloud_downsampled);
		cloud_receieved_->clear();

		//remove points near the ground
		for(auto location =cloud_downsampled->begin(); location < cloud_downsampled->end();)
		{
			double distanceFromGroundPlane=fabs(location->y - (GROUND_SLOPE*location->z+GROUND_YINTERCEPT));
				//point's actual y-coordinate - ground's expected y-coordinate
	 			//std::cout << "x: " << location->x << "  "
				  //<< "y: " << location->y << "  "
				  //<< "z: " << location->z <<std::endl; 
		
			if(distanceFromGroundPlane <= GROUND_TOLERANCEFINE) //this point isn't anywhere near the ground
			{ 
				cloud_downsampled->erase(location);	//delete point near the ground
			}
			else
			{ 
				location++;
				//std::cout << "SLOPE: " << GROUND_SLOPE << "  "
						 // << "YINTERCEPT: " << GROUND_YINTERCEPT << "  "
						 // << "distance " << distanceFromGroundPlane <<std::endl;
			}   
		}

		//crop the cloud
		filter_crop.setInputCloud(cloud_downsampled);
		filter_crop.setFilterFieldName("x");
		filter_crop.setFilterLimits(-CROP_XRADIUS, CROP_XRADIUS);
		filter_crop.filter(*cloud_front);

		filter_crop.setInputCloud(cloud_front);
		filter_crop.setFilterFieldName("y");
		filter_crop.setFilterLimits(CROP_YMIN, CROP_YMAX);
		filter_crop.filter(*cloud_front);

		filter_crop.setInputCloud(cloud_front);
		filter_crop.setFilterFieldName("z");
		filter_crop.setFilterLimits(CROP_ZMIN, CROP_ZMAX);
		filter_crop.filter(*cloud_front);

		//cloud_front_pub.publish(*cloud_front);		
		//cloud_obstacle_pub.publish(*cloud_front);
		
		/*
		if(currentMOTION!=FORWARD) 
			heightRangeFrontSamples.clear(); //use straight snapshots while turning
		heightRangeFrontSamples.push_front(cloud_front->size());
		while(heightRangeFrontSamples. size()>(unsigned)NUM_SAMPLES) 
			heightRangeFrontSamples.pop_back(); //constrain our backlog

		//compute average number of points
		for(  std::list<int>::iterator location=heightRangeFrontSamples.begin(); 
							location!=heightRangeFrontSamples.end(); location++  )
			averageObstacles+=*location;
		averageObstacles/=heightRangeFrontSamples.size();
		*/
		
		//let's DRIVE!
		//if(averageObstacles>0) //something is in our way!
		if( cloud_front->size() > 0)
		{
			float centroidX=0;
			float centroidZ=0;

			//compute the centroid of the detected points
			for(auto point=cloud_front->begin(); point < cloud_front->end(); point++ )
			{
				centroidX+=point->x;
				centroidZ+=point->z;
				//std::cout << "x: " << point->x << "  "
						  //<< "y: " << point->y << "  "
						  //<< "z: " << point->z <<std::endl; 
						  
			}
			centroidX/=cloud_front->size();
			centroidZ/=cloud_front->size();
			
			if(centroidZ < DISTANCE_MIN) //too close to the obstacle
			{
				if(centroidX<0) //obstacle(s)' centroid is off to left
					direction=RIGHT;
				else //centroidX>=0
					direction=LEFT;
			}
			else if (centroidZ > DISTANCE_MAX) //too far from the obstacle
			{
				direction=FORWARD;
			}
			else	//within the appropriate range
			{
				if( planeExtract(cloud_downsampled) ) //如果提取到大的平面则视为墙面，进入原地旋转模式
					direction=ROTATE;
				else
				{
					if(centroidX<0) //obstacle(s)' centroid is off to left
					    direction=RIGHT;
					else //centroidX>=0
					    direction=LEFT;
				}
			}
			
			if(SHOW_VERBOSE)
				ROS_INFO("HEIGHT RANGE :: Seeing %4d points in our way\n -> Centroid is at %.3f , %.3f  ", averageObstacles, centroidX,centroidZ);


		}
		else //nothing to see here
			direction=FORWARD;


		//variable declarations
		DriveAction newMotion;
		geometry_msgs::Twist decision;

		//decide what to do, given the advice we've received
		newMotion=direction;

		//don't reverse the direction of a turn
		if( newMotion!=ROTATE &&
			newMotion!=FORWARD && 
			currentMOTION!=FORWARD && 
			newMotion!=currentMOTION)
		{
			if(SHOW_VERBOSE) ROS_INFO("PILOT :: Overrode recommended oscillation");

			newMotion=currentMOTION; //keep rotating in the same direction we were
		}

		//make our move
		switch(newMotion)
		{
			case LEFT:
				if(SHOW_VERBOSE) ROS_INFO("PILOT :: Turning %5s", "LEFT");
				decision.angular.z=DRIVE_ANGULARSPEED;
				break;
			case RIGHT:
				if(SHOW_VERBOSE) ROS_INFO("PILOT :: Turning %5s", "RIGHT");
				decision.angular.z=-DRIVE_ANGULARSPEED;
				break;
			case ROTATE:
				if(SHOW_VERBOSE) ROS_INFO("PILOT :: Turning %5s", "ROTATE");
				decision.angular.z=DRIVE_ANGULARSPEED;
				rotate_time = ros::Time::now();
				break;
			default:
				decision.linear.x=DRIVE_LINEARSPEED;
		}
		if(DRIVE_MOVE) velocity_pub.publish(decision);
		//tell the obstacle detectors what we've done
		currentMOTION=newMotion;
		
		ros::Time current_time_p = ros::Time::now();
		float dt = (current_time_p - last_time_p).toSec();	
		std::cout << "time_pilot:" << dt <<" s"<< std::endl;
		
	}
                 
} 
   
bool Motion_controller::planeExtract(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_ptr)
{
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudout(new pcl::PointCloud<pcl::PointXYZ>);
	double plane_model_a,plane_model_b,plane_model_c,plane_model_d;
	
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);   
	seg.setMethodType(pcl::SAC_RANSAC);     
	seg.setDistanceThreshold(0.01);    
	seg.setInputCloud(point_cloud_ptr);
	seg.segment(*inliers, *coefficients);

	//pcl::copyPointCloud(*point_cloud_ptr, *inliers, *cloudout);
	if (inliers->indices.size() == 0)
	{
		//PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return false;
	}
	//parameters of plane 
	plane_model_a = coefficients->values[0];
	plane_model_b = coefficients->values[1];
	plane_model_c = coefficients->values[2];
	plane_model_d = coefficients->values[3];
	
	std::cout << "  xa=" << coefficients->values[0]
			  << "  yb=" << coefficients->values[1]
			  << "  zc=" << coefficients->values[2]
			  << "  d =" << coefficients->values[3] << endl;
	//std::cout <<"totoal number:"<< cloudout->points.size()<< endl;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "obstacle_detector"); //string here is the node name
	ros::NodeHandle node("namespace_ObDetector"); //string here is the namespace for parameters

	//initial parameter values
	node.setParam("crop_xradius", 0.2); //should be slightly greater than robot's radius
	node.setParam("crop_ymin", -0.07); //should be slightly above robot's height
	node.setParam("crop_ymax", 0.35); //should be slightly above the ground's highest point
	node.setParam("crop_zmin", 0.0); //greater than zero excludes points close to robot
	node.setParam("crop_zmax", 2.5); //farthest to search for obstacles: lower for tighter maneuvering, higher for greater safety
	node.setParam("size_downsampling", 0.04); //less is more: should be low enough to eliminate noise from the region of interest (negative for [really bad] default)
	node.setParam("num_samples", 1); //number of samples to average: should be low enough to prevent false positives
	node.setParam("ground_bumperfrontal", 0.1); //extra uncropped space on the front and back edges of the plane whose edges, borders, and presence are disregarded; note that for the front edge only, this is used with ground_closez to tolerate the gap between the robot and plane
	node.setParam("ground_bumperlateral", 0.02); //extra uncropped space on the left and right edges of the plane whose edges, borders, and presence are disregarded
	node.setParam("ground_closey", 0.2855); //y-coordinate of the closest point on the ground
	node.setParam("ground_closez", 0.677); //corresponding z-coordinate for bumper border and modeling the plane
	node.setParam("ground_fary", 0.25); //y-coordinate of a far point on the ground
	node.setParam("ground_farz", 2.4); //corresponding z-coordinate for modeling the plane
	node.setParam("ground_tolerancefine", 0.05); //maximum y-coordinate deviation of points that are still considered part of the ground itself
	node.setParam("ground_tolerancerough", 0.05); //maximum y-coordinate deviation of points that are evaluated at all
	node.setParam("ground_normalsmoothing", -1.0); //smoothing size for normal estimation (negative for default)
	node.setParam("ground_thresholdlower", 1.0); //for curvature-based edge detection: cutoff for consideration as possible edges (negative for default)
	node.setParam("ground_thresholdhigher", 1.7); //for curvature-based edge detection: cutoff for definite classification as edges (negative for default)
	node.setParam("ground_outlierradius", 0.05); //radius used for neighbor search to filter out outliers (negative to disable outlier removal)
	node.setParam("ground_normalestimation", -1); //normal estimation method: as defined in IntegralImageNormalEstimation (negative for default)
	node.setParam("ground_outlierneighbors", 6); //minimum neighbors to be spared by outlier persecution (negative for default)

	node.setParam("drive_linearspeed", 0.2);
	node.setParam("drive_angularspeed", 0.2);
	node.setParam("drive_move", false);
	node.setParam("show_verbose", true);

	node.setParam("distance_min",1.5);
	node.setParam("distance_max",2.5);
	
	Motion_controller explorer(node); //block to do obstacle avoidance


	//clean up parameters, plus a Vim macro to generate them from "default parameter values"
	node.deleteParam("crop_xradius");
	node.deleteParam("crop_ymin");
	node.deleteParam("crop_ymax");
	node.deleteParam("crop_zmin");
	node.deleteParam("crop_zmax");
	node.deleteParam("size_downsampling");
	node.deleteParam("num_samples");
	node.deleteParam("ground_bumperfrontal");
	node.deleteParam("ground_bumperlateral");
	node.deleteParam("ground_closey");
	node.deleteParam("ground_closez");
	node.deleteParam("ground_fary");
	node.deleteParam("ground_farz");
	node.deleteParam("ground_tolerancefine");
	node.deleteParam("ground_tolerancerough");
	node.deleteParam("ground_normalsmoothing");
	node.deleteParam("ground_thresholdlower");
	node.deleteParam("ground_thresholdhigher");
	node.deleteParam("ground_outlierradius");
	node.deleteParam("ground_normalestimation");
	node.deleteParam("ground_outlierneighbors");
	node.deleteParam("drive_linearspeed");
	node.deleteParam("drive_angularspeed");
	node.deleteParam("drive_move");
	node.deleteParam("show_verbose");
	
	node.deleteParam("distance_min");
	node.deleteParam("distance_max");
}

