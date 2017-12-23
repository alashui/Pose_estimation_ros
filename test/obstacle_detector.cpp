#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
//#include "pcl/features/organized_edge_detection.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "geometry_msgs/Twist.h"

/**
Represents a request for a particular drive action, which may be to go straight, turn left, or turn right
*/
enum DriveAction
{
	FORWARD, LEFT, RIGHT
};

/**
Performs obstacle detection and avoidance using two algorithms simultaneously
*/
class ObstacleAvoidance
{
  private:
    ros::Time last_time;	
	ros::NodeHandle node;
	ros::Publisher velocity;
	ros::Publisher cloud_downsampled_pub; //downsampled cloud
	ros::Publisher cloud_obstacle_pub; //obstacle
	ros::Publisher cloud_front_pub; //heightRange's region of interest
	DriveAction currentMOTION; //pilot's account of what was last done: detection algorithms should not modify!
	DriveAction directionsPrimary; //the height range algorithm's suggestion
	std::list<int> heightRangeFrontSamples;
	double last_GROUND_CLOSEY, last_GROUND_CLOSEZ, last_GROUND_FARY, last_GROUND_FARZ; //only recalculate the below when necessary
	double GROUND_SLOPE, GROUND_YINTERCEPT; //model the ground's location
	DriveAction groundLastForcedTurn; //which way we would have turned: should never be set to FORWARD

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_receieved_;
	bool is_received;
  public:

	ObstacleAvoidance(ros::NodeHandle& handle):
		node(handle), 
		velocity(node.advertise<geometry_msgs::Twist>
						("/mobile_base/commands/velocity", 1)), 
		cloud_downsampled_pub(node.advertise<pcl::PointCloud<pcl::PointXYZ> >("cloud_downsampled", 1)), 
		cloud_front_pub(node.advertise<pcl::PointCloud<pcl::PointXYZ> >("cloud_front", 1)), 
		cloud_obstacle_pub(node.advertise<pcl::PointCloud<pcl::PointXYZ> >("obstacle", 1)), 
		//occlusions(node.advertise<pcl::PointCloud<pcl::PointXYZ> >("occlusions", 1)), 
		currentMOTION(FORWARD), directionsPrimary(FORWARD), 
	    last_GROUND_CLOSEY(0),last_GROUND_CLOSEZ(0),  
		last_GROUND_FARY(0), last_GROUND_FARZ(0), 
		groundLastForcedTurn(LEFT),
		cloud_receieved_(new pcl::PointCloud<pcl::PointXYZ>),
		is_received(false)
	{
		last_time = ros::Time::now();	
		ros::MultiThreadedSpinner threads(2);
		ros::Subscriber heightRange=node.subscribe("/camera/depth_registered/points", 1, 	
													&ObstacleAvoidance::pointCloud_callback, this);
		//ros::Subscriber groundEdges=node.subscribe("/cloud_throttled", 1, &ObstacleAvoidance::groundEdges, this);
		ros::Timer pilot=node.createTimer(ros::Duration(0.1), &ObstacleAvoidance::pilot, this);

		threads.spin(); //blocks until the node is interrupted
	}

	void pointCloud_callback(const sensor_msgs::PointCloud2& input)
	{	
		ros::Time current_time = ros::Time::now();
		
		float dt = (current_time - last_time).toSec();	
		std::cout << "time_pointCloud_callback:" << dt <<" s"<< std::endl;
		       				
		pcl::fromROSMsg(input, *cloud_receieved_); 		
		is_received = true;
		last_time = current_time;
	}

	void pilot(const ros::TimerEvent& time)
	{
		if(is_received)
		{	
			ros::Time last_time_p = ros::Time::now();
			is_received = false;
				
			double CROP_XRADIUS, CROP_YMIN, CROP_YMAX, CROP_ZMIN, CROP_ZMAX;
			double GROUND_CLOSEY, GROUND_CLOSEZ, GROUND_FARY, GROUND_FARZ;
			double GROUND_TOLERANCEFINE, GROUND_TOLERANCEROUGH,HEIGHT_DOWNSAMPLING;;
			int HEIGHT_SAMPLES;
			bool HEIGHT_VERBOSE;
		
			double DRIVE_LINEARSPEED, DRIVE_ANGULARSPEED;
			bool DRIVE_MOVE, DRIVE_VERBOSE;		

			node.getParamCached("height_downsampling", HEIGHT_DOWNSAMPLING);
			node.getParamCached("crop_xradius", CROP_XRADIUS);
			node.getParamCached("crop_ymin", CROP_YMIN);
			node.getParamCached("crop_ymax", CROP_YMAX);
			node.getParamCached("crop_zmin", CROP_ZMIN);
			node.getParamCached("crop_zmax", CROP_ZMAX);
			node.getParamCached("height_samples", HEIGHT_SAMPLES);
			node.getParamCached("height_verbose", HEIGHT_VERBOSE);
			node.getParamCached("ground_closey", GROUND_CLOSEY);
			node.getParamCached("ground_closez", GROUND_CLOSEZ);
			node.getParamCached("ground_fary", GROUND_FARY);
			node.getParamCached("ground_farz", GROUND_FARZ);
			node.getParamCached("ground_tolerancefine", GROUND_TOLERANCEFINE);
		
			node.getParamCached("drive_linearspeed", DRIVE_LINEARSPEED);
			node.getParamCached("drive_angularspeed", DRIVE_ANGULARSPEED);
			node.getParamCached("drive_move", DRIVE_MOVE);
			node.getParamCached("drive_verbose", DRIVE_VERBOSE);	
			
			//model the plane of the ground iff the user changed its keypoints
			if( GROUND_CLOSEY!=last_GROUND_CLOSEY || 
				GROUND_CLOSEZ!=last_GROUND_CLOSEZ || 
				GROUND_FARY!=last_GROUND_FARY || 
				GROUND_FARZ!=last_GROUND_FARZ    )
			{
				GROUND_SLOPE=(GROUND_FARY-GROUND_CLOSEY)/(GROUND_FARZ-GROUND_CLOSEZ);
				GROUND_YINTERCEPT=(GROUND_CLOSEY+GROUND_FARY)/2-GROUND_SLOPE*(GROUND_CLOSEZ+GROUND_FARZ)/2;
				last_GROUND_CLOSEY=GROUND_CLOSEY;
				last_GROUND_FARY=GROUND_FARY;
				last_GROUND_CLOSEZ=GROUND_CLOSEZ;
				last_GROUND_FARZ=GROUND_FARZ;
			}
			
			pcl::VoxelGrid<pcl::PointXYZ> filter_downsample;
			pcl::PassThrough<pcl::PointXYZ> filter_crop;
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_front(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_obstacle(new pcl::PointCloud<pcl::PointXYZ>);
			
			int averageObstacles=0; //number of points in our way after averaging our readings
		
			//downsample cloud
			filter_downsample.setInputCloud(cloud_receieved_);
			if(HEIGHT_DOWNSAMPLING>=0) 
				filter_downsample.setLeafSize(  (float)HEIGHT_DOWNSAMPLING, 
												(float)HEIGHT_DOWNSAMPLING,
												(float)HEIGHT_DOWNSAMPLING   );
			filter_downsample.filter(*cloud_downsampled);
			cloud_downsampled_pub.publish(*cloud_downsampled);
			cloud_receieved_->clear();

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

			cloud_front_pub.publish(*cloud_front);
			
			
			//remove points near the ground
			for(auto location =cloud_front->begin(); location < cloud_front->end();)
			{
				double distanceFromGroundPlane=fabs(location->y - (GROUND_SLOPE*location->z+GROUND_YINTERCEPT));
					//point's actual y-coordinate - ground's expected y-coordinate
		 			//std::cout << "x: " << location->x << "  "
					  //<< "y: " << location->y << "  "
					  //<< "z: " << location->z <<std::endl; 
			
				if(distanceFromGroundPlane <= GROUND_TOLERANCEFINE) //this point isn't anywhere near the ground
				{ 
					cloud_front->erase(location);	//delete point near the ground
				}
				else
				{ 
					location++;
					//std::cout << "SLOPE: " << GROUND_SLOPE << "  "
							 // << "YINTERCEPT: " << GROUND_YINTERCEPT << "  "
							 // << "distance " << distanceFromGroundPlane <<std::endl;
				}   
			}
			
			cloud_obstacle_pub.publish(*cloud_front);
		
			if(currentMOTION!=FORWARD) 
				heightRangeFrontSamples.clear(); //use straight snapshots while turning
			heightRangeFrontSamples.push_front(cloud_front->size());
			while(heightRangeFrontSamples. size()>(unsigned)HEIGHT_SAMPLES) 
				heightRangeFrontSamples.pop_back(); //constrain our backlog

			//compute average number of points
			for(  std::list<int>::iterator location=heightRangeFrontSamples.begin(); 
								location!=heightRangeFrontSamples.end(); location++  )
				averageObstacles+=*location;
			averageObstacles/=heightRangeFrontSamples.size();

			//let's DRIVE!
			if(averageObstacles>0) //something is in our way!
			{
				float centroidX=0;

				//compute the centroid of the detected points
				for(auto point=cloud_front->begin(); point < cloud_front->end(); point++ )
				{
					centroidX+=point->x;
					//std::cout << "x: " << point->x << "  "
							  //<< "y: " << point->y << "  "
							  //<< "z: " << point->z <<std::endl; 
							  
				}
				centroidX/=cloud_front->size();

				//if(HEIGHT_VERBOSE)
					ROS_INFO("HEIGHT RANGE :: Seeing %4d points in our way\n -> Centroid is at %.3f ", averageObstacles, centroidX);

				if(centroidX<0) //obstacle(s)'[s] centroid is off to left
					directionsPrimary=RIGHT;
				else //centroidX>=0
					directionsPrimary=LEFT;
			}
			else //nothing to see here
				directionsPrimary=FORWARD;


			//variable declarations
			DriveAction newMotion;
			geometry_msgs::Twist decision;

			//decide what to do, given the advice we've received
			newMotion=directionsPrimary;

			//don't reverse the direction of a turn
			if(newMotion!=FORWARD && currentMOTION!=FORWARD && newMotion!=currentMOTION)
			{
				if(DRIVE_VERBOSE) ROS_INFO("PILOT :: Overrode recommended oscillation");

				newMotion=currentMOTION; //keep rotating in the same direction we were
			}

			//make our move
			switch(newMotion)
			{
				case LEFT:
					if(DRIVE_VERBOSE) ROS_INFO("PILOT :: Turning %5s", "LEFT");
					decision.angular.z=DRIVE_ANGULARSPEED;
					break;
				case RIGHT:
					if(DRIVE_VERBOSE) ROS_INFO("PILOT :: Turning %5s", "RIGHT");
					decision.angular.z=-DRIVE_ANGULARSPEED;
					break;
				default:
					decision.linear.x=DRIVE_LINEARSPEED;
			}
			if(DRIVE_MOVE) velocity.publish(decision);
			//tell the obstacle detectors what we've done
			currentMOTION=newMotion;
			
			ros::Time current_time_p = ros::Time::now();
			float dt = (current_time_p - last_time_p).toSec();	
			std::cout << "time_pilot:" << dt <<" s"<< std::endl;
			
		}
	}
};


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
	node.setParam("height_downsampling", 0.04); //less is more: should be low enough to eliminate noise from the region of interest (negative for [really bad] default)
	node.setParam("height_samples", 1); //number of samples to average: should be low enough to prevent false positives
	node.setParam("height_verbose", false);
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
	node.setParam("ground_verbose", false);
	node.setParam("drive_linearspeed", 0.2);
	node.setParam("drive_angularspeed", 0.2);
	node.setParam("drive_move", false);
	node.setParam("drive_verbose", true);

	ObstacleAvoidance workhorse(node); //block to do obstacle avoidance


	//clean up parameters, plus a Vim macro to generate them from "default parameter values"
	node.deleteParam("crop_xradius");
	node.deleteParam("crop_ymin");
	node.deleteParam("crop_ymax");
	node.deleteParam("crop_zmin");
	node.deleteParam("crop_zmax");
	node.deleteParam("height_downsampling");
	node.deleteParam("height_samples");
	node.deleteParam("height_verbose");
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
	node.deleteParam("ground_verbose");
	node.deleteParam("drive_linearspeed");
	node.deleteParam("drive_angularspeed");
	node.deleteParam("drive_move");
	node.deleteParam("drive_verbose");
}
