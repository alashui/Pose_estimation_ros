#ifndef LocalizationRough_H
#define LocalizationRough_H

#include "localization/common_include.h"
#include "localization/pose_estimation.h"
#include "localization/image_retrieve.h"
#include "localization/config.h"

#include <opencv2/opencv.hpp>
#include <math.h>

#include "localization/image_capture.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace localization
{

class PoseResult   //表示相机位姿
{
	public:
		double score, x, y,theta;
		int frame_id,num_inliers;
		bool state;
		PoseResult():
			score(0), x(0), y(0),theta(0),
			frame_id(0),num_inliers(0),state(false){}		    			
}; 




}
#endif



 
