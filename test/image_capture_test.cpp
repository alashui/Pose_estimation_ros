#include "localization/common_include.h"
#include "localization/config.h"

#include "localization/image_capture.h"
#include "std_msgs/String.h"

int main(int argc, char** argv) 
{
    string parameter_file_dir;
    
    if ( argc != 2 )
    {
        cout<<"use default parameter_file!"<<endl;
        parameter_file_dir ="/home/robot/hi_robot/src/localization_ros/config/default.yaml";
        //return 1;
    }
    else
    	parameter_file_dir =argv[1];
    

    localization::Config::setParameterFile ( parameter_file_dir );  
    string capture_save_dir_ = localization::Config::get<string> ( "capture_save_dir" );

    ros::init( argc, argv, "image_captrue" );

    ROS_INFO("Capturer started...");
    localization::Capturer capture(capture_save_dir_);
	capture.Enable_=true;
}