/********************************************************
程序名称:pose_estimation
版本:v01
作者:赖立海
功能描述:通过kinect获得的图像信息,估计机器人的位姿,并作为导
		 航的初始位姿发布
流程:
	 步骤0.初始化--读入建图时的所有关键帧及其相应相机位姿，提取特征点，
	 			   计算描述子，计算词袋向量，这些信息初始化为一个数据库
	 			   
	 步骤1.捕获图像--根据机器人运动信息获取图像

	 步骤2.检索相似帧--在所有关键帧中检索与捕获的图像相似的帧
	 				  (视觉词袋实现) 得到若干相似帧,筛选得分高于阈
	 				  值的结果,若得分都太低则回到步骤1.
	 				  
	 步骤3.运动估计--通过检索出来的相似帧及其先验位姿信息计算当前
	 		   		捕获帧的相机位姿(3d-2d RansacPnP)
	 		   		
	 步骤4.筛选结果--根据词袋比较得分特征匹配数,RansacPnP的内点数,
		 		    解得的位姿相对相似帧相机位姿运动是否过大筛选,
		 		    若没有满足要求的结果则回到步骤1.
		 		    
	 步骤5.发布结果--对求得的比较好的结果加权平均得到最终结果

*************************************************************/
#include "localization/common_include.h"
#include "localization/pose_estimation.h"
#include "localization/image_retrieve.h"
#include "localization/config.h"

#include <opencv2/opencv.hpp>
#include <math.h>

#include "localization/image_capture.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "std_msgs/String.h"
//the following are UBUNTU/LINUX ONLY terminal color
#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */
class PoseResult   //表示相机位姿
{
	public:
		double score, x, y,theta;int frame_id,num_inliers;bool state;
		PoseResult():
			score(0), x(0), y(0),theta(0),
			frame_id(0),num_inliers(0),state(false){}		    			
};

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
    
/***初始化***/
	cout<<"initializing... "<<endl;
    localization::Config::setParameterFile ( parameter_file_dir );
        
    //string map_dir = localization::Config::get<string> ( "map_dir" );


    
	localization::PoseEstimation pose_estimation;    
	pose_estimation.mapInitialization();		//初始化地图
       
    //localization::ImageRetrieve::Ptr image_retrieve (new localization::ImageRetrieve );
    localization::ImageRetrieve image_retrieve;
    image_retrieve.map_ = pose_estimation.map_;  //数据库为地图中的关键帧
    image_retrieve.databaseInit(); 
    int num_result=20;
    image_retrieve.setResultNum(num_result);
    
    cout<<"initialization complete. "<<endl;
/*************/
    string image_query_dir = localization::Config::get<string> ( "image_query_dir" );
    string dir_result_after_g2o(image_query_dir  + "/result_after.g2o");
    string dir_res(image_query_dir+"/res.txt");
    string dir_gth(image_query_dir+"/gth.txt");
    string dir_err(image_query_dir+"/err.txt");
    
	ofstream fout_res(dir_res); //记录位姿估计结果 
	ofstream fout_gth(dir_gth); //记录基准位姿
	ofstream fout_err(dir_err); //记录误差 
	
	ifstream fin(dir_result_after_g2o);	 
	if (!fin)								
	{
		cerr<<"cannot find result_after.g2o file"<<endl;
			//return 1;
	}
	else//文件存在
	{

		string temp;	
		int num_line=0;
		while(getline(fin,temp))		//获取一行,一行数据格式为  VERTEX_SE3:QUAT 1   0 0 0 0 0 0 1  
											//八位数字分别表示 x y z qx qy qz qw(前三位为位置，后四位为四元数表示的旋转角)
		{	
			//对每一行数据的读入
			num_line++;
			if (num_line==2)	//忽略第二行 数据为 FIX 1
				continue;
			string temp_str;  //忽略每行前面的字符串 VERTEX_SE3:QUAT
			vector<double> double_vec; 
			double num;
			istringstream iss(temp);
			iss >> temp_str;
			if (temp_str!="VERTEX_SE3:QUAT") break;  	
				
			while(iss >> num)  		//分别将这一行数据读入						
				double_vec.push_back(num);
				
			image_retrieve.frame_query_=localization::Frame::createFrame();
			
			int frame_id= double_vec[0];
			
			image_retrieve.frame_query_->id_=frame_id;
			string rgb_dir = image_query_dir+"/rgb/rgb"+to_string(frame_id)+".png";
			string depth_dir = image_query_dir+"/depth/depth"+to_string(frame_id)+".png";     
			image_retrieve.frame_query_->color_ = imread(rgb_dir);
       		//image_retrieve.frame_query_->depth_ = imread(depth_dir,-1);


		
			//将x y z q1 q2 q3 q4 转换为sophus::se3 表示
			Eigen::Vector3d t(double_vec[1],double_vec[2],double_vec[3]);
			Eigen::Quaterniond q(double_vec[7],double_vec[4],double_vec[5],double_vec[6]);
											//Eigen::Quaterniond里使用顺序qw qx qy qz 
			Sophus::SE3 T1(q,t);
			//frame->T_c_w_ = T.inverse();
			
			//Sophus::SE3 Twc = T1.inverse();
			
			PoseResult pose_gth;							
			pose_gth.x= T1.matrix()(0,3);
			pose_gth.y= T1.matrix()(2,3);			
			pose_gth.theta= acos(0.5*((T1.matrix()(0,0)+T1.matrix()(1,1)+T1.matrix()(2,2))-1)) ;
			
			fout_gth << pose_gth.x << ' ' << pose_gth.y << ' ' << pose_gth.theta << endl;//记录基准位姿
						
			cout<<"frame"<<frame_id<<endl;
		    image_retrieve.frame_query_->extractKeyPoints();
		    image_retrieve.frame_query_->computeDescriptors();
		    
		    image_retrieve.retrieve_result();
		    
		    ////对检索出的结果估计位姿
		    vector<PoseResult>pose_result_vec;		    	
			for(int k=0; k<num_result; k++)
			{
				PoseResult pose_result;
				SE3 T_temp;
				pose_estimation.T_c_w_estimated_ = T_temp;		//对上一次的结果清零
				
				pose_result.frame_id = image_retrieve.EntryId_frame_id_[image_retrieve.result_[k].Id];        	
				pose_result.score = image_retrieve.result_[k].Score;
				
				if(pose_result.score<0.0150)		//得分太低的不计算运动
				{
					pose_result_vec.push_back(pose_result);
					continue;
				}
				/*
				cout <<"result " << k 
					 //<<" entry_id: " << image_retrieve.result_[k].Id 
					 <<"   frame_id: " << pose_result.frame_id
					 <<"   score: "  << pose_result.score 
					 <<endl;
			   */

				//Mat K = ( Mat_<double> ( 3,3 ) << 525.0, 0, 319.5, 0, 525.0, 239.5, 0, 0, 1 );                 
					            			 				
				pose_estimation.curr_= image_retrieve.frame_query_;
				pose_estimation.ref_= pose_estimation.map_->keyframes_[ pose_result.frame_id ];	   		
				if ( pose_estimation.featureMatching(pose_estimation.curr_, pose_estimation.ref_) ) //匹配成功才做运动估计
				{
					pose_estimation.poseEstimationPnP();
					pose_result.num_inliers =pose_estimation.num_inliers_	;	//RANSAC运动估计的内点数	
					pose_result.state = pose_estimation.checkEstimatedPose();	//运动估计结果的状态						
				}
						
				Sophus::SE3 Twc = pose_estimation.T_c_w_estimated_.inverse();			
				//cout <<"Twc"<<endl<<Twc.matrix() << endl <<endl <<endl;	
					
				pose_result.x= Twc.matrix()(0,3);
				pose_result.y= Twc.matrix()(2,3);			
				pose_result.theta= acos(0.5*((Twc.matrix()(0,0)+Twc.matrix()(1,1)+Twc.matrix()(2,2))-1)) ;
				pose_result_vec.push_back(pose_result);
										
			}
	
	
			////对结果进行筛选
			double pose_x(0), pose_y(0),pose_theta(0);
			vector<PoseResult>good_pose_vec;
			
			for(int k=0; k<num_result; k++)
				if(pose_result_vec[k].state)
					good_pose_vec.push_back(pose_result_vec[k]);
		
		
			int num_good_pose(good_pose_vec.size());
			//选出距离最近的两个结果,数据量不多暂且这样筛选数据
			int index_1(0),index_2(1);
			double min_dist(100.0);			
			if(num_good_pose >=2)
			{
				struct dist_ 
				{
					double operator () (const PoseResult &res1,const PoseResult &res2) 
					{
						return sqrt( pow(res1.x-res2.x, 2)+pow(res1.y-res2.y, 2) );
					}       
				} dist;
				
				if(num_good_pose =2)
					min_dist=dist(good_pose_vec[index_1], good_pose_vec[index_2]);
				
				else 
				{	    		    
					for(int i=0;i<num_good_pose;i++)
					for(int j=i+1;j<num_good_pose;j++)
					{
						double d_t = dist(good_pose_vec[i], good_pose_vec[j]);
						if(d_t < min_dist)
						{
							min_dist=d_t; //距离最近的两个结果之间的距离
							index_1=i;
							index_2=j;
						}
					}
				}
			}
			if( min_dist < 0.8 )
			{
				pose_x=0.5*(good_pose_vec[index_1].x + good_pose_vec[index_2].x);
				pose_y=0.5*(good_pose_vec[index_1].y + good_pose_vec[index_2].y);
				pose_theta=0.5*(good_pose_vec[index_1].theta + good_pose_vec[index_2].theta);
			


				cout<<BOLDGREEN"good pose :" <<" x:" << pose_x 	
									<<"   y:" << pose_y 
									<<"   theta:" << pose_theta
									<<endl<<endl<<endl;
								
									
				fout_res << pose_x << ' ' << pose_y << ' ' << pose_theta <<' '<<"1" << endl;//记录基准位姿	
				fout_err << fabs(pose_x-pose_gth.x) << ' ' 
					 << fabs(pose_y-pose_gth.y) << ' ' 
					 << fabs(pose_theta-pose_gth.theta) <<' '
					 <<"1" << endl;						
			}
			else			 
			{

				cout <<BOLDRED"not find valid similar frame!" <<endl;
				cout << endl <<endl;				
			
				fout_res << pose_x << ' ' << pose_y << ' ' << pose_theta <<' '<<"0" << endl;//记录基准位姿
				fout_err << fabs(pose_x-pose_gth.x) << ' ' 
					 << fabs(pose_y-pose_gth.y) << ' ' 
					 << fabs(pose_theta-pose_gth.theta) <<' '
					 <<"0" << endl;					
			}							
		}
	} 
		
	
	fout_res.close();
	fout_gth.close();
	fout_err.close();

}

















