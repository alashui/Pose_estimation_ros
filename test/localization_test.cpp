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
        
    string map_dir = localization::Config::get<string> ( "map_dir" );
    //string image_query_dir = localization::Config::get<string> ( "image_query_dir" );
    string capture_save_dir_ = localization::Config::get<string> ( "capture_save_dir" );
    string dir_result_detailed_out(capture_save_dir_+"/result_detailed.txt");
	string dir_result_simple_out(capture_save_dir_+"/result_simple.txt");
    
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

	/*初始化节点,订阅/odom话题中的消息,在话题/initialpose发布位姿估计信息*/
    ros::init( argc, argv, "PoseEstimation" );
    ros::NodeHandle n;    
	ros::Publisher initial_pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
	ros::Publisher resultState_pub = n.advertise<std_msgs::String>("/localization_state",10);
	
	
    ROS_INFO("Capturer started...");
    localization::Capturer capture(capture_save_dir_);
	capture.Enable_=true;
	
	int num_image_captured=0;      //捕获成功的图像数量（实际计数的是参与查询的的数量）

	
	//ros::Rate loop_rate(10);

	ofstream fout(dir_result_detailed_out); //记录详细输出结果
	ofstream fout1(dir_result_simple_out); //记录简单输出结果
		
	fout1<<"image_index state  x_value y_value theta_value"<<endl;
	
    while( ros::ok() )
    {
    	ros::spinOnce();               // check for incoming messages
					
		if(capture.state_)		//图像获取成功,计算相机位姿
		{
			capture.state_=false;
			
			num_image_captured++;
			
		/***对捕获到的图像检索相似帧***/           		 		                     
		    image_retrieve.frame_query_->color_ = capture.color_;
		    image_retrieve.frame_query_->depth_ = capture.depth_;   
		    image_retrieve.frame_query_->extractKeyPoints();
		    image_retrieve.frame_query_->computeDescriptors();
		    
		    image_retrieve.retrieve_result();         

		    cout<<"searching for image_query "<< num_image_captured <<" returns "<< num_result <<" results" <<endl;
		    fout<<"searching for image_query "<< num_image_captured <<" returns "<< num_result <<" results" <<endl;
		/***************************/    
		 
		/***对得到的所有相似帧，分别估计当前相机位姿***/    
		    fout1<< num_image_captured << "  ";

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
		    	
				cout <<"result " << k 
					 //<<" entry_id: " << image_retrieve.result_[k].Id 
					 <<"   frame_id: " << pose_result.frame_id
					 <<"   score: "  << pose_result.score 
					 <<endl;
			   
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
				cout <<"Twc"<<endl<<Twc.matrix() << endl <<endl <<endl;	
						
				pose_result.x= Twc.matrix()(0,3);
				pose_result.y= Twc.matrix()(2,3);			
				pose_result.theta= acos(0.5*((Twc.matrix()(0,0)+Twc.matrix()(1,1)+Twc.matrix()(2,2))-1)) ;
				pose_result_vec.push_back(pose_result);
											
			}
		/***********************************/
			
			
		/***记录所有结果到文件***/
			cout <<"result"<<endl;
			for(int k=0; k<num_result; k++)
			{
 			    if(pose_result_vec[k].score<0.0150) //得分太低的没有计算运动，结果都是0，不保存
		    	{
		    		continue;
		    	}
		    	
				fout << "result " << k << ":   " 
					 << "frame_id: " << pose_result_vec[k].frame_id << "   "
					 << " score: "  << pose_result_vec[k].score << "   "
					 << " num_inliers: "  << pose_result_vec[k].num_inliers << "   "
					 << "x:"<< pose_result_vec[k].x << "   "
					 << "y:"<< pose_result_vec[k].y << "   "
					 << "theta:"<<pose_result_vec[k].theta <<"   "
					 << "state:"<<pose_result_vec[k].state << endl;
			}
		/**********************/	
	
			//对求得的结果作加权平均，分数占0.8，内点0.2
			
			double pose_x(0), pose_y(0),pose_theta(0);
			//double pose_score_total(0), pose_inliers_total(0);	
			vector<PoseResult>good_pose_vec;
				
			for(int k=0; k<num_result; k++)
			{
				if(pose_result_vec[k].state)
				{
					//num_good_pose++;
					good_pose_vec.push_back(pose_result_vec[k]);
					
					//pose_score_total += pose_result_vec[k].score;
					//pose_inliers_total += pose_result_vec[k].num_inliers;
				}	
			}
			
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
			if(min_dist < 0.8)
			{
				pose_x=0.5*(good_pose_vec[index_1].x + good_pose_vec[index_2].x);
				pose_y=0.5*(good_pose_vec[index_1].y + good_pose_vec[index_2].y);
				pose_theta=0.5*(good_pose_vec[index_1].theta + good_pose_vec[index_2].theta);
				
				
				/*
				for(int k=0; k<num_result; k++)
				{
					if(pose_result_vec[k].state)//改掉，不做加权，而是选
					{					
						double pose_weight(0);
						pose_weight =( (0.8*pose_result_vec[k].score   /pose_score_total)  + 
								       (0.2*pose_result_vec[k].num_inliers /pose_inliers_total)  );
						pose_x +=pose_weight * pose_result_vec[k].x; 							  
						pose_y +=pose_weight * pose_result_vec[k].y;
						pose_theta +=pose_weight * pose_result_vec[k].theta;
					}	

				}*/
				
				/*根据计算结果发布初始位姿消息*/
				geometry_msgs::PoseWithCovarianceStamped pose;
				pose.header.frame_id = "map";
				pose.pose.pose.position.x= pose_x;
				pose.pose.pose.position.y= pose_y;
				pose.pose.pose.position.z= 0;					  
				pose.pose.pose.orientation.x= 0;
				pose.pose.pose.orientation.y= 0;						  
				pose.pose.pose.orientation.z= sin(0.5*(pose_theta-0.5*M_PI));
				pose.pose.pose.orientation.w= cos(0.5*(pose_theta-0.5*M_PI));
				pose.pose.covariance={ 0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
									   0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 
									   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
									   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
									   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
									   0.0, 0.0, 0.0, 0.0, 0.0, 0.068 };
				initial_pose_pub.publish(pose);
				std_msgs::String state_temp;
				std::stringstream ss;
				ss << "FIND " << endl;
				state_temp.data = ss.str();
				resultState_pub.publish(state_temp);
				ros::spinOnce();
				
				cout<<BOLDGREEN"good pose :" <<" x:" << pose_x 	
									<<"   y:" << pose_y 
									<<"   theta:" << pose_theta
									<<endl<<endl<<endl;
									
										
				fout<<"good pose :" <<" x:" << pose_x 	
									<<"   y:" << pose_y 
									<<"   theta:" << pose_theta
									<<"   min_dist: "<<min_dist<<endl;	
				fout << endl << endl <<endl;
					
				fout1<<"find "<< pose_x 	
							  <<" " << pose_y 
							  <<" " << pose_theta
							  <<endl;	
													
			}
			else			 
			{
				std_msgs::String state_temp;
				std::stringstream ss;
				ss << "LOST " << endl;
				state_temp.data = ss.str();
				resultState_pub.publish(state_temp);
				ros::spinOnce();
				
				cout <<BOLDRED"not find valid similar frame!" <<endl;
				cout << endl <<endl;	
				
				
				fout << "not find valid similar frame!" <<endl;
				fout << endl <<endl << endl ;
				
				fout1 << "lost" <<endl;				
			} 
		
		}
 	
		//loop_rate.sleep();
  	}
	fout.close();
	fout1.close();
}



/*

result for rgb300.png

result 0:    frame_id: 300    score: 1   		 x:13.9296   y:2.82265   theta:2.41964
result 1:    frame_id: 301    score: 0.102776    x:13.9793   y:2.78729   theta:2.42875
result 2:    frame_id: 299    score: 0.0693958   x:13.8875   y:2.86637   theta:2.41272
result 3:    frame_id: 302    score: 0.0536191   x:13.996    y:2.76473   theta:2.4303
result 4:    frame_id: 298    score: 0.0448145   x:112.629   y:-30.5906  theta:3.02878
result 5:    frame_id: 303    score: 0.0435787   x:14.9075   y:0.737955  theta:2.32366
result 6:    frame_id: 305    score: 0.037726    x:13.9842   y:2.84597   theta:2.43421
result 7:    frame_id: 304    score: 0.0310616   x:13.9858   y:2.81322   theta:2.43141
result 8:    frame_id: 306    score: 0.0286385   x:13.8923   y:2.84655   theta:2.40503
result 9:    frame_id: 307    score: 0.0264989   x:13.8643   y:2.86456   theta:2.3985
result 10:   frame_id: 308    score: 0.0244973   x:13.8881   y:2.82965   theta:2.40215
result 11:   frame_id: 297    score: 0.0135155   x:6.565     y:8.81041   theta:2.74595
result 12:   frame_id: 256    score: 0.0111524   x:10509.5   y:3854.42   theta:1.48
result 13:   frame_id: 311    score: 0.0108131   x:9.89627   y:0.878723  theta:1.96324
result 14:   frame_id: 110    score: 0.0106924   x:9.26337   y:16.8362   theta:0.807775
result 15:   frame_id: 317    score: 0.00989605  x:34147.5   y:57296.7   theta:2.33352
result 16:   frame_id: 340    score: 0.00939412  x:7.00126   y:-2.26659  theta:2.67488
result 17:   frame_id: 270    score: 0.00890024  x:16.3584   y:0.0297195 theta:2.63166
result 18:   frame_id: 370    score: 0.00875891  x:-20.455   y:12.4545   theta:3.06371
result 19:   frame_id: 267    score: 0.00873694  x:13.6721   y:-1.4608   theta:2.1875




result for rgb452.png

result 0:    frame_id: 452    score: 1           x:0.408557   	y:-2.40073   theta:0.782908
result 1:    frame_id: 453    score: 0.108716    x:0.356602   	y:-2.41706   theta:0.77187
result 2:    frame_id: 451    score: 0.106872    x:0.482839   	y:-2.3951    theta:0.796882
result 3:    frame_id: 454    score: 0.0810632   x:0.239636   	y:-2.41744   theta:0.748581
result 4:    frame_id: 450    score: 0.0662164   x:-4.04426   	y:4.84234    theta:3.10194
result 5:    frame_id: 455    score: 0.0624233   x:0.186706   	y:-2.44466   theta:0.735096
result 6:    frame_id: 449    score: 0.0553199   x:-2.57991   	y:5.7375     theta:3.11677
result 7:    frame_id: 456    score: 0.0416935   x:-5.16213   	y:-1.73001   theta:3.0485
result 8:    frame_id: 458    score: 0.0310182   x:-6.29236   	y:-1.30025   theta:3.06427
result 9:    frame_id: 448    score: 0.0299908   x:5.90433    	y:2.05365    theta:2.68338
result 10:   frame_id: 457    score: 0.0251521   x:-4.79126   	y:-2.17759   theta:2.94126
result 11:   frame_id: 461    score: 0.0208819   x:-10.8196   	y:-18.6922   theta:3.10637
result 12:   frame_id: 463    score: 0.0190736   x:1.74737    	y:-1.28862   theta:2.87178
result 13:   frame_id: 459    score: 0.0187747   x:24.6494    	y:16.2173    theta:1.99607
result 14:   frame_id: 460    score: 0.0185662   x:-2.16e+06    y:-1.591e+06 theta:2.45028
result 15:   frame_id: 462    score: 0.0173106   x:0.297174   	y:-2.19306   theta:0.779233
result 16:   frame_id: 447    score: 0.0129455   x:-1.36563   	y:1.73796    theta:2.05446
result 17:   frame_id: 110    score: 0.0110651   x:-67.6046   	y:-40.2864   theta:2.94448
result 18:   frame_id: 4      score: 0.0109389   x:-0.0288433   y:0.0145766  theta:1.09768
result 19:   frame_id: 127    score: 0.0104159   x:3.29576   	y:5.21191    theta:1.43948



result for rgb260.png 

result 0:    frame_id: 260    score: 1           x:9.88652   y:3.61278   theta:2.72927
result 1:    frame_id: 261    score: 0.0864794   x:10.3216   y:3.01969   theta:2.73475
result 2:    frame_id: 258    score: 0.0818935   x:10.3458   y:2.96634   theta:2.73824
result 3:    frame_id: 259    score: 0.0803074   x:10.3559   y:3.01768   theta:2.7419
result 4:    frame_id: 256    score: 0.0754513   x:10.3826   y:2.97796   theta:2.74815
result 5:    frame_id: 253    score: 0.0697977   x:10.3326   y:2.98229   theta:2.7439
result 6:    frame_id: 257    score: 0.0630406   x:10.3628   y:2.96788   theta:2.74328
result 7:    frame_id: 262    score: 0.061544    x:10.3165   y:3.0371    theta:2.73522
result 8:    frame_id: 254    score: 0.0599786   x:10.435    y:3.02145   theta:2.76374
result 9:    frame_id: 252    score: 0.0598985   x:10.4017   y:3.0538    theta:2.76079
result 10:   frame_id: 251    score: 0.0540477   x:10.3529   y:3.06178   theta:2.75177
result 11:   frame_id: 255    score: 0.05008     x:10.3326   y:2.9465    theta:2.73857
result 12:   frame_id: 263    score: 0.0360672   x:10.3219   y:3.04862   theta:2.73592
result 13:   frame_id: 264    score: 0.0319114   x:10.2862   y:3.08902   theta:2.73475
result 14:   frame_id: 249    score: 0.0304109   x:10.3726   y:3.08206   theta:2.7598
result 15:   frame_id: 248    score: 0.0262705   x:10.3619   y:3.05817   theta:2.75758
result 16:   frame_id: 265    score: 0.0260094   x:10.2952   y:3.09771   theta:2.73509
result 17:   frame_id: 250    score: 0.0258338   x:10.3628   y:3.01612   theta:2.75198
result 18:   frame_id: 247    score: 0.0215262   x:22531.4   y:1601.53   theta:1.60017
result 19:   frame_id: 267    score: 0.0186977   x:9.99037   y:2.99555   theta:2.68821




result  for rgb2.png 

result 0:    frame_id: 1      score: 0.144893    x:0.000482244   y:0.0234564   theta:0.028621
result 1:    frame_id: 477    score: 0.11729     x:0.0378861     y:0.0749989   theta:0.0359883
result 2:    frame_id: 475    score: 0.108709    x:0.0195725     y:0.00706001  theta:0.0421862
result 3:    frame_id: 4      score: 0.108444    x:0.0272674     y:0.0527278   theta:0.0332201
result 4:    frame_id: 3      score: 0.107565    x:0.0118323     y:0.0450191   theta:0.0297721
result 5:    frame_id: 478    score: 0.0983268   x:0.0470925     y:0.0818096   theta:0.0318857
result 6:    frame_id: 476    score: 0.0972937   x:0.0217826     y:0.00455701  theta:0.0367172
result 7:    frame_id: 6      score: 0.0917573   x:-1527.85      y:-9417.39    theta:2.73811
result 8:    frame_id: 474    score: 0.0913623   x:-0.0314661    y:0.00241921  theta:0.0312285
result 9:    frame_id: 479    score: 0.0876875   x:-1.22341      y:8.21544     theta:3.1272
result 10:   frame_id: 5      score: 0.0851114   x:0.0177218     y:0.0872163   theta:0.0342836
result 11:   frame_id: 482    score: 0.0825751   x:-1.21187      y:8.55739     theta:3.13638
result 12:   frame_id: 480    score: 0.073984    x:-1.26003      y:8.1173      theta:3.12611
result 13:   frame_id: 471    score: 0.0738817   x:0.0141057     y:-0.0605316  theta:0.0504018
result 14:   frame_id: 7      score: 0.0703707   x:0.26556       y:-0.159435   theta:0.0898385
result 15:   frame_id: 481    score: 0.0679087   x:0.885976      y:9.03687     theta:3.12507
result 16:   frame_id: 469    score: 0.0671866   x:-323377       y:-353985     theta:0.803534
result 17:   frame_id: 462    score: 0.0662981   x:0.0528013     y:-0.569841   theta:0.0689776
result 18:   frame_id: 8      score: 0.064611    x:0.206035      y:-0.144628   theta:0.0824043
result 19:   frame_id: 472    score: 0.0642401   x:86209.5       y:5929.47     theta:2.90297




result for rgb166.png 
result 0:    frame_id: 166    score: 1   		 x:5.93231   y:2.12083   theta:2.98002
result 1:    frame_id: 167    score: 0.0711954   x:5.89423   y:2.10534   theta:2.99095
result 2:    frame_id: 168    score: 0.05143     x:5.82774   y:2.02262   theta:3.00744
result 3:    frame_id: 162    score: 0.0433674   x:6.07323   y:2.24401   theta:2.94426
result 4:    frame_id: 172    score: 0.0425471   x:5.92175   y:1.8964    theta:2.98179
result 5:    frame_id: 163    score: 0.0415118   x:5.93876   y:2.29396   theta:2.98268
result 6:    frame_id: 169    score: 0.0397726   x:5.81286   y:2.00867   theta:3.01124
result 7:    frame_id: 161    score: 0.0377171   x:6.00542   y:2.44722   theta:2.97254
result 8:    frame_id: 164    score: 0.0367645   x:5.93752   y:2.26431   theta:2.98401
result 9:    frame_id: 170    score: 0.0272418   x:5.76781   y:1.92724   theta:3.02319
result 10:   frame_id: 173    score: 0.0248939   x:6.1099    y:2.30433   theta:2.96002
result 11:   frame_id: 158    score: 0.0214829   x:7.10162   y:-8.507    theta:2.89693
result 12:   frame_id: 174    score: 0.0213827   x:5.71859   y:2.4348    theta:3.02341
result 13:   frame_id: 157    score: 0.0207831   x:7.22937   y:-9.53613  theta:3.07671
result 14:   frame_id: 160    score: 0.0192237   x:5.8681    y:1.99376   theta:2.98528
result 15:   frame_id: 171    score: 0.0186299   x:5.75016   y:1.88338   theta:3.02816
result 16:   frame_id: 159    score: 0.0172483   x:5.17752   y:-9.54924  theta:3.04377
result 17:   frame_id: 156    score: 0.0139583   x:7.07287   y:-8.96258  theta:2.80645
result 18:   frame_id: 175    score: 0.0132031   x:5.64273   y:2.57497   theta:3.03079
result 19:   frame_id: 121    score: 0.0122519   x:0.802124  y:-1.1073   theta:1.27738


result for rgb260.png 

result 0:    frame_id: 260    score: 1   		 x:9.88652   y:3.61278   theta:2.72927   state:1
result 1:    frame_id: 261    score: 0.0864794   x:10.3216   y:3.01969   theta:2.73475   state:1
result 2:    frame_id: 258    score: 0.0818935   x:10.3458   y:2.96634   theta:2.73824   state:1
result 3:    frame_id: 259    score: 0.0803074   x:10.3559   y:3.01768   theta:2.7419    state:1
result 4:    frame_id: 256    score: 0.0754513   x:10.3826   y:2.97796   theta:2.74815   state:1
result 5:    frame_id: 253    score: 0.0697977   x:10.3326   y:2.98229   theta:2.7439    state:1
result 6:    frame_id: 257    score: 0.0630406   x:10.3628   y:2.96788   theta:2.74328   state:1
result 7:    frame_id: 262    score: 0.061544    x:10.3165   y:3.0371    theta:2.73522   state:1
result 8:    frame_id: 254    score: 0.0599786   x:10.435    y:3.02145   theta:2.76374   state:1
result 9:    frame_id: 252    score: 0.0598985   x:10.4017   y:3.0538    theta:2.76079   state:1
result 10:   frame_id: 251    score: 0.0540477   x:10.3529   y:3.06178   theta:2.75177   state:1
result 11:   frame_id: 255    score: 0.05008     x:10.3326   y:2.9465    theta:2.73857   state:1
result 12:   frame_id: 263    score: 0.0360672   x:10.3219   y:3.04862   theta:2.73592   state:1
result 13:   frame_id: 264    score: 0.0319114   x:10.2862   y:3.08902   theta:2.73475   state:1
result 14:   frame_id: 249    score: 0.0304109   x:10.3726   y:3.08206   theta:2.7598    state:1
result 15:   frame_id: 248    score: 0.0262705   x:10.3619   y:3.05817   theta:2.75758   state:1
result 16:   frame_id: 265    score: 0.0260094   x:10.2952   y:3.09771   theta:2.73509   state:1
result 17:   frame_id: 250    score: 0.0258338   x:10.3628   y:3.01612   theta:2.75198   state:1
result 18:   frame_id: 247    score: 0.0215262   x:22531.4   y:1601.53   theta:1.60017   state:0
result 19:   frame_id: 267    score: 0.0186977   x:9.99037   y:2.99555   theta:2.68821   state:1
result 20:   frame_id: 270    score: 0.0127184   x:19.4899   y:-3.43963  theta:0.641845  state:0
result 21:   frame_id: 390    score: 0.0121089   x:3.12576   y:-3.14527  theta:1.69019   state:0
result 22:   frame_id: 272    score: 0.0120134   x:-85.9652  y:-61.811   theta:3.0239    state:0
result 23:   frame_id: 269    score: 0.0117791   x:15993.8   y:13644.6   theta:3.01761   state:0
result 24:   frame_id: 246    score: 0.0114794   x:12.1802   y:-11.654   theta:3.10526   state:0
result 25:   frame_id: 266    score: 0.0112503   x:10.2204   y:3.00691   theta:2.71409   state:1
result 26:   frame_id: 268    score: 0.0097494   x:10.0221   y:2.9804    theta:2.68733   state:1
result 27:   frame_id: 311    score: 0.00927616  x:4.13517   y:-2.40855  theta:2.45819   state:0
result 28:   frame_id: 428    score: 0.00913736  x:3.03934   y:-1.63611  theta:2.02017   state:0
result 29:   frame_id: 431    score: 0.00913369  x:3.45266   y:-0.26802  theta:2.62451   state:0

result for rgb108.png 
result 0:    frame_id: 148    score: 0.0126872    x:5.73222   y:2.03783    theta:1.04336   state:0
result 1:    frame_id: 433    score: 0.0120464    x:-1.5408   y:-1.38974   theta:1.33463   state:0
result 2:    frame_id: 359    score: 0.0111475    x:9.13025   y:-2.15195   theta:1.19723   state:1
result 3:    frame_id: 151    score: 0.0106566    x:-20.6192  y:-23.9704   theta:1.26239   state:0
result 4:    frame_id: 176    score: 0.00925884   x:2.21208   y:1.33933    theta:2.25988   state:0
result 5:    frame_id: 318    score: 0.00922569   x:13.9951   y:-0.0801253 theta:1.93416   state:0
result 6:    frame_id: 182    score: 0.00917663   x:14.1641   y:-1.77477   theta:0.842042  state:0
result 7:    frame_id: 175    score: 0.00916646   x:0   	  y:0   	   theta:0  	   state:0
result 8:    frame_id: 183    score: 0.00907113   x:12.6079   y:-3.71523   theta:3.01019   state:0
result 9:    frame_id: 41     score: 0.00904253   x:2.33991   y:7.214      theta:0.665852  state:0
result 10:   frame_id: 48     score: 0.00875533   x:2.15045   y:-1.94479   theta:2.19531   state:0
result 11:   frame_id: 270    score: 0.00851405   x:4.07441   y:-3.97683   theta:1.38664   state:0
result 12:   frame_id: 105    score: 0.0082153    x:9.75673   y:5.94076    theta:2.64359   state:0
result 13:   frame_id: 181    score: 0.00808754   x:2.87382   y:2.17858    theta:1.97673   state:0
result 14:   frame_id: 261    score: 0.00761995   x:11.4178   y:-6.94552   theta:0.967663  state:0
result 15:   frame_id: 395    score: 0.00735333   x:41.1342   y:6.50983    theta:2.48904   state:0
result 16:   frame_id: 152    score: 0.00731932   x:0   	  y:0   	   theta:0   	   state:0
result 17:   frame_id: 254    score: 0.00729397   x:10.5334   y:-2.5741    theta:1.32274   state:0
result 18:   frame_id: 43     score: 0.00721448   x:12.2319   y:1.5622     theta:2.71008   state:0
result 19:   frame_id: 93     score: 0.00721273   x:0  	  	  y:0   	   theta:0   	   state:0
result 20:   frame_id: 83     score: 0.00716563   x:4.97184   y:2.40009    theta:2.21632   state:0
result 21:   frame_id: 238    score: 0.00707212   x:8.7471    y:-1.25118   theta:2.64561   state:0
result 22:   frame_id: 243    score: 0.00705722   x:13.6948   y:3.02097    theta:1.21648   state:0
result 23:   frame_id: 178    score: 0.00704144   x:0   	  y:0   	   theta:0   	   state:0
result 24:   frame_id: 174    score: 0.00696645   x:0   	  y:0      	   theta:0   	   state:0
result 25:   frame_id: 109    score: 0.00693818   x:8.3267    y:2.75408    theta:2.28706   state:0
result 26:   frame_id: 331    score: 0.00690783   x:5.68314   y:-5.4072    theta:1.20681   state:0
result 27:   frame_id: 17     score: 0.0068879    x:-2.91933  y:0.561363   theta:1.98133   state:0
result 28:   frame_id: 457    score: 0.00682607   x:-622.976  y:-441.11    theta:1.51505   state:0
result 29:   frame_id: 266    score: 0.00677521   x:14.3227   y:-1.02441   theta:0.881669  state:0












[ INFO] [1510385715.153572786]: Saving images
[ INFO] [1510385715.153833802]: db info 1
[ INFO] [1510385715.154442472]: db info 2
RGB 801
depth 796
extract keypoints cost time: 0.010369
extract computeDescriptors cost time: 0.009579
image retrieve done.
searching for image_query  returns 20 results
result
result 0:   frame_id: 144    score: 0.0115343   x:0   y:0   theta:0   state:0
result 1:   frame_id: 267    score: 0.0109804   x:0   y:0   theta:0   state:0
result 2:   frame_id: 197    score: 0.0107532   x:0   y:0   theta:0   state:0
result 3:   frame_id: 306    score: 0.0105963   x:0   y:0   theta:0   state:0
result 4:   frame_id: 53    score: 0.0102801   x:0   y:0   theta:0   state:0
result 5:   frame_id: 1    score: 0.0101397   x:0   y:0   theta:0   state:0
result 6:   frame_id: 155    score: 0.00930604   x:0   y:0   theta:0   state:0
result 7:   frame_id: 80    score: 0.00925041   x:0   y:0   theta:0   state:0
result 8:   frame_id: 52    score: 0.00924555   x:0   y:0   theta:0   state:0
result 9:   frame_id: 76    score: 0.00880561   x:0   y:0   theta:0   state:0
result 10:   frame_id: 269    score: 0.00878789   x:0   y:0   theta:0   state:0
result 11:   frame_id: 263    score: 0.00865759   x:0   y:0   theta:0   state:0
result 12:   frame_id: 232    score: 0.00860694   x:0   y:0   theta:0   state:0
result 13:   frame_id: 36    score: 0.00855813   x:0   y:0   theta:0   state:0
result 14:   frame_id: 128    score: 0.00845718   x:0   y:0   theta:0   state:0
result 15:   frame_id: 87    score: 0.00828084   x:0   y:0   theta:0   state:0
result 16:   frame_id: 152    score: 0.00817061   x:0   y:0   theta:0   state:0
result 17:   frame_id: 317    score: 0.00766044   x:0   y:0   theta:0   state:0
result 18:   frame_id: 285    score: 0.00756331   x:0   y:0   theta:0   state:0
result 19:   frame_id: 174    score: 0.00733861   x:0   y:0   theta:0   state:0
not find valid similar frame!
not find valid similar frame!
not find valid similar frame!
capture.Enable_ = true 
dd: 0.100002   th: 0.0146551
 callback cvbrige
 callback cvbrige enable
[ INFO] [1510385715.806979215]: Saving images
[ INFO] [1510385715.807018452]: db info 1
[ INFO] [1510385715.807388895]: db info 2
RGB 811
depth 812
extract keypoints cost time: 0.00786
extract computeDescriptors cost time: 0.012438
image retrieve done.
searching for image_query  returns 20 results
result
result 0:   frame_id: 81    score: 0.0128038   x:0   y:0   theta:0   state:0
result 1:   frame_id: 324    score: 0.00985493   x:0   y:0   theta:0   state:0
result 2:   frame_id: 172    score: 0.00948935   x:0   y:0   theta:0   state:0
result 3:   frame_id: 31    score: 0.00942564   x:0   y:0   theta:0   state:0
result 4:   frame_id: 40    score: 0.0088545   x:0   y:0   theta:0   state:0
result 5:   frame_id: 76    score: 0.00835628   x:0   y:0   theta:0   state:0
result 6:   frame_id: 228    score: 0.00796765   x:0   y:0   theta:0   state:0
result 7:   frame_id: 182    score: 0.00760807   x:0   y:0   theta:0   state:0
result 8:   frame_id: 355    score: 0.00757068   x:0   y:0   theta:0   state:0
result 9:   frame_id: 175    score: 0.00752849   x:0   y:0   theta:0   state:0
result 10:   frame_id: 319    score: 0.00744803   x:0   y:0   theta:0   state:0
result 11:   frame_id: 420    score: 0.00736176   x:0   y:0   theta:0   state:0
result 12:   frame_id: 466    score: 0.00735321   x:0   y:0   theta:0   state:0
result 13:   frame_id: 69    score: 0.00726397   x:0   y:0   theta:0   state:0
result 14:   frame_id: 180    score: 0.00726383   x:0   y:0   theta:0   state:0
result 15:   frame_id: 70    score: 0.00725522   x:0   y:0   theta:0   state:0
result 16:   frame_id: 18    score: 0.00725116   x:0   y:0   theta:0   state:0
result 17:   frame_id: 1    score: 0.00723031   x:0   y:0   theta:0   state:0
result 18:   frame_id: 478    score: 0.00715514   x:0   y:0   theta:0   state:0
result 19:   frame_id: 388    score: 0.00707371   x:0   y:0   theta:0   state:0
not find valid similar frame!
not find valid similar frame!
not find valid similar frame!
capture.Enable_ = true 
dd: 0.1   th: 0.0448896
 callback cvbrige
 callback cvbrige enable
[ INFO] [1510385716.430074799]: Saving images
[ INFO] [1510385716.430107692]: db info 1
[ INFO] [1510385716.430655312]: db info 2
RGB 829
depth 831
extract keypoints cost time: 0.007893
extract computeDescriptors cost time: 0.008948
image retrieve done.
searching for image_query  returns 20 results
result
result 0:   frame_id: 82    score: 0.0127187   x:0   y:0   theta:0   state:0
result 1:   frame_id: 80    score: 0.0123598   x:0   y:0   theta:0   state:0
result 2:   frame_id: 156    score: 0.0109472   x:0   y:0   theta:0   state:0
result 3:   frame_id: 311    score: 0.0108439   x:0   y:0   theta:0   state:0
result 4:   frame_id: 340    score: 0.0105163   x:0   y:0   theta:0   state:0
result 5:   frame_id: 176    score: 0.00959763   x:0   y:0   theta:0   state:0
result 6:   frame_id: 89    score: 0.00923207   x:0   y:0   theta:0   state:0
result 7:   frame_id: 84    score: 0.00920599   x:0   y:0   theta:0   state:0
result 8:   frame_id: 348    score: 0.00907773   x:0   y:0   theta:0   state:0
result 9:   frame_id: 464    score: 0.00880553   x:0   y:0   theta:0   state:0
result 10:   frame_id: 356    score: 0.00878365   x:0   y:0   theta:0   state:0
result 11:   frame_id: 467    score: 0.00849373   x:0   y:0   theta:0   state:0
result 12:   frame_id: 426    score: 0.00802604   x:0   y:0   theta:0   state:0
result 13:   frame_id: 264    score: 0.00799802   x:0   y:0   theta:0   state:0
result 14:   frame_id: 441    score: 0.0079008   x:0   y:0   theta:0   state:0
result 15:   frame_id: 261    score: 0.00779578   x:0   y:0   theta:0   state:0
result 16:   frame_id: 255    score: 0.0076482   x:0   y:0   theta:0   state:0
result 17:   frame_id: 179    score: 0.00757875   x:0   y:0   theta:0   state:0
result 18:   frame_id: 81    score: 0.00741357   x:0   y:0   theta:0   state:0
result 19:   frame_id: 46    score: 0.00728175   x:0   y:0   theta:0   state:0
not find valid similar frame!
not find valid similar frame!
not find valid similar frame!
capture.Enable_ = true 
dd: 0.1   th: 0.0432207
 callback cvbrige
 callback cvbrige enable
[ INFO] [1510385717.045978256]: Saving images
[ INFO] [1510385717.046014338]: db info 1
[ INFO] [1510385717.046282465]: db info 2
RGB 845
depth 849
capture.Enable_ = true 
dd: 0.101143   th: 0.0418197
extract keypoints cost time: 0.007568
extract computeDescriptors cost time: 0.008727
image retrieve done.
searching for image_query  returns 20 results
result
result 0:   frame_id: 89    score: 0.0119371   x:0   y:0   theta:0   state:0
result 1:   frame_id: 315    score: 0.0119299   x:0   y:0   theta:0   state:0
result 2:   frame_id: 409    score: 0.0116333   x:0   y:0   theta:0   state:0
result 3:   frame_id: 81    score: 0.0112111   x:0   y:0   theta:0   state:0
result 4:   frame_id: 193    score: 0.0108017   x:0   y:0   theta:0   state:0
result 5:   frame_id: 78    score: 0.0106892   x:0   y:0   theta:0   state:0
result 6:   frame_id: 88    score: 0.0106059   x:0   y:0   theta:0   state:0
result 7:   frame_id: 60    score: 0.0105833   x:0   y:0   theta:0   state:0
result 8:   frame_id: 319    score: 0.0104557   x:0   y:0   theta:0   state:0
result 9:   frame_id: 388    score: 0.010271   x:0   y:0   theta:0   state:0
result 10:   frame_id: 324    score: 0.0099559   x:0   y:0   theta:0   state:0
result 11:   frame_id: 86    score: 0.0096067   x:0   y:0   theta:0   state:0
result 12:   frame_id: 348    score: 0.00949405   x:0   y:0   theta:0   state:0
result 13:   frame_id: 62    score: 0.00925752   x:0   y:0   theta:0   state:0
result 14:   frame_id: 76    score: 0.00900158   x:0   y:0   theta:0   state:0
result 15:   frame_id: 432    score: 0.00895284   x:0   y:0   theta:0   state:0
result 16:   frame_id: 79    score: 0.00887818   x:0   y:0   theta:0   state:0
result 17:   frame_id: 85    score: 0.00881091   x:0   y:0   theta:0   state:0
result 18:   frame_id: 306    score: 0.00855925   x:0   y:0   theta:0   state:0
result 19:   frame_id: 263    score: 0.008411   x:0   y:0   theta:0   state:0
not find valid similar frame!
not find valid similar frame!
not find valid similar frame!
capture.Enable_ = true 
dd: 0.1   th: 0.0404647
 callback cvbrige
 callback cvbrige enable
[ INFO] [1510385717.712203937]: Saving images
[ INFO] [1510385717.712241481]: db info 1
[ INFO] [1510385717.712529265]: db info 2
RGB 869
depth 865
extract keypoints cost time: 0.007729
extract computeDescriptors cost time: 0.009066
image retrieve done.
searching for image_query  returns 20 results
result 0   frame_id: 78   score: 0.0154006
good matches: 41
match cost time: 0.001907
pnp inliers: 6
reject because inlier is too small: 6
Twc
-0.145715 -0.259246 -0.954756  -17.1422
  -0.9893 0.0452251  0.138707   10.4914
0.00721972  0.964752 -0.263062   7.18452
        0         0         0         1


result
result 0:   frame_id: 78    score: 0.0154006   x:-17.1422   y:7.18452   theta:2.32098   state:0
result 1:   frame_id: 311    score: 0.0146339   x:0   y:0   theta:0   state:0
result 2:   frame_id: 83    score: 0.013738   x:0   y:0   theta:0   state:0
result 3:   frame_id: 71    score: 0.0124965   x:0   y:0   theta:0   state:0
result 4:   frame_id: 80    score: 0.0119216   x:0   y:0   theta:0   state:0
result 5:   frame_id: 309    score: 0.0116311   x:0   y:0   theta:0   state:0
result 6:   frame_id: 181    score: 0.0113639   x:0   y:0   theta:0   state:0
result 7:   frame_id: 79    score: 0.0113487   x:0   y:0   theta:0   state:0
result 8:   frame_id: 90    score: 0.0113384   x:0   y:0   theta:0   state:0
result 9:   frame_id: 476    score: 0.0104398   x:0   y:0   theta:0   state:0
result 10:   frame_id: 60    score: 0.0104369   x:0   y:0   theta:0   state:0
result 11:   frame_id: 87    score: 0.0102477   x:0   y:0   theta:0   state:0
result 12:   frame_id: 88    score: 0.0101154   x:0   y:0   theta:0   state:0
result 13:   frame_id: 270    score: 0.0100008   x:0   y:0   theta:0   state:0
result 14:   frame_id: 418    score: 0.00999971   x:0   y:0   theta:0   state:0
result 15:   frame_id: 86    score: 0.00985774   x:0   y:0   theta:0   state:0
result 16:   frame_id: 77    score: 0.00970771   x:0   y:0   theta:0   state:0
result 17:   frame_id: 56    score: 0.00896509   x:0   y:0   theta:0   state:0
result 18:   frame_id: 161    score: 0.00884421   x:0   y:0   theta:0   state:0
result 19:   frame_id: 180    score: 0.00882865   x:0   y:0   theta:0   state:0
not find valid similar frame!
not find valid similar frame!
not find valid similar frame!
capture.Enable_ = true 
dd: 0.100002   th: 0.0332339
 callback cvbrige
 callback cvbrige enable
[ INFO] [1510385718.350298493]: Saving images
[ INFO] [1510385718.350342948]: db info 1
[ INFO] [1510385718.350869483]: db info 2
RGB 888
depth 884
extract keypoints cost time: 0.008431
extract computeDescriptors cost time: 0.01151
image retrieve done.
searching for image_query  returns 20 results
result 0   frame_id: 80   score: 0.0167461
good matches: 55
match cost time: 0.002166
pnp inliers: 7
reject because inlier is too small: 7
Twc
  0.670175 -0.0382401  -0.741218 -0.0447638
 0.0505473  -0.994002   0.096984   0.304159
  -0.74048  -0.102463  -0.664222    12.7372
         0          0          0          1


result
result 0:   frame_id: 80    score: 0.0167461   x:-0.0447638   y:12.7372   theta:3.03222   state:0
result 1:   frame_id: 90    score: 0.0146362   x:0   y:0   theta:0   state:0
result 2:   frame_id: 92    score: 0.013595   x:0   y:0   theta:0   state:0
result 3:   frame_id: 271    score: 0.012827   x:0   y:0   theta:0   state:0
result 4:   frame_id: 59    score: 0.0127138   x:0   y:0   theta:0   state:0
result 5:   frame_id: 171    score: 0.011153   x:0   y:0   theta:0   state:0
result 6:   frame_id: 63    score: 0.0108542   x:0   y:0   theta:0   state:0
result 7:   frame_id: 10    score: 0.0107177   x:0   y:0   theta:0   state:0
result 8:   frame_id: 306    score: 0.0105126   x:0   y:0   theta:0   state:0
result 9:   frame_id: 194    score: 0.010217   x:0   y:0   theta:0   state:0
result 10:   frame_id: 190    score: 0.0100956   x:0   y:0   theta:0   state:0
result 11:   frame_id: 48    score: 0.0100053   x:0   y:0   theta:0   state:0
result 12:   frame_id: 87    score: 0.00970492   x:0   y:0   theta:0   state:0
result 13:   frame_id: 88    score: 0.00969473   x:0   y:0   theta:0   state:0
result 14:   frame_id: 161    score: 0.00953108   x:0   y:0   theta:0   state:0
result 15:   frame_id: 131    score: 0.00934452   x:0   y:0   theta:0   state:0
result 16:   frame_id: 264    score: 0.00929462   x:0   y:0   theta:0   state:0
result 17:   frame_id: 152    score: 0.00926462   x:0   y:0   theta:0   state:0
result 18:   frame_id: 174    score: 0.00925376   x:0   y:0   theta:0   state:0
result 19:   frame_id: 261    score: 0.00922262   x:0   y:0   theta:0   state:0
not find valid similar frame!
not find valid similar frame!
not find valid similar frame!
capture.Enable_ = true 
dd: 0.1   th: 0.0576728
 callback cvbrige
 callback cvbrige enable
[ INFO] [1510385719.048344717]: Saving images
[ INFO] [1510385719.048415229]: db info 1
[ INFO] [1510385719.049199418]: db info 2
RGB 904
depth 905
extract keypoints cost time: 0.009076
extract computeDescriptors cost time: 0.01012
image retrieve done.
searching for image_query  returns 20 results
result 0   frame_id: 89   score: 0.020519
good matches too small: 7
Twc
1 0 0 0
0 1 0 0
0 0 1 0
0 0 0 1


result 1   frame_id: 83   score: 0.0182135
good matches: 52
match cost time: 0.002675
pnp inliers: 16
reject because motion is too large: 18.1622
Twc
-0.769973 -0.607369 -0.195565  0.291379
-0.0620843 -0.233725  0.970319   13.8535
 -0.63505   0.75926  0.142254   17.3843
        0         0         0         1


result
result 0:   frame_id: 89    score: 0.020519   x:0   y:0   theta:0   state:0
result 1:   frame_id: 83    score: 0.0182135   x:0.291379   y:17.3843   theta:2.76718   state:0
result 2:   frame_id: 306    score: 0.0130594   x:0   y:0   theta:0   state:0
result 3:   frame_id: 271    score: 0.0123793   x:0   y:0   theta:0   state:0
result 4:   frame_id: 84    score: 0.0121625   x:0   y:0   theta:0   state:0
result 5:   frame_id: 181    score: 0.0119079   x:0   y:0   theta:0   state:0
result 6:   frame_id: 188    score: 0.0117324   x:0   y:0   theta:0   state:0
result 7:   frame_id: 80    score: 0.0114695   x:0   y:0   theta:0   state:0
result 8:   frame_id: 79    score: 0.0113174   x:0   y:0   theta:0   state:0
result 9:   frame_id: 310    score: 0.0113067   x:0   y:0   theta:0   state:0
result 10:   frame_id: 302    score: 0.0108206   x:0   y:0   theta:0   state:0
result 11:   frame_id: 267    score: 0.0105449   x:0   y:0   theta:0   state:0
result 12:   frame_id: 98    score: 0.0103138   x:0   y:0   theta:0   state:0
result 13:   frame_id: 49    score: 0.0102841   x:0   y:0   theta:0   state:0
result 14:   frame_id: 88    score: 0.0100564   x:0   y:0   theta:0   state:0
result 15:   frame_id: 189    score: 0.0100305   x:0   y:0   theta:0   state:0
result 16:   frame_id: 82    score: 0.0098194   x:0   y:0   theta:0   state:0
result 17:   frame_id: 180    score: 0.00959533   x:0   y:0   theta:0   state:0
result 18:   frame_id: 90    score: 0.00952972   x:0   y:0   theta:0   state:0
result 19:   frame_id: 322    score: 0.00911411   x:0   y:0   theta:0   state:0
not find valid similar frame!
not find valid similar frame!
not find valid similar frame!
 callback cvbrige
 callback cvbrige
 callback cvbrige
 callback cvbrige
 callback cvbrige
 callback cvbrige
 callback cvbrige
 callback cvbrige
 callback cvbrige
 callback cvbrige
 callback cvbrige
 callback cvbrige
 callback cvbrige
 callback cvbrige
capture.Enable_ = true 
dd: 0.1   th: 0.0324488
 callback cvbrige
 callback cvbrige enable
[ INFO] [1510385728.092935695]: Saving images
[ INFO] [1510385728.092991193]: db info 1
[ INFO] [1510385728.093314284]: db info 2
RGB 1176
depth 1172
capture.Enable_ = true 
dd: 0.101549   th: 0.00433585
extract keypoints cost time: 0.008915
extract computeDescriptors cost time: 0.008785
image retrieve done.
searching for image_query  returns 20 results
result
result 0:   frame_id: 318    score: 0.0105451   x:0   y:0   theta:0   state:0
result 1:   frame_id: 184    score: 0.0102579   x:0   y:0   theta:0   state:0
result 2:   frame_id: 352    score: 0.0099901   x:0   y:0   theta:0   state:0
result 3:   frame_id: 82    score: 0.00977907   x:0   y:0   theta:0   state:0
result 4:   frame_id: 177    score: 0.00966637   x:0   y:0   theta:0   state:0
result 5:   frame_id: 469    score: 0.00936211   x:0   y:0   theta:0   state:0
result 6:   frame_id: 182    score: 0.00929538   x:0   y:0   theta:0   state:0
result 7:   frame_id: 91    score: 0.00918974   x:0   y:0   theta:0   state:0
result 8:   frame_id: 166    score: 0.0090919   x:0   y:0   theta:0   state:0
result 9:   frame_id: 408    score: 0.00899394   x:0   y:0   theta:0   state:0
result 10:   frame_id: 314    score: 0.00856055   x:0   y:0   theta:0   state:0
result 11:   frame_id: 79    score: 0.00846723   x:0   y:0   theta:0   state:0
result 12:   frame_id: 95    score: 0.00836999   x:0   y:0   theta:0   state:0
result 13:   frame_id: 86    score: 0.00834773   x:0   y:0   theta:0   state:0
result 14:   frame_id: 85    score: 0.00792248   x:0   y:0   theta:0   state:0
result 15:   frame_id: 180    score: 0.00771737   x:0   y:0   theta:0   state:0
result 16:   frame_id: 311    score: 0.00764848   x:0   y:0   theta:0   state:0
result 17:   frame_id: 375    score: 0.00750412   x:0   y:0   theta:0   state:0
result 18:   frame_id: 471    score: 0.00745729   x:0   y:0   theta:0   state:0
result 19:   frame_id: 44    score: 0.00733968   x:0   y:0   theta:0   state:0
not find valid similar frame!
not find valid similar frame!
not find valid similar frame!
capture.Enable_ = true 
dd: 0.1   th: 0.00448991
 callback cvbrige
 callback cvbrige enable
[ INFO] [1510385729.095934757]: Saving images
[ INFO] [1510385729.095998368]: db info 1
[ INFO] [1510385729.096315835]: db info 2
RGB 1200
depth 1197
extract keypoints cost time: 0.007772
extract computeDescriptors cost time: 0.009179
image retrieve done.
searching for image_query  returns 20 results
result 0   frame_id: 180   score: 0.0196897
good matches: 161
match cost time: 0.009579
pnp inliers: 0
virtual bool g2o::SparseOptimizer::initializeOptimization(g2o::HyperGraph::VertexSet&, int): Attempt to initialize an empty graph
virtual int g2o::SparseOptimizer::optimize(int, bool): 0 vertices to optimize, maybe forgot to call initializeOptimization()
reject because inlier is too small: 0
Twc
 0.187312   0.91312  0.362115  0.539013
-0.0519328  0.377329 -0.924622   2.11942
-0.980927  0.154387  0.118099   2.17514
        0         0         0         1


result
result 0:   frame_id: 180    score: 0.0196897   x:0.539013   y:2.17514   theta:1.7301   state:0
result 1:   frame_id: 98    score: 0.0133674   x:0   y:0   theta:0   state:0
result 2:   frame_id: 182    score: 0.0131463   x:0   y:0   theta:0   state:0
result 3:   frame_id: 56    score: 0.0121595   x:0   y:0   theta:0   state:0
result 4:   frame_id: 53    score: 0.0115496   x:0   y:0   theta:0   state:0
result 5:   frame_id: 93    score: 0.0112798   x:0   y:0   theta:0   state:0
result 6:   frame_id: 79    score: 0.0110812   x:0   y:0   theta:0   state:0
result 7:   frame_id: 30    score: 0.0105588   x:0   y:0   theta:0   state:0
result 8:   frame_id: 78    score: 0.0104111   x:0   y:0   theta:0   state:0
result 9:   frame_id: 54    score: 0.0102997   x:0   y:0   theta:0   state:0
result 10:   frame_id: 80    score: 0.0101203   x:0   y:0   theta:0   state:0
result 11:   frame_id: 70    score: 0.0100012   x:0   y:0   theta:0   state:0
result 12:   frame_id: 270    score: 0.00979979   x:0   y:0   theta:0   state:0
result 13:   frame_id: 68    score: 0.00968567   x:0   y:0   theta:0   state:0
result 14:   frame_id: 95    score: 0.00948151   x:0   y:0   theta:0   state:0
result 15:   frame_id: 396    score: 0.00882465   x:0   y:0   theta:0   state:0
result 16:   frame_id: 90    score: 0.00864319   x:0   y:0   theta:0   state:0
result 17:   frame_id: 58    score: 0.00856776   x:0   y:0   theta:0   state:0
result 18:   frame_id: 81    score: 0.00856159   x:0   y:0   theta:0   state:0
result 19:   frame_id: 390    score: 0.00839844   x:0   y:0   theta:0   state:0
not find valid similar frame!
not find valid similar frame!
not find valid similar frame!
capture.Enable_ = true 
dd: 0.1   th: 0.00447663
capture.Enable_ = true 
dd: 0.1   th: 0.0064708
 callback cvbrige
 callback cvbrige enable
[ INFO] [1510385729.941705840]: Saving images
[ INFO] [1510385729.941761827]: db info 1
[ INFO] [1510385729.943018659]: db info 2
RGB 1220
depth 1212
extract keypoints cost time: 0.009772
extract computeDescriptors cost time: 0.008608
image retrieve done.
searching for image_query  returns 20 results
result 0   frame_id: 80   score: 0.0180632
good matches: 43
match cost time: 0.002488
pnp inliers: 0
virtual bool g2o::SparseOptimizer::initializeOptimization(g2o::HyperGraph::VertexSet&, int): Attempt to initialize an empty graph
virtual int g2o::SparseOptimizer::optimize(int, bool): 0 vertices to optimize, maybe forgot to call initializeOptimization()
reject because inlier is too small: 0
Twc
 0.573448 -0.724177  0.383046  -16.4932
 -0.48773 -0.677455 -0.550612   17.8061
 0.658238  0.128924 -0.741689   24.8027
        0         0         0         1


result 1   frame_id: 90   score: 0.0174249
good matches: 33
match cost time: 0.005438
pnp inliers: 21
reject because motion is too large: 13.1348
Twc
  0.999837 0.00692788  0.0166607    6.94628
-0.0120763    0.94299   0.332603   -7.18866
-0.0134066   -0.33275    0.94292   -1.90916
         0          0          0          1


result
result 0:   frame_id: 80    score: 0.0180632   x:-16.4932   y:24.8027   theta:2.74621   state:0
result 1:   frame_id: 90    score: 0.0174249   x:6.94628   y:-1.90916   theta:0.339644   state:0
result 2:   frame_id: 265    score: 0.0135825   x:0   y:0   theta:0   state:0
result 3:   frame_id: 82    score: 0.0125849   x:0   y:0   theta:0   state:0
result 4:   frame_id: 88    score: 0.0124164   x:0   y:0   theta:0   state:0
result 5:   frame_id: 56    score: 0.0121738   x:0   y:0   theta:0   state:0
result 6:   frame_id: 89    score: 0.0120673   x:0   y:0   theta:0   state:0
result 7:   frame_id: 194    score: 0.0115236   x:0   y:0   theta:0   state:0
result 8:   frame_id: 91    score: 0.0109161   x:0   y:0   theta:0   state:0
result 9:   frame_id: 373    score: 0.0105984   x:0   y:0   theta:0   state:0
result 10:   frame_id: 376    score: 0.0104388   x:0   y:0   theta:0   state:0
result 11:   frame_id: 86    score: 0.0101279   x:0   y:0   theta:0   state:0
result 12:   frame_id: 269    score: 0.00989783   x:0   y:0   theta:0   state:0
result 13:   frame_id: 83    score: 0.00983888   x:0   y:0   theta:0   state:0
result 14:   frame_id: 85    score: 0.00965278   x:0   y:0   theta:0   state:0
result 15:   frame_id: 37    score: 0.00950319   x:0   y:0   theta:0   state:0
result 16:   frame_id: 336    score: 0.00939954   x:0   y:0   theta:0   state:0
result 17:   frame_id: 262    score: 0.00925008   x:0   y:0   theta:0   state:0
result 18:   frame_id: 72    score: 0.00922606   x:0   y:0   theta:0   state:0
result 19:   frame_id: 372    score: 0.00919487   x:0   y:0   theta:0   state:0
not find valid similar frame!
not find valid similar frame!
not find valid similar frame!
capture.Enable_ = true 
dd: 0.0924169   th: 0.0872665
 callback cvbrige
 callback cvbrige enable
[ INFO] [1510385731.738759608]: Saving images
[ INFO] [1510385731.738793916]: db info 1
[ INFO] [1510385731.739085835]: db info 2
RGB 1249
depth 1243
extract keypoints cost time: 0.008002
extract computeDescriptors cost time: 0.007876
image retrieve done.
searching for image_query  returns 20 results
result 0   frame_id: 90   score: 0.0244953
good matches: 13
match cost time: 0.007043
pnp inliers: 9
reject because inlier is too small: 9
Twc
-0.772111 -0.625402 -0.112772   5.17703
-0.635411  0.757004   0.15231   2.11759
-0.00988591  0.189257 -0.981878   1.57008
        0         0         0         1


result 1   frame_id: 91   score: 0.0195662
good matches too small: 8
Twc
1 0 0 0
0 1 0 0
0 0 1 0
0 0 0 1


result 2   frame_id: 94   score: 0.0182191
Segmentation fault (core dumped)











*/















