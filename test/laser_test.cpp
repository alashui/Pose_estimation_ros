#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/Twist.h>
class Stopper
{
  public:
    const  double FORWARD_SPEED_MPS = 0.5;
    const  double MIN_SCAN_ANGLE_RAD = -20.0/180*M_PI;
    const  double MAX_SCAN_ANGLE_RAD = +20.0/180*M_PI;
    const  float MIN_PROXIMITY_RANGE_M = 0.5;

    Stopper();
    void startMoving();

  private:
    ros::NodeHandle node;
    ros::Publisher commandPub;
    ros::Subscriber laserSub;
    bool keepMoving;
    void moveForward();
    void scanCallback( const sensor_msgs::LaserScan ::ConstPtr& scan);
};

Stopper::Stopper()
{
    keepMoving=true;
    commandPub=node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",10);
    laserSub=node.subscribe("scan",1,&Stopper::scanCallback,this);

}

void Stopper::moveForward()
{
    geometry_msgs::Twist msg;
    msg.linear.x=FORWARD_SPEED_MPS;
    commandPub.publish(msg);
}

void Stopper::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    int minIndex=ceil(( MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan-> angle_increment);
    int maxIndex = floor(( MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan-> angle_increment);
    float closestRange = scan->ranges[minIndex];
    for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++) 
    {
        if (scan->ranges[currIndex] < closestRange)
            closestRange = scan->ranges[currIndex];
    
    }
    ROS_INFO_STREAM("Closest range: " << closestRange);
    if (closestRange < MIN_PROXIMITY_RANGE_M) 
    {
        ROS_INFO("Stop!");
        keepMoving = false;
    }

}

void Stopper::startMoving()
{
    ros::Rate rate(10);
    ROS_INFO("start moving");
    while(ros::ok()&&keepMoving)
    {
        moveForward();
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"stopper");
    Stopper stopper;
    stopper.startMoving();
    return 0;
}