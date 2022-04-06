#include "flycapture/FlyCapture2.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/TimeReference.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include <mutex>

#include <iostream>

using namespace FlyCapture2;

void ImuTriggerCallback(const sensor_msgs::TimeReference::ConstPtr& msg);

void lidarTriggerCallback(const sensor_msgs::LaserScanPtr& msg);

sensor_msgs::LaserScan timeshare;
ros::Publisher scan_pub;
std::string frame_id = "camera";
std::mutex mu;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_s2_trigger_js_node");
    ros::NodeHandle nh("~");

    // pubish
    scan_pub = nh.advertise<sensor_msgs::LaserScan>("/trigger/scan", 1);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // subscribe
    ros::Subscriber subTriggerTime = nh.subscribe("/imu/trigger_time", 1, ImuTriggerCallback);
    ros::Subscriber subRPLidar = nh.subscribe("/scan", 1, lidarTriggerCallback);
    ros::spin();

    return 0;
}

void ImuTriggerCallback(const sensor_msgs::TimeReference::ConstPtr& msg)
{    
    mu.lock();
    std::cout << "TiggerTime: " << msg->time_ref << std::endl;    
    timeshare.header.stamp = msg->time_ref;
    mu.unlock();    
}

void lidarTriggerCallback(const sensor_msgs::LaserScanPtr& msg)
{    
    mu.lock();
    std::cout << "LidarTime: " << timeshare.header.stamp << ", " << msg->header.stamp << std::endl;    
    std::cout << "LidarTime: " << timeshare.header.stamp - msg->header.stamp << std::endl;    
    mu.unlock();
}