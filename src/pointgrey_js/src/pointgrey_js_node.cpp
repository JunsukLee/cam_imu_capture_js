#include "flycapture/FlyCapture2.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/TimeReference.h"
#include "sensor_msgs/Image.h"

#include <iostream>

using namespace FlyCapture2;

void ImuTriggerCallback(const sensor_msgs::TimeReference::ConstPtr& msg);

FlyCapture2::Error error;
FlyCapture2::Camera camera;
FlyCapture2::CameraInfo camInfo;
Image rawImage;
cv::Mat image;
sensor_msgs::ImagePtr msg_image;
std::string frame_id = "camera";
image_transport::Publisher pub_image;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointgrey_js_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);    
    pub_image = it.advertise("/cam0/image_raw", 1);

    // Connect the camera
    error = camera.Connect( 0 );
    if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to connect to camera" << std::endl;     
        return false;
    }

    // Get the camera info and print it out
    error = camera.GetCameraInfo( &camInfo );
    if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to get camera info from camera" << std::endl;     
        return false;
    }
    std::cout << camInfo.vendorName << " "
              << camInfo.modelName << " " 
              << camInfo.serialNumber << std::endl;
	
    error = camera.StartCapture();
    if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
    {
        std::cout << "Bandwidth exceeded" << std::endl;     
        return false;
    }
    else if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to start image capture" << std::endl;     
        return false;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ros::Subscriber sub = nh.subscribe("/imu/trigger_time", 1, ImuTriggerCallback);
    ros::spin();

    error = camera.StopCapture();
    if ( error != PGRERROR_OK )
    {
        // This may fail when the camera was removed, so don't show 
        // an error message
    }  

    camera.Disconnect();

    return 0;
}

void ImuTriggerCallback(const sensor_msgs::TimeReference::ConstPtr& msg)
{    
    //cv_bridge::CvImageConstPtr ptr;
    //std::cerr << "ImuTriggerCallback" << std::endl;

    // Get the image    
    Error error = camera.RetrieveBuffer( &rawImage );
    if ( error != PGRERROR_OK )
    {
        std::cout << "capture error" << std::endl;            
    }else
    {
        unsigned int rowBytes = (double)rawImage.GetReceivedDataSize()/(double)rawImage.GetRows();         
        image = cv::Mat(rawImage.GetRows(), rawImage.GetCols(), CV_8UC1, rawImage.GetData(),rowBytes);      
        msg_image = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg(); // "rgb8", "rgba8", "bgr8", "bgra8", "mono8", "mono16"
        msg_image->header.frame_id = frame_id;
        msg_image->header.stamp = msg->time_ref;        
        pub_image.publish(msg_image);      
    }
}