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

//void ImuTriggerCallback(const sensor_msgs::TimeReference::ConstPtr& msg);
void publishImasge(FlyCapture2::Camera &camera,
                  sensor_msgs::ImagePtr &msg_image,
                  image_transport::Publisher &pub_image,
                  Image &rawImage,
                  cv::Mat &image,
                  ros::Time &nowRosTime);


Image rawImage0;
Image rawImage1;
cv::Mat image0;
cv::Mat image1;
sensor_msgs::ImagePtr msg_image0;
sensor_msgs::ImagePtr msg_image1;
std::string frame_id = "camera";
image_transport::Publisher pub_imageLeft;  // cam0
image_transport::Publisher pub_imageRight; // cam1

PGRGuid pgid0;									 // PointsGray cam0
PGRGuid pgid1;									 // PointsGray cam1
BusManager busMgr;								 // PointsGray
FlyCapture2::Error error;
FlyCapture2::Camera camera0;
FlyCapture2::Camera camera1;
FlyCapture2::CameraInfo camInfo0;
FlyCapture2::CameraInfo camInfo1;

int CAM1_serialNumber = 16400089;
int CAM2_serialNumber = 16400120;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointgrey_stereo_js_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);    
    pub_imageLeft = it.advertise("/cam0/image_raw", 1);
    pub_imageRight = it.advertise("/cam1/image_raw", 1);

    // Get a guid using serial number
	busMgr.GetCameraFromSerialNumber(CAM1_serialNumber, &pgid0);  // PointsGray ȸ�� �Լ�, ī�޶� �ø��� �´� ���׽�Ʈ ���� 
	busMgr.GetCameraFromSerialNumber(CAM2_serialNumber, &pgid1);  // PointsGray ȸ�� �Լ�, ī�޶� �ø��� �´� ���׽�Ʈ ���� 

    // Connect the camera(Left)
    error = camera0.Connect(&pgid0);
    if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to connect to camera Left(0)" << std::endl;     
        return false;
    }

    // Connect the camera(Right)
    error = camera1.Connect(&pgid1);
    if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to connect to camera Right(1)" << std::endl;     
        return false;
    }

    // Get the camera info and print it out
    error = camera0.GetCameraInfo( &camInfo0 );
    if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to get camera info from camera0 (Left)" << std::endl;     
        return false;
    }
    std::cout << camInfo0.vendorName << " "
              << camInfo0.modelName << " " 
              << camInfo0.serialNumber << std::endl;

    // Get the camera info and print it out
    error = camera1.GetCameraInfo( &camInfo1 );
    if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to get camera info from camera1 (Right)" << std::endl;     
        return false;
    }
    std::cout << camInfo1.vendorName << " "
              << camInfo1.modelName << " " 
              << camInfo1.serialNumber << std::endl;
	
    error = camera0.StartCapture();
    if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
    {
        std::cout << "Bandwidth exceeded - LeftCam(0)" << std::endl;     
        return false;
    }
    else if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to start image capture - LeftCam(0)" << std::endl;     
        return false;
    }

    error = camera1.StartCapture();
    if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
    {
        std::cout << "Bandwidth exceeded - RightCam(1)" << std::endl;     
        return false;
    }
    else if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to start image capture - RightCam(1)" << std::endl;     
        return false;
    }

    //
    ros::Rate loop_rate(100);
    ros::Time nowRosTime = ros::Time::now();
    while(ros::ok())
    {
        // void publishImasge(FlyCapture2::Camera &camera,
        //           sensor_msgs::ImagePtr &msg_image,
        //           image_transport::Publisher &pub_image,
        //           Image &rawImage,
        //           cv::Mat &image)
        nowRosTime = ros::Time::now();
        publishImasge(camera0, msg_image0, pub_imageLeft, rawImage0, image0, nowRosTime);
        publishImasge(camera1, msg_image1, pub_imageRight, rawImage1, image1, nowRosTime);
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //ros::Subscriber sub = nh.subscribe("/imu/trigger_time", 1, ImuTriggerCallback);
    

    error = camera0.StopCapture();
    if ( error != PGRERROR_OK )
    {
        // This may fail when the camera was removed, so don't show 
        // an error message
    }  

    error = camera1.StopCapture();
    if ( error != PGRERROR_OK )
    {
        // This may fail when the camera was removed, so don't show 
        // an error message
    }

    camera0.Disconnect();
    camera1.Disconnect();

    return 0;
}

void publishImasge(FlyCapture2::Camera &camera,
                  sensor_msgs::ImagePtr &msg_image,
                  image_transport::Publisher &pub_image,
                  Image &rawImage,
                  cv::Mat &image,
                  ros::Time &nowRosTime)
{
    // Get the image    
    Error error = camera.RetrieveBuffer( &rawImage );
    if ( error != PGRERROR_OK )
    {
        std::cout << "capture error" << std::endl;            
    }else
    {
        unsigned int rowBytes = (double)rawImage.GetReceivedDataSize()/(double)rawImage.GetRows();         
        image = cv::Mat(rawImage.GetRows(), rawImage.GetCols(), CV_8UC3, rawImage.GetData(),rowBytes);      
        msg_image = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image).toImageMsg(); // "rgb8", "rgba8", "bgr8", "bgra8", "mono8", "mono16"
        msg_image->header.frame_id = frame_id;
        //msg_image->header.stamp = msg->time_ref;
        msg_image->header.stamp = nowRosTime;
        pub_image.publish(msg_image);      
    }
}


// void ImuTriggerCallback(const sensor_msgs::TimeReference::ConstPtr& msg)
// {    
//     //cv_bridge::CvImageConstPtr ptr;
//     //std::cerr << "ImuTriggerCallback" << std::endl;

//     // Get the image    
//     Error error = camera.RetrieveBuffer( &rawImage );
//     if ( error != PGRERROR_OK )
//     {
//         std::cout << "capture error" << std::endl;            
//     }else
//     {
//         unsigned int rowBytes = (double)rawImage.GetReceivedDataSize()/(double)rawImage.GetRows();         
//         image = cv::Mat(rawImage.GetRows(), rawImage.GetCols(), CV_8UC1, rawImage.GetData(),rowBytes);      
//         msg_image = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg(); // "rgb8", "rgba8", "bgr8", "bgra8", "mono8", "mono16"
//         msg_image->header.frame_id = frame_id;
//         msg_image->header.stamp = msg->time_ref;        
//         pub_imageLeft.publish(msg_image);      
//     }
// }