#include "flycapture/FlyCapture2.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/TimeReference.h"
#include "sensor_msgs/Image.h"
#include "pointgrey_js/parameters.h"

#include <iostream>

using namespace FlyCapture2;

void ImuTriggerCallback(const sensor_msgs::TimeReference::ConstPtr& msg);


PGRGuid pgid;									 // PointsGray cam
BusManager busMgr;								 // PointsGray
FlyCapture2::Error error;
FlyCapture2::Camera camera;
FlyCapture2::CameraInfo camInfo;
Image rawImage;
cv::Mat image;
cv::Mat imagebgr;
sensor_msgs::ImagePtr msg_image;
sensor_msgs::ImagePtr msg_imagebgr;
std::string frame_id = "camera";
image_transport::Publisher pub_image;
image_transport::Publisher pub_imagebgr;

int CAM_serialNumber = -1;
bool getColorData = false;
double imageResizeScale = 1.0;
std::string RawImageType = "mono8";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointgrey_right_rectify_js_node");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);    

    nh.getParam("RightCamID", CAM_serialNumber);
    nh.getParam("GetColorData", getColorData);
    nh.getParam("ResizeScale", imageResizeScale);
    nh.getParam("RawImageType", RawImageType);

    readStereoParameters("/home/junesuk/workspace/tool/cam_imu_capture/catkin_ws/src/pointgrey_js/config/stereo_parking_crvlResize.yaml");

    pub_image = it.advertise("/cam1/image_raw", 1);
    //pub_image = it.advertise("/camera/right/image_raw", 1);
    

    if(getColorData)
        pub_imagebgr = it.advertise("/cam1/imagebgr_raw", 1);

    

    std::cout << "[Right]CAM_serialNumber: " << CAM_serialNumber << std::endl;

    // Get a guid using serial number
	busMgr.GetCameraFromSerialNumber(CAM_serialNumber, &pgid);  // PointsGray ȸ�� �Լ�, ī�޶� �ø��� �´� ���׽�Ʈ ���� 

    // Connect the camera
    error = camera.Connect( &pgid );
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
        
        if(RawImageType == "rgb8")
        {
            imagebgr = cv::Mat(rawImage.GetRows(), rawImage.GetCols(), CV_8UC3, rawImage.GetData(),rowBytes);
        }else if(RawImageType == "mono8")
        {
            imagebgr = cv::Mat(rawImage.GetRows(), rawImage.GetCols(), CV_8UC1, rawImage.GetData(),rowBytes);
        }
    
        if(imageResizeScale < 1.0)
        {
            cv::Mat resizeTemp;
            const int resizeHeight = imagebgr.rows*imageResizeScale;
            const int resizeWidth  = imagebgr.cols*imageResizeScale;
            cv::resize(imagebgr, resizeTemp, cv::Size(resizeHeight,resizeWidth), cv::INTER_AREA);
            //std::cout << "[Resize] row, col : " << resizeHeight << ", " << resizeWidth << std::endl;
            imagebgr = resizeTemp.clone();
        }

        if(RawImageType == "rgb8")
        {
            cv::cvtColor(imagebgr, image, cv::COLOR_BGR2GRAY);
        }else if(RawImageType == "mono8")
        {
            image = imagebgr;
        }

        // Rectification
        cv::Mat image_rectify;
        cv::remap(image, image_rectify, RectifyMap_cam1_map1, RectifyMap_cam1_map2, cv::INTER_LINEAR);
        cv::imshow("[Right] image", image);
        cv::imshow("[Right] image_rectify", image_rectify);
        cv::waitKey(1);
        
        // Only Gray
        msg_image = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_rectify).toImageMsg(); // "rgb8", "rgba8", "bgr8", "bgra8", "mono8", "mono16"
        msg_image->header.frame_id = frame_id;
        msg_image->header.stamp = msg->time_ref;        
        //pub_image.publish(msg_image);
        

        // + Color        
        if(getColorData)
        {
            msg_imagebgr = cv_bridge::CvImage(std_msgs::Header(), "rgb8", imagebgr).toImageMsg(); // "rgb8", "rgba8", "bgr8", "bgra8", "mono8", "mono16"        
            msg_imagebgr->header.frame_id = frame_id;
            msg_imagebgr->header.stamp = msg->time_ref;
        }
        
        
        pub_image.publish(msg_image);
        
        if(getColorData)
            pub_imagebgr.publish(msg_imagebgr);
    }
}