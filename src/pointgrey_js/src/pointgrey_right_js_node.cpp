#include "flycapture/FlyCapture2.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/TimeReference.h"
#include "sensor_msgs/Image.h"

#include <iostream>
#include <queue>

using namespace FlyCapture2;

void ImuTriggerCallback(const sensor_msgs::TimeReference::ConstPtr& msg);
void capture_process();
void capture_pub();


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
unsigned int rowBytes;
bool init_flycapture2 = false;
bool init_start_capture = false;
static std::queue<cv::Mat> queue_img;
static std::queue<ros::Time> queue_imuTime;
std::mutex m_buf;
int resizeHeight;
int resizeWidth;

int CAM_serialNumber = -1;
bool getColorData = false;
double imageResizeScale = 1.0;
std::string RawImageType = "mono8";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointgrey_right_js_node");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);    

    nh.getParam("RightCamID", CAM_serialNumber);
    nh.getParam("GetColorData", getColorData);
    nh.getParam("ResizeScale", imageResizeScale);
    nh.getParam("RawImageType", RawImageType);

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
    std::thread sync_thread{capture_process};
    std::thread sync_thread2{capture_pub};
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
    // Only Gray
    m_buf.lock();
    queue_imuTime.push(msg->time_ref);    
    m_buf.unlock();
    if(init_start_capture==false)
    {
        init_start_capture = true;
    }
}

void capture_process()
{
    while(init_start_capture==false)
    {
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }

    while(1)
    {        
        error = camera.RetrieveBuffer( &rawImage );
        if ( error != PGRERROR_OK )
        {
            std::cout << "capture error" << std::endl;            
        }else
        {
            if(!init_flycapture2)
            {
                rowBytes = (double)rawImage.GetReceivedDataSize()/(double)rawImage.GetRows();
                resizeHeight = rawImage.GetRows()*imageResizeScale;
                resizeWidth  = rawImage.GetCols()*imageResizeScale;
                init_flycapture2 = true;
            }

            if(RawImageType == "mono8")
            {
                imagebgr = cv::Mat(rawImage.GetRows(), rawImage.GetCols(), CV_8UC1, rawImage.GetData(),rowBytes);
            }
            // else if(RawImageType == "rgb8")
            // {
            //     imagebgr = cv::Mat(rawImage.GetRows(), rawImage.GetCols(), CV_8UC3, rawImage.GetData(),rowBytes);
            // }

            if(imageResizeScale < 1.0)
            {
                cv::resize(imagebgr, imagebgr, cv::Size(resizeHeight,resizeWidth), cv::INTER_AREA);                
            }
            
            if(RawImageType == "mono8")
            {
                image = imagebgr;
            }
            // else if(RawImageType == "rgb8")
            // {
            //     cv::cvtColor(imagebgr, image, cv::COLOR_BGR2GRAY);
            // }
                    
            // Only Gray
            m_buf.lock();
            queue_img.push(image);
            m_buf.unlock();

            //msg_image = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg(); // "rgb8", "rgba8", "bgr8", "bgra8", "mono8", "mono16"
            //msg_image->header.frame_id = frame_id;
            //msg_image->header.stamp = msg->time_ref;
            //pub_image.publish(msg_image);
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

void capture_pub()
{
    while(1)
    {
        m_buf.lock();
        if((queue_img.size() > 0) && (queue_imuTime.size() > 0))
        {
            msg_image = cv_bridge::CvImage(std_msgs::Header(), "mono8", queue_img.front()).toImageMsg(); // "rgb8", "rgba8", "bgr8", "bgra8", "mono8", "mono16"
            msg_image->header.frame_id = frame_id;
            msg_image->header.stamp = queue_imuTime.front();
            pub_image.publish(msg_image);
            // std::cerr << "[right]queue_img.size(): " << queue_img.size() << std::endl;
            // std::cerr << "[right]queue_imuTime.size(): " << queue_imuTime.size() << std::endl;
            queue_img.pop();
            queue_imuTime.pop();
        }
        m_buf.unlock();
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}
