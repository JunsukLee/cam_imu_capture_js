#include "ros/ros.h"
#include "sensor_msgs/TimeReference.h"

#include <sstream>

int main(int argc, char **argv)
{  
  ros::init(argc, argv, "generator_trigger_js_node");

  ros::NodeHandle n("~");
  
  double device_hz;

  n.getParam("device_hz", device_hz);

  std::cout << "device_hz: " << device_hz << std::endl;

  ros::Publisher triggerTime_pub = n.advertise<sensor_msgs::TimeReference>("/imu/trigger_time", 1);

  ros::Rate loop_rate(device_hz);

  while (ros::ok())
  {
    sensor_msgs::TimeReference msg;

    ros::Time nowRosTime = ros::Time::now();

    msg.header.frame_id = "frame_id";
    msg.time_ref = nowRosTime;
   
    triggerTime_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}


