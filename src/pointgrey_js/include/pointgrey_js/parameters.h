#include <iostream>
#include <fstream>
#include <map>
#include <opencv2/opencv.hpp>

extern cv::Mat RectifyMap_cam0_map1, RectifyMap_cam0_map2;
extern cv::Mat RectifyMap_cam1_map1, RectifyMap_cam1_map2; 

int readStereoParameters(std::string config_file);