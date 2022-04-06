cmake_minimum_required(VERSION 3.0.2)
project(pointgrey_js)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_library(POINTGREY_LIB NAMES libflycapture.so.2 flycapture)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(shared_lib
    src/parameters.cpp
    )

###################################################################################################
########### Tigger generator  #####################################################################
###################################################################################################
add_executable(generator_trigger_js_node src/generator_trigger_js_node.cpp)
target_link_libraries(generator_trigger_js_node ${catkin_LIBRARIES} ${POINTGREY_LIB} ${OpenCV_LIBS})


###################################################################################################
########### C  A  M  ##############################################################################
###################################################################################################
add_executable(pointgrey_js_node src/pointgrey_js_node.cpp)
target_link_libraries(pointgrey_js_node ${catkin_LIBRARIES} ${POINTGREY_LIB} ${OpenCV_LIBS})

# stereo left-cam
add_executable(pointgrey_left_js_node src/pointgrey_left_js_node.cpp)
target_link_libraries(pointgrey_left_js_node ${catkin_LIBRARIES} ${POINTGREY_LIB} ${OpenCV_LIBS})

# stereo left-cam-rectify
add_executable(pointgrey_left_rectify_js_node src/pointgrey_left_rectify_js_node.cpp)
target_link_libraries(pointgrey_left_rectify_js_node ${catkin_LIBRARIES} ${POINTGREY_LIB} ${OpenCV_LIBS} shared_lib)

# stereo right-cam
add_executable(pointgrey_right_js_node src/pointgrey_right_js_node.cpp)
target_link_libraries(pointgrey_right_js_node ${catkin_LIBRARIES} ${POINTGREY_LIB} ${OpenCV_LIBS})

# stereo right-cam
add_executable(pointgrey_right_rectify_js_node src/pointgrey_right_rectify_js_node.cpp)
target_link_libraries(pointgrey_right_rectify_js_node ${catkin_LIBRARIES} ${POINTGREY_LIB} ${OpenCV_LIBS} shared_lib)

add_executable(pointgrey_stereo_js_node src/pointgrey_stereo_js_node.cpp)
target_link_libraries(pointgrey_stereo_js_node ${catkin_LIBRARIES} ${POINTGREY_LIB} ${OpenCV_LIBS})


###################################################################################################
########### RPLidar  ##############################################################################
###################################################################################################
add_executable(rplidar_s2_trigger_js_node src/rplidar_s2_trigger_js.cpp)
target_link_libraries(rplidar_s2_trigger_js_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

