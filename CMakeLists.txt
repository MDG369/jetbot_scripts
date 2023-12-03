add_compile_options(-std=c++14)
cmake_minimum_required(VERSION 2.8)

set(TARGET_NAME myapp)
project(${TARGET_NAME})
find_package(depthai)
find_package(Threads REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(realsense2)
find_package(PCL REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  cv_bridge
  sensor_msgs
  tf2_ros
  tf2
)

include_directories(
/home/jetbot/VSLAM-Object-Search
/home/jetbot/VSLAM-Object-Search/include
/home/jetbot/VSLAM-Object-Search/include/CameraModels
/home/jetbot/VSLAM-Object-Search/Thirdparty/Sophus
/home/jetbot/depthai-core/build
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
)


SET(OpenCV_DIR )
find_package(OpenCV 4.8)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 3 not found.")
   endif()
find_package(depthai)
find_package(Threads REQUIRED)
include_directories(
/home/jetbot/VSLAM-Object-Search
/home/jetbot/VSLAM-Object-Search/include
/home/jetbot/VSLAM-Object-Search/include/CameraModels
/home/jetbot/VSLAM-Object-Search/Thirdparty/Sophus
/home/jetbot/depthai-core/build
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}

)


add_library(depthai SHARED IMPORTED)
set_property(TARGET depthai PROPERTY IMPORTED_LOCATION /home/jetbot/depthai-core/build/libdepthai-opencv.so)



#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY /home/jetbot/VSLAM-Object-Search/Examples/Stereo/ros-st)
#add_executable(publisher_node ROS-Stereo/ros_stereo_pub.cc)
#target_link_libraries(publisher_node ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${EIGEN3_LIBS} ${catkin_LIBRARIES} /home/jetbot/VSLAM-Object-Search/Thirdparty/DBoW2/lib/libDBoW2.so -lboost_serialization
#-lcrypto
# depthai
#/home/jetbot/VSLAM-Object-Search/Thirdparty/g2o/lib/libg2o.so
#/home/jetbot/VSLAM-Object-Search/lib/libORB_SLAM3.so
#depthai::opencv)


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY /home/jetbot/VSLAM-Object-Search/Examples/Stereo/ros-st)
add_executable(publisher_video_node ROS-Stereo/ros_stereo_video_pub.cc)
target_link_libraries(publisher_video_node ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${EIGEN3_LIBS} ${catkin_LIBRARIES} /home/jetbot/VSLAM-Object-Search/Thirdparty/DBoW2/lib/libDBoW2.so -lboost_serialization
-lcrypto
 depthai

/home/jetbot/VSLAM-Object-Search/Thirdparty/g2o/lib/libg2o.so
/home/jetbot/VSLAM-Object-Search/lib/libORB_SLAM3.so
depthai::opencv)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY /home/jetbot/VSLAM-Object-Search/Examples/Stereo/ros-st)
#add_executable(subscriber_node ROS-Stereo/ros_stereo_sub.cc)
#target_link_libraries(subscriber_node ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${EIGEN3_LIBS} ${catkin_LIBRARIES} /home/jetbot/VSLAM-Object-Search/Thirdparty/DBoW2/lib/libDBoW2.so  -lboost_serialization
#-lcrypto
# depthai
#/home/jetbot/VSLAM-Object-Search/Thirdparty/g2o/lib/libg2o.so
#depthai::opencv)
