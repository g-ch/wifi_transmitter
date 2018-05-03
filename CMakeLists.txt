cmake_minimum_required(VERSION 2.8.3)
project(socket)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs  
  cv_bridge  
  image_transport)


catkin_package(
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})


include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(socket_client src/socket_client.cpp)
target_link_libraries(socket_client ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(socket_sever src/socket_sever.cpp)
target_link_libraries(socket_sever ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
