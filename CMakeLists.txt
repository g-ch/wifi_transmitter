cmake_minimum_required(VERSION 2.8.3)
project(wifi_transmitter)
add_definitions(-std=c++11 -g -O1 -ftree-vectorize -march=native)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  topic_tools
  geometry_msgs
  visualization_msgs
  message_generation
  roslz4
)

add_message_files(
        FILES
        Display.msg
        ObjectInTracking.msg
        ObjectsInTracking.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        visualization_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime topic_tools
)


find_package(OpenCV REQUIRED)

include_directories(
        include ${catkin_INCLUDE_DIRS}
        # include
        ${OpenCV_INCLUDE_DIRS}
)

include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(multi_thread_client_gg src/multi_thread_client_gg.cpp src/MultiThreadSocket.cpp src/MultiThreadSocket.h)
add_dependencies(multi_thread_client_gg        ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(multi_thread_client_gg ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(multi_thread_server_gg src/multi_thread_server_gg.cpp src/MultiThreadSocket.cpp src/MultiThreadSocket.h)
add_dependencies(multi_thread_server_gg        ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(multi_thread_server_gg ${catkin_LIBRARIES} ${OpenCV_LIBS})


