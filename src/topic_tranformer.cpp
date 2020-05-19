//
// Created by cc on 2019/12/14.
//

#include <ros/ros.h>
#include <wifi_transmitter/Display.h>
#include <wifi_transmitter/ObjectsInTracking.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <iostream>

using namespace std;

void dataCallback(const )

int main(int argc, char** argv){
    ros::init(argc, argv, "topic_transformer");
    ros::NodeHandle n;

    ros::Subscriber data_sub = n.subscribe("/Display/transfered",1,dataCallback);

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);


    ros::Rate loop_rate(10);

    while(ros::ok()){


        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

