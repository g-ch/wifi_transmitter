#include "ros/ros.h"
#include "ros/message_traits.h"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include <unistd.h>
#include <sys/socket.h>
#include <thread>
#include "MultiThreadSocket.h"
#include <topic_tools/shape_shifter.h>
#include <lz4.h>
#include <std_msgs/Float64MultiArray.h>

//订阅消息include文件
#include <wifi_transmitter/Display.h>
#include <wifi_transmitter/ObjectsInTracking.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
//
// 打开一个摄像头,然后通过socket发送,有三个线程
// 1 用于打开摄像头并存到buffer
// 2 用于发送buffer
// 3 用于发送float 数据测试

/*
这里简单比较一下TCP和UDP在编程实现上的一些区别：

TCP流程
     建立一个TCP连接需要三次握手，而断开一个TCP则需要四个分节。当某个应用进程调用close(主动端)后
(可以是服务器端，也可以是客户 端)，这一端的TCP发送一个FIN，表示数据发送完毕；另一端(被动端)发送一
个确认，当被动端待处理的应用进程都处理完毕后，发送一个FIN到主动端，并关闭套接口，主动端接收到这个
FIN后再发送一个确认，到此为止这个TCP连接被断开。

UDP套接口
　　UDP套接口是无连接的、不可靠的数据报协议；既然他不可靠为什么还要用呢？
　　其一：当应用程序使用广播或多播是只能使用UDP协议；
　　其二：由于它是无连接的，所以速度快。因为UDP套接口是无连接的，如果一方的数据报丢失，那另一方将无
        限等待，解决办法是设置一个超时。在编写UDP套接口程序时，有几点要注意：建立套接口时socket函
        数的第二个参数应该是SOCK_DGRAM，说明是建立一个UDP套接口；由于UDP是无连接的，所以服务器端
        并不需要listen或accept函数；当UDP套接口调用connect函数时，内核只记录连接放的IP地址 和端
        口，并立即返回给调用进程.
*/

#define send_pcl_socket_port 12668
#define send_fdata_socket_port 12669
#define DisplayMsg wifi_transmitter::Display
#define SEND_INTERVAL 1

using namespace std;
using namespace cv;

void send_pcl_func();
void send_other_func();

// used for pcl data
int src_buffer_size;
int max_compressed_buffer_size;
int compressed_data_size;
std::vector<u_char> compressed_buffer;
bool pcl_cloud_ready = false;
bool pcl_message_received = false;

bool pcl_data_locked = false;

//used for other msg
int other_buffer_size;
int max_compressed_other_buffer_size;
int compressed_other_data_size;
std::vector<u_char> compressed_other_buffer;
bool other_ready = false;
bool other_message_received = false;

DisplayMsg display_msg;


MultiThreadSocket send_pcl_socket(send_pcl,send_pcl_socket_port);
MultiThreadSocket send_other_socket(send_float_data,send_fdata_socket_port);

void objs_msg_cb(const wifi_transmitter::ObjectsInTracking::ConstPtr &msg){
    display_msg.objects = *msg;
    other_message_received = true;
    // display_msg.objects.header = msg->header;
    // display_msg.objects.result = msg->result;
}

void point32_msg_cb(const geometry_msgs::Point32::ConstPtr& msg){
    display_msg.vel_info = *msg;
    other_message_received = true;
}

void marker_msg_cb(const visualization_msgs::Marker::ConstPtr& msg){
    display_msg.makers = *msg;
    other_message_received = true;
}

void pose_msg_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    display_msg.local_pose = msg->pose;
    other_message_received = true;
}

void sim_pose_msg_cb(const geometry_msgs::Pose::ConstPtr& msg){
    display_msg.local_pose.position.x = -msg->position.y;
    display_msg.local_pose.position.y = msg->position.x;
    display_msg.local_pose.position.z = msg->position.z;

    display_msg.local_pose.orientation.x = -msg->orientation.y;
    display_msg.local_pose.orientation.y = msg->orientation.x;
    display_msg.local_pose.orientation.z = msg->orientation.z;
    display_msg.local_pose.orientation.w = msg->orientation.w;
    other_message_received = true;
}

void detectionTimeCallback(const std_msgs::Float64& msg){
    display_msg.detection_time = msg;
}

void costHeadVelocityCallback(const std_msgs::Float64MultiArray& msg){
    display_msg.cost_head_velocity = msg;
}

void costHeadDirectionCallback(const std_msgs::Float64MultiArray& msg){
    display_msg.cost_head_direction = msg;
}

void costHeadObjectsCallback(const std_msgs::Float64MultiArray& msg){
    display_msg.cost_head_objects = msg;
}

void costHeadFluctuationCallback(const std_msgs::Float64MultiArray& msg){
    display_msg.cost_head_fluctuation = msg;
}

void costHeadFinalCallback(const std_msgs::Float64MultiArray& msg){
    display_msg.cost_head_final = msg;
}

void costHeadUpdateCallback(const std_msgs::Float64MultiArray& msg){
    display_msg.cost_head_update = msg;
}


void general_ros_message_callback(const topic_tools::ShapeShifter::ConstPtr& msg){

    if(!pcl_data_locked){
        pcl_data_locked = true;

        src_buffer_size = msg->size();
        std::vector<u_char> src_buffer;
        // copy raw memory into the buffer
        src_buffer.resize(src_buffer_size);
        ros::serialization::OStream stream(src_buffer.data(), src_buffer_size);
        msg->write(stream);
        max_compressed_buffer_size = LZ4_compressBound(src_buffer_size);
        compressed_buffer.resize(max_compressed_buffer_size);
        compressed_data_size = LZ4_compress_default(reinterpret_cast<const char*>(src_buffer.data()),
                                                              reinterpret_cast<char*>(compressed_buffer.data()),
                                                              src_buffer_size,
                                                              max_compressed_buffer_size);
        compressed_buffer.resize(compressed_data_size);

        pcl_message_received = true;
        // ROS_INFO_STREAM("MD5: "<<msg->getMD5Sum()<<" data type"<<msg->getDataType()<<" msg def size: "<< msg->getMessageDefinition().size());
        //ROS_INFO_STREAM("check size: "<<src_buffer_size<<"compressed size "<<compressed_data_size);
        pcl_data_locked = false;
    }
    
}

template <typename T>
void other_message_compress(const T& msg){
    other_ready = false;
    other_buffer_size = ros::serialization::serializationLength(msg);
    std::vector<u_char> src_other_buffer;
    // copy raw memory into the buffer
    src_other_buffer.resize(other_buffer_size);
    ros::serialization::OStream other_stream(src_other_buffer.data(), other_buffer_size);
    ros::serialization::serialize(other_stream, msg);
    max_compressed_other_buffer_size = LZ4_compressBound(other_buffer_size);
    compressed_other_buffer.resize(max_compressed_other_buffer_size);
    compressed_other_data_size = LZ4_compress_default(reinterpret_cast<const char*>(src_other_buffer.data()),
                                                          reinterpret_cast<char*>(compressed_other_buffer.data()),
                                                          other_buffer_size,
                                                          max_compressed_other_buffer_size);
    compressed_other_buffer.resize(compressed_other_data_size);
    other_ready = true;
    // ROS_INFO_STREAM("MD5: "<<ros::message_traits::MD5Sum<DisplayMsg>::value()<<"data type "<<ros::message_traits::DataType<DisplayMsg>::value());
    // ROS_INFO_STREAM("check size: "<<src_buffer_size<<"compressed size "<<compressed_data_size);
}


void timerCallback(const ros::TimerEvent& e){
    static int send_which_one = 0;
    if(send_which_one == 0){
        if(other_message_received){
            other_ready = true;
            if(pcl_message_received) {send_which_one = 1;}
        }
    }else{
        if(pcl_message_received){
            pcl_cloud_ready = true;
            if(other_message_received) {send_which_one = 0;}
        }
    }
}


/* @function main */
int main( int argc, char **argv )
{
    ros::init(argc, argv, "multi_thread_socket_server");
    ros::NodeHandle ros_nh;

    //直接通过订阅通用消息发送
    ros::Subscriber pcl_subscriber = ros_nh.subscribe("/ring_buffer/cloud_ob",1,general_ros_message_callback);

    //通过自己合成消息，直接发送
    ros::Subscriber objs_sub = ros_nh.subscribe("/mot/objects_in_tracking_predicted",1,objs_msg_cb);
    ros::Subscriber point32_sub = ros_nh.subscribe("/place_velocity_info_corrected",1,point32_msg_cb);
    ros::Subscriber marker_sub = ros_nh.subscribe("/visualization_marker",1,marker_msg_cb);
    ros::Subscriber pose_sub = ros_nh.subscribe("/mavros/local_position/pose",1,pose_msg_cb);  // For real world
    // ros::Subscriber pose_sub = ros_nh.subscribe("/iris/ground_truth/pose",1,sim_pose_msg_cb); // For simulation
    ros::Subscriber detection_time_sub = ros_nh.subscribe("/yolo_ros_real_pose/detection_time", 1, detectionTimeCallback);

    ros::Subscriber cost_head_velocity_sub = ros_nh.subscribe("/head_cost/cost_head_velocity", 1, costHeadVelocityCallback);
    ros::Subscriber cost_head_direction_sub = ros_nh.subscribe("/head_cost/cost_head_direction", 1, costHeadDirectionCallback);
    ros::Subscriber cost_head_objects_sub = ros_nh.subscribe("/head_cost/cost_head_objects", 1, costHeadObjectsCallback);
    ros::Subscriber cost_head_fluctuation_sub = ros_nh.subscribe("/head_cost/cost_head_fluctuation", 1, costHeadFluctuationCallback);
    ros::Subscriber cost_head_final_sub = ros_nh.subscribe("/head_cost/cost_head_final", 1, costHeadFinalCallback);
    ros::Subscriber cost_head_update_sub = ros_nh.subscribe("/head_cost/cost_head_update", 1, costHeadUpdateCallback);

    ros::Timer timer1 = ros_nh.createTimer(ros::Duration(0.25), timerCallback); // RATE 4 Hz to publish

    std::thread send_pcl_socket_thread(send_pcl_func);
    std::thread send_other_socket_thread(send_other_func);

    send_pcl_socket_thread.join();
    send_other_socket_thread.join();

    send_pcl_socket.close_socket();
    send_other_socket.close_socket();
}

void send_frame_test(const u_char * send_compressed_pcl_buffer){
    int decompressed_data_size = src_buffer_size;
    int font = decompressed_data_size/1024;
    int back = decompressed_data_size%1024;
    if(back > 0) {
        font += 1;
    }
    int size = compressed_data_size;
    int pkg_num = size/1024;
    int last_pkg_bytes = size % 1024;
    if(last_pkg_bytes > 0) {
        pkg_num += 1;
    }
    else{
        last_pkg_bytes = 1024;
    }
    unsigned char buffer[8];
    memset(buffer,0, sizeof(buffer));
    /*send mark*/
    buffer[0] = 'c';
    buffer[1] = 'h';
    buffer[2] = (u_char)font;
    buffer[3] = (u_char)(back/256);
    buffer[4] = (u_char)(back%256);
    buffer[5] = (unsigned char) pkg_num; //255k at most ///Too large?
    buffer[6] = (unsigned char) (last_pkg_bytes / 256);
    buffer[7] = (unsigned char) (last_pkg_bytes % 256);
    //send size msg
    send_pcl_socket.send_msg(buffer, sizeof(buffer));
    //send buffer msg
    unsigned char send_buffer[1024];
    for(int j=0; j<pkg_num; j++)
    {
        int pkg_size = 1024;
        if(j == pkg_num-1){
            pkg_size=last_pkg_bytes;
        }
        for(int i=0; i<pkg_size; i++){
            send_buffer[i] = *(send_compressed_pcl_buffer+1024*j+i);
        }
        send_pcl_socket.send_msg(send_buffer);
    }

    /// Check connection
    static int failure_times = 0;
    bool if_connected = send_pcl_socket.receive_heartbeat();
    if(!if_connected){
        failure_times ++;
        if(failure_times > 3){
            send_pcl_socket.restart_connection();
            failure_times = 0;
        }
    }else{
        failure_times = 0;
    }
}


void send_other_test(const u_char * send_compressed_pcl_buffer){
    int decompressed_data_size = other_buffer_size;
    int font = decompressed_data_size/1024;
    int back = decompressed_data_size%1024;
    if(back > 0) {
        font += 1;
    }
    int size = compressed_other_data_size;
    int pkg_num = size/1024;
    int last_pkg_bytes = size % 1024;
    if(last_pkg_bytes > 0) {
        pkg_num += 1;
    }
    else{
        last_pkg_bytes = 1024;
    }
    unsigned char buffer[8];
    memset(buffer,0, sizeof(buffer));
    /*send mark*/
    buffer[0] = 'c';
    buffer[1] = 'h';
    buffer[2] = (u_char)font;
    buffer[3] = (u_char)(back/256);
    buffer[4] = (u_char)(back%256);
    buffer[5] = (unsigned char) pkg_num; //255k at most
    buffer[6] = (unsigned char) (last_pkg_bytes / 256);
    buffer[7] = (unsigned char) (last_pkg_bytes % 256);
    //send size msg
    send_other_socket.send_msg(buffer, sizeof(buffer));
    //send buffer msg
    unsigned char send_buffer[1024];
    for(int j=0; j<pkg_num; j++)
    {
        int pkg_size = 1024;
        if(j == pkg_num-1){
            pkg_size=last_pkg_bytes;
        }
        for(int i=0; i<pkg_size; i++){
            send_buffer[i] = *(send_compressed_pcl_buffer+1024*j+i);
        }
        send_other_socket.send_msg(send_buffer);
    }

    /// Check connection
    static int failure_times = 0;
    bool if_connected = send_other_socket.receive_heartbeat();
    if(!if_connected){
        failure_times ++;
        if(failure_times > 3){
            send_other_socket.restart_connection();
            failure_times = 0;
        }
    }else{
        failure_times = 0;
    }

}
void send_pcl_func(){
    ROS_INFO_STREAM("Send PCl thread Opend....");

    while(ros::ok()){
        while (!send_pcl_socket.is_connected_to_client()){
            ROS_INFO_STREAM("[server PCl]Waiting for Client Connect...");
            send_pcl_socket.wait_client_connect();
            usleep(10e3);
            ros::spinOnce();
        }
//        ROS_INFO_STREAM("[server PCl]Client Connected");
        // package number and last package size
        while (pcl_cloud_ready && !pcl_data_locked) {

            pcl_data_locked = true;
            const u_char * test_ptr = &compressed_buffer[0];
            try{
                send_frame_test(test_ptr);
                std::cout << "Pcl data sended!" << std::endl;
            }catch(int e){
                std::cout << " an error occured when sending point cloud!" << std::endl;
            }

            usleep(10e3);
            pcl_cloud_ready = false;
            pcl_data_locked = false;
        }
        ros::spinOnce();
    }
}

void send_other_func(){
    ROS_INFO_STREAM("Send other thread Opend....");
    while(ros::ok()){

        while (!send_other_socket.is_connected_to_client()){
            ROS_INFO_STREAM("[server other]Waiting for Client Connect...");
            send_other_socket.wait_client_connect();
        }

        // package number and last package size
        while (other_ready) {
            try{            
                other_message_compress(display_msg);
                const u_char * other_test_ptr = &compressed_other_buffer[0];
                send_other_test(other_test_ptr);            
            }catch(int e){
                std::cout << " an error occured when sending other informations!" << std::endl;
            }

            usleep(10e3);
            other_ready = false;
        }
        ros::spinOnce();
    }
}


