#include "ros/ros.h"
#include "ros/message_traits.h"
#include "opencv2/core.hpp"
#include <unistd.h>
#include <sys/socket.h>
#include <thread>
#include "MultiThreadSocket.h"
#include <lz4.h>
#include <topic_tools/shape_shifter.h>
#include <sensor_msgs/PointCloud2.h>
#include <wifi_transmitter/Display.h>
#include <std_msgs/Float64MultiArray.h>


using namespace std;
using namespace cv;
#define PCLmsgtype sensor_msgs::PointCloud2
#define DisplayMsg wifi_transmitter::Display


// 配合服务器端程序接收

//string server_addr = "127.0.0.1"; //在本机测试用这个地址，如果连接其他电脑需要更换IP
string server_addr = "192.168.100.104";
const string topic_name = "/ring_buffer/cloud_ob/transfered";
const string other_topic_name = "/Display/transfered";
int recv_pcl_socket_port = 12668;
int recv_fdata_socket_port = 12669;
// u_char recv_fdata_buf[10];//content buff area

int src_buffer_size;
int max_compressed_buffer_size;
int compressed_data_size;
bool pcl_cloud_ready = false;

topic_tools::ShapeShifter shapeshifted_msg;
topic_tools::ShapeShifter shapeshifted_other_msg;

std::map<std::string,ros::Publisher>  publishers_;
std::map<std::string,ros::Publisher>  other_publishers_;
MultiThreadSocket recv_other_socket(receive_float_data,server_addr,recv_fdata_socket_port);
MultiThreadSocket recv_pcl_socket(receive_pcl,server_addr,recv_pcl_socket_port);

void recv_pcl_func(u_char* size_buffer,u_char * buffer,ros::NodeHandle nh);
void recv_other_func(u_char* size_buffer,u_char * buffer,ros::NodeHandle nh);

/* @function main */
int main( int argc, char **argv )
{
    ros::init(argc, argv, "multi_thread_socket_client");
    ros::NodeHandle nh;
    u_char pcl_size_buffer[8]="";
    u_char other_size_buffer[8]="";
    u_char recv_pcl_buf[1024]="";// receive pcl content buff area
    u_char recv_other_buf[1024]="";// receive other content buff area
    std::thread recv_pcl_socket_thread(recv_pcl_func,pcl_size_buffer,recv_pcl_buf,nh);
    std::thread recv_other_socket_thread(recv_other_func,other_size_buffer,recv_other_buf,nh);

    recv_pcl_socket_thread.join();
    recv_other_socket_thread.join();
    recv_pcl_socket.close_socket();
    recv_pcl_socket.close_socket();
}

void recv_pcl_func(u_char * size_buffer,u_char * buffer,ros::NodeHandle nh){
    ROS_INFO_STREAM("[client PCl] recev Pcl thread opened ");
    while(!recv_pcl_socket.is_connected_to_server()){
        recv_pcl_socket.connect_to_server();
        ROS_INFO_STREAM("[client PCl] Connecting to server..... ");
    }
    ROS_INFO_STREAM("[client PCl] Server Connected ..... ");
    // vector <uchar> rec_vec;
    while(ros::ok()){
        int recv_len = 0;
        try{
            recv_len = recv_pcl_socket.receive_msg(size_buffer, 8);
        }catch(...){
            ROS_ERROR("[client PCl] Receive Msg Error ... Receive Next Msg....");
        }
        if(recv_len>0){
            if(size_buffer[0]=='c' && size_buffer[1]=='h'){
                int font = size_buffer[2];
                int back = size_buffer[3]*256+size_buffer[4];
                int check_size = (font-1)*1024+back;
                int pkg_num = size_buffer[5];
                // cout << "total pkg num :"<<pkg_num<<endl;
                int last_pkg_bytes = size_buffer[6]*256 + size_buffer[7];
                long int total_size = (pkg_num-1)*1024 + last_pkg_bytes;
                std::vector<u_char> compressed_buffer(total_size);
                std::vector<char> decompressed_buffer(check_size + 65535);  ///CHG + 65535

                for(int i=0; i<pkg_num; i++)
                {
                    recv_pcl_socket.receive_msg(buffer, 1024);
                    int pkg_size = 1024;
                    if(i == pkg_num-1 && last_pkg_bytes!=0) pkg_size=last_pkg_bytes;
                    for(int j=0; j<pkg_size; j++)
                        compressed_buffer[1024*i+j] = buffer[j];
                    // 显示图像
                }

                /* Send heart beat*/
                usleep(100);
                recv_pcl_socket.send_heartbeat();

                /*decode and show*/
                usleep(1e3);
                const int decompressed_size = LZ4_decompress_safe(reinterpret_cast<const char*>(compressed_buffer.data()),
                                                                  decompressed_buffer.data(),
                                                                  compressed_buffer.size(),
                                                                  check_size);
                ros::serialization::IStream stream( reinterpret_cast<uint8_t*>(decompressed_buffer.data()), check_size); //decompressed_buffer.size()  to check_size, CHG
                shapeshifted_msg.read(stream);
                // ROS_INFO_STREAM("check size: "<< check_size<<" total_size :"<<total_size);
                ROS_INFO_STREAM("[pcl] check size: "<< check_size<<" msg_size :"<<shapeshifted_msg.size());
                // shapeshifted_msg.morph(sample_md5,datatype,defination,"");
                shapeshifted_msg.morph(
                        ros::message_traits::MD5Sum<PCLmsgtype>::value(),
                        ros::message_traits::DataType<PCLmsgtype>::value(),
                        ros::message_traits::Definition<PCLmsgtype>::value(),
                        "");
                auto pub_it = publishers_.find(topic_name);
                if(check_size==decompressed_size){
                    if( pub_it == publishers_.end() )
                    {
                        ros::Publisher publisher = shapeshifted_msg.advertise(nh, topic_name, 1, true);
                        auto res = publishers_.insert( std::make_pair(topic_name, publisher) );
                        pub_it = res.first;
                        pub_it->second.publish(shapeshifted_msg);
                    }
		            else{pub_it->second.publish(shapeshifted_msg);}
                }
                else{
                    ROS_INFO_STREAM("[pcl]error data chk_size: "<<check_size<<" decompressed size:"<<decompressed_size);
                }

            }
        }
        else{
             ROS_INFO_STREAM("[pcl]error received an empty package. recv_len="<<recv_len);
        }
	
        ros::spinOnce();
    }
}

void recv_other_func(u_char * size_buffer,u_char * buffer,ros::NodeHandle nh){
    ROS_INFO_STREAM("[client other] recev Pcl thread opened ");
    while(!recv_other_socket.is_connected_to_server()){
        recv_other_socket.connect_to_server();
        ROS_INFO_STREAM("[client other] Connecting to server..... ");
    }
    ROS_INFO_STREAM("[client other] Server Connected ..... ");
    // vector <uchar> rec_vec;
    while(ros::ok()){
        int recv_len = 0;
        try{
            recv_len = recv_other_socket.receive_msg(size_buffer, 8);
        }catch(...){
            ROS_ERROR("[client other] Receive Msg Error ... Receive Next Msg....");
        }
        if(recv_len>0){
            if(size_buffer[0]=='c' && size_buffer[1]=='h'){
                int font = size_buffer[2];
                int back = size_buffer[3]*256+size_buffer[4];
                int check_size = (font-1)*1024+back;
                int pkg_num = size_buffer[5];
                // cout << "total pkg num :"<<pkg_num<<endl;
                int last_pkg_bytes = size_buffer[6]*256 + size_buffer[7];
                long int total_size = (pkg_num-1)*1024 + last_pkg_bytes;
                std::vector<u_char> compressed_buffer(total_size);
                std::vector<char> decompressed_buffer(check_size);

                for(int i=0; i<pkg_num; i++)
                {
                    recv_other_socket.receive_msg(buffer, 1024);
                    int pkg_size = 1024;
                    if(i == pkg_num-1 && last_pkg_bytes!=0) pkg_size=last_pkg_bytes;
                    for(int j=0; j<pkg_size; j++)
                        compressed_buffer[1024*i+j] = buffer[j];
                    // 显示图像
                }

                /* Send heart beat*/
                usleep(100);
                recv_other_socket.send_heartbeat();

                /*decode and show*/
                usleep(1e3);
                const int other_decompressed_size = LZ4_decompress_safe(reinterpret_cast<const char*>(compressed_buffer.data()),
                                                                  decompressed_buffer.data(),
                                                                  compressed_buffer.size(),
                                                                  check_size);
                ros::serialization::IStream stream( reinterpret_cast<uint8_t*>(decompressed_buffer.data()), decompressed_buffer.size() );
                shapeshifted_other_msg.read(stream);
                // ROS_INFO_STREAM("check size: "<< check_size<<" total_size :"<<total_size);
                // ROS_INFO_STREAM("[other] check size: "<< check_size<<" msg_size :"<<shapeshifted_other_msg.size());
                // shapeshifted_other_msg.morph(other_sample_md5,other_datatype,other_defination,"");
                shapeshifted_other_msg.morph(
                        ros::message_traits::MD5Sum<DisplayMsg>::value(),
                        ros::message_traits::DataType<DisplayMsg>::value(),
                        ros::message_traits::Definition<DisplayMsg>::value(),
                        "");
                auto other_pub_it = other_publishers_.find(other_topic_name);
                if(check_size==other_decompressed_size){
                    if( other_pub_it == other_publishers_.end() )
                    {
                        ros::Publisher other_publisher = shapeshifted_other_msg.advertise(nh, other_topic_name, 1, true);
                        auto other_res = other_publishers_.insert( std::make_pair(other_topic_name, other_publisher) );
                        other_pub_it = other_res.first;
                    }
                }
                else{
                    ROS_INFO_STREAM("[float]error data chk_size: "<<check_size<<" decompressed size:"<<other_decompressed_size);
                }
                other_pub_it->second.publish(shapeshifted_other_msg);
            }
        }
        ros::spinOnce();
    }
}
