#include "ros/ros.h"  
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include <stdio.h>  
#include <string.h>  
#include <iostream>  
#include <stdlib.h>  
#include <string>  
#include <unistd.h>  
#include <arpa/inet.h>  
#include <sys/socket.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

//#include <cv.h>   
#define ADDR "192.168.1.102" 
//#define ADDR "127.0.0.1" //在本机测试用这个地址，如果连接其他电脑需要更换IP  

#define SERVERPORT 10001  
#define BYTE_SEND_INTERVAL 1
#define HALF_RANGE 3.2
#define RESOLUSTION 0.1

using namespace std;
using namespace cv;
int sock_clit;
struct sockaddr_in serv_addr; 

pcl::PointCloud<pcl::PointXYZ> cloud;
int point_num, cur_num;
bool cloud_broken;
int broken_check;

int send_data(char* data) //255k at most
{
    /*send mark*/
    char buffer[8];
    buffer[0] = 's';
    buffer[1] = 't';
    buffer[2] = 'a';
    buffer[3] = 'r';
    buffer[4] = 't';
    buffer[5] = *data;
    buffer[6] = *(data+1);
    buffer[7] = *(data+2);

    sendto(sock_clit, buffer, 8, 0, (struct sockaddr *)&serv_addr,sizeof(struct sockaddr));
    waitKey(BYTE_SEND_INTERVAL);

	return 1;
}


void timerCallback(const ros::TimerEvent &e)
{
    broken_check ++;
    if(broken_check > 10000) //Just in case
        broken_check = 10000;
    if(broken_check > 20) //2 seconds
        cloud_broken = true;
    else 
        cloud_broken = false;
}

/* @function main */
int main( int argc, char **argv )
{
    ros::init(argc, argv, "socket_client");
    ros::NodeHandle n;

    /* publisher to rviz*/
    ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("client/cloud", 500);
    ros::Duration(0.5).sleep();

    /* Watch dog for cloud */
    broken_check = 0;
    cloud_broken = false;
    ros::Timer timer = n.createTimer(ros::Duration(0.1), timerCallback);

    /*compress paras init*/
    vector<int> param= vector<int>(2); 
    param[0]=CV_IMWRITE_JPEG_QUALITY; 
    param[1]=40;//default(95) 0-100 

    /*socket paras init*/   
    char get_msg[10] = {0};    
  
    //sock = socket(AF_INET, SOCK_STREAM, 0);  
    sock_clit = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in c_in; 
    socklen_t len=sizeof(struct sockaddr);

    struct timeval timeout;
    timeout.tv_sec = 1;//秒
    timeout.tv_usec = 0;//微秒
    if (setsockopt(sock_clit, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1) {
        cout<<"setsockopt failed:"<<endl;
    }

    if(sock_clit == -1){ 

        return -1;  
    }  

    memset(&serv_addr, 0, sizeof(serv_addr));  
    serv_addr.sin_family = AF_INET;  
    serv_addr.sin_addr.s_addr = inet_addr(ADDR);  // 注释1  
    serv_addr.sin_port = htons(SERVERPORT);  
    if(connect(sock_clit, (struct sockaddr*) &serv_addr, sizeof(serv_addr))==-1){ // 注释2  
        cout << "connect error\n";  
        return -1;  
    }  
    else{  
        cout << "connected ...\n" << endl;  //注释3  
    }  

    /*define mat for image*/
  	Size rec_size = Size(640,480);
    Mat rec_img = Mat(rec_size, CV_8UC3);
    Mat HSV_img;
  	namedWindow("client");
    namedWindow("target");
    
    cout<<"transmission started"<<endl;

    char quit_flag = ' ';

    unsigned char image_buf[1024]="";//content buff area, image
    double buf[1024];  // content buff area, cloud

    int trans_type = 1;

    while(ros::ok && quit_flag!='q'){
        
	    
        if(trans_type == 1)
        {
            /*request image*/
            char send_buff[8] = "";
            send_buff[0] = 1;
            send_buff[1] = 1;
            send_buff[2] = 1;
            send_data(send_buff);
            cout<<"image request sent"<<endl;

             /*recieve*/
            int r_len=recvfrom(sock_clit,image_buf,1024,0,(struct sockaddr*)&c_in,&len);

            if(r_len > 0)
            {  
                cout<<"image received!"<<endl;
                /*Verify first package*/
                if(image_buf[0]=='d' && image_buf[1]=='a' && image_buf[2]=='t' && image_buf[3]=='i')
                {
                    int pkg_num = image_buf[5] + image_buf[4] * 256;
                    int last_pkg_bytes = image_buf[6]*256 + image_buf[7];
                    vector <uchar> rec_vec;

                    long int total_size = (pkg_num-1)*1024 + last_pkg_bytes;
                    //cout <<"received data length "<<total_size<<endl;
                    rec_vec.resize(total_size);

                    /*receive and reconstruct data*/
                    for(int i=0; i<pkg_num; i++)
                    {
                        r_len = recvfrom(sock_clit,image_buf,1024,0,(struct sockaddr*)&c_in,&len);
                        int pkg_size = 1024;
                        if(i == pkg_num-1 && last_pkg_bytes!=0) pkg_size=last_pkg_bytes;

                        //cout <<"pkg_size "<<pkg_size<<endl;

                        for(int j=0; j<pkg_size; j++)
                            rec_vec[1024*i+j] = image_buf[j];

                    }

                    /*decode and show*/
                    rec_img = imdecode(rec_vec,CV_LOAD_IMAGE_COLOR);
                    if(!rec_img.empty())imshow("client", rec_img);
                    quit_flag = waitKey(10);

                    /* Find shiny points */
                    cvtColor(rec_img, HSV_img, CV_BGR2HSV);
                    vector<Mat> channels;
                    split(HSV_img, channels);
                    int nr= HSV_img.rows; // number of rows  
                    int nc= HSV_img.cols; // total number of elements per line  

                    Mat background(nr, nc, CV_8UC1, Scalar::all(0));
                    for (int j=0; j<nr; j++) {  
                      uchar* data_H= channels[0].ptr<uchar>(j); 
                      uchar* data_S= channels[1].ptr<uchar>(j); 
                      uchar* data_V= channels[2].ptr<uchar>(j);  
                      uchar* data_Background = background.ptr<uchar>(j);

                        for (int i=0; i<nc; i++) {  
                            if(data_H[i] > 0 && data_H[i] < 256 && data_S[i] > 0 && data_S[i] < 256 && data_V[i] > 200 && data_V[i] < 256)
                                data_Background[i] = 255;
                        }                    
                    }
                    imshow("target", background);
                    quit_flag = waitKey(10);

                }
            }

            trans_type = 2;
        }
        else if(trans_type == 2)
        {
            /*request cloud*/
            char send_buff[8] = "";
            send_buff[0] = 2;
            send_buff[1] = 2;
            send_buff[2] = 2;
            send_data(send_buff);

            broken_check = 0; //start count, if can not receive all data in 2s, break

             /*recieve*/
            int r_len=recvfrom(sock_clit,image_buf,1024,0,(struct sockaddr*)&c_in,&len);

            if(r_len > 0)
            {  
                cout<<"cloud received!"<<endl;
                /*Verify first package*/
                if(image_buf[0]=='d' && image_buf[1]=='a' && image_buf[2]=='t' && image_buf[3]=='c')
                {
                    int pkg_num = image_buf[5] + image_buf[4] * 256;
                    int last_pkg_bytes = image_buf[6]*256 + image_buf[7];
                    vector <uchar> rec_vec;

                    long int total_size = (pkg_num-1)*1024 + last_pkg_bytes;
                    //cout <<"received data length "<<total_size<<endl;
                    rec_vec.resize(total_size);

                    /*receive and reconstruct data*/
                    for(int i=0; i<pkg_num; i++)
                    {
                        r_len = recvfrom(sock_clit,image_buf,1024,0,(struct sockaddr*)&c_in,&len);
                        int pkg_size = 1024;
                        if(i == pkg_num-1 && last_pkg_bytes!=0) pkg_size=last_pkg_bytes;

                        //cout <<"pkg_size "<<pkg_size<<endl;

                        for(int j=0; j<pkg_size; j++)
                            rec_vec[1024*i+j] = image_buf[j];

                    }

                    /*decode and show*/
                    for(int i = 0; i < rec_vec.size() / 3; i++)
                    {
                        pcl::PointXYZ p;
                        p.x = (double)rec_vec.at(3 * i) * RESOLUSTION - HALF_RANGE;
                        //cout<<rec_vec.at(3 * i)<<", "<<p.x<<endl;
                        p.y = (double)rec_vec.at(3 * i + 1) * RESOLUSTION - HALF_RANGE;
                        p.z = (double)rec_vec.at(3 * i + 2) * RESOLUSTION - HALF_RANGE;
                        //cout<<p.x<<","<<p.y<<","<<p.z<<endl;
                        cloud.points.push_back(p);
                    }
                    sensor_msgs::PointCloud2 cloud2;
                    pcl::toROSMsg(cloud, cloud2);
                    cloud2.header.frame_id = "world";
                    cloud_pub.publish(cloud2);
                    cout << "cloud publish to rviz! \n" << endl;
                    cloud.points.clear();
                }
            }

            trans_type = 1;
        }        

        quit_flag = waitKey(100);
        ros::spinOnce();  
    } 
    
    destroyWindow("client");
    destroyWindow("target");
    close(sock_clit);  

    return 0;
}
