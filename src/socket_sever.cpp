#include "ros/ros.h"  
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include <stdio.h>  
#include <iostream>  
#include <stdlib.h>  
#include <netinet/in.h>  
#include <unistd.h>  
#include <sys/socket.h>  
#include <arpa/inet.h>  
#include <string.h>   
  
#define PORT 10001  
#define BYTE_SEND_INTERVAL 1

using namespace std;
using namespace cv;
int sock_sev;
struct sockaddr_in s_in;// address structure 
struct sockaddr_in c_in; 
socklen_t in_len; 

int send_data(unsigned char* data, long int size) //255k at most
{
    /*package number and last package size*/
    int pkg_num = size/1024; 
    int last_pkg_bytes = size % 1024;
    if(last_pkg_bytes > 0) pkg_num += 1;
    else last_pkg_bytes = 1024;

    unsigned char buffer[1024];
    for(int i=0; i<1024; i++)
        buffer[i] = 0;

    /*send mark*/
    buffer[0] = 'i';
    buffer[1] = 'm';
    buffer[2] = 'a';
    buffer[3] = 'g';
    buffer[4] = 'e';
    buffer[5] = (unsigned char) pkg_num; //255k at most
    buffer[6] = (unsigned char) (last_pkg_bytes / 256);
    buffer[7] = (unsigned char) (last_pkg_bytes % 256);

    sendto(sock_sev, buffer, 1024, 0, (struct sockaddr *)&c_in,sizeof(struct sockaddr));
    waitKey(BYTE_SEND_INTERVAL);

    /*send data*/
    for(int j=0; j<pkg_num; j++)
    {
        int pkg_size = 1024;
        if(j == pkg_num-1) pkg_size=last_pkg_bytes;

        for(int i=0; i<pkg_size; i++)
            buffer[i] = *(data+1024*j+i);

        sendto(sock_sev, buffer, 1024, 0, (struct sockaddr *)&c_in,sizeof(struct sockaddr));
        waitKey(BYTE_SEND_INTERVAL);
    }

    return 1;
}


/* @function main */
int main( int argc, char **argv )
{
    ros::init(argc, argv, "socket_server");
    ros::NodeHandle n; 
  
    //socklen_t len;  
    unsigned char buf[1024]="";//content buff area 
    char recvbuf[8]="";
  
    s_in.sin_family = AF_INET;//IPV4 communication domain  
    s_in.sin_addr.s_addr=INADDR_ANY;//accept any address  
    s_in.sin_port = htons(PORT);//change port to netchar  
  
 
    sock_sev = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    bind(sock_sev,(struct sockaddr *)&s_in,sizeof(struct sockaddr));  

  	socklen_t len=sizeof(struct sockaddr);

  	Size rec_size = Size(640,480);
  	Mat rec_img	= Mat(rec_size, CV_8UC3);
  	namedWindow("server"); 

    /*open camera*/
    VideoCapture camera(-1); 
    if(!camera.isOpened())  
    {  
        cout<<"can not open camera!"<<endl;
        return -1;  
    }

    Mat input_frame;
    Mat resize_frame;
    Size send_size = Size(1280,720); 

     /*compress paras init*/
    vector<int> param= vector<int>(2); 
    param[0]=CV_IMWRITE_JPEG_QUALITY; 
    param[1]=50;//default(95) 0-100 

    vector<uchar> trans_buff;//buffer for coding  

    cout<<"begin"<<endl; 

    char quit_flag = ' '; 
 

    while(ros::ok && quit_flag!='q'){ 

        in_len = sizeof(c_in);  
        //清空接收缓存数组  
        memset(recvbuf, 0, sizeof(recvbuf));  
        //开始接收数据  
        int n = recvfrom(sock_sev, recvbuf, sizeof(recvbuf), 0, (struct sockaddr *)&c_in, &in_len);  
        cout<<(int)recvbuf[5]<<(int)recvbuf[6]<<(int)recvbuf[7]<<endl;

        if(n > 0)
        {
            /*send*/
            camera >> input_frame;
            resize_frame = Mat(send_size, input_frame.type());
            resize(input_frame, resize_frame, send_size, CV_INTER_CUBIC);

            imencode(".jpg",resize_frame,trans_buff,param);
            unsigned char* buff = &trans_buff[0];

            //send(sock, buff, trans_buff.size(), 0);
            send_data(buff, trans_buff.size());

            cout<<"image size "<<trans_buff.size()<<endl;

            if(!input_frame.empty())imshow("server", resize_frame); 
        }

        quit_flag = waitKey(33);
       
        ros::spinOnce(); 
    }  
    destroyWindow("server"); 
    close(sock_sev);

    return 0;
}
