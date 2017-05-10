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
//#include <cv.h>   
//#define ADDR "192.168.1.144"   
#define ADDR "127.0.0.1" //在本机测试用这个地址，如果连接其他电脑需要更换IP  

#define SERVERPORT 10001  
#define BYTE_SEND_INTERVAL 1

using namespace std;
using namespace cv;
int sock_clit;
struct sockaddr_in serv_addr; 

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

/* @function main */
int main( int argc, char **argv )
{
    ros::init(argc, argv, "socket_client");
    ros::NodeHandle n;

    /*compress paras init*/
    vector<int> param= vector<int>(2); 
    param[0]=CV_IMWRITE_JPEG_QUALITY; 
    param[1]=50;//default(95) 0-100 

    /*socket paras init*/   
    char get_msg[10] = {0};    
  
    //sock = socket(AF_INET, SOCK_STREAM, 0);  
    sock_clit = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in c_in; 
    socklen_t len=sizeof(struct sockaddr);

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

  	Size rec_size = Size(1280,720);
    Mat rec_img = Mat(rec_size, CV_8UC3);
  	namedWindow("client");

    /*define mat for image*/
    
    cout<<"transmission started"<<endl;

    char quit_flag = ' ';
    unsigned char buf[1024]="";//content buff area 

    while(ros::ok && quit_flag!='q'){
        
	    
        char send_buff[8] = "";
        send_buff[0] = 1;
        send_buff[1] = 2;
        send_buff[2] = 3;
        send_data(send_buff);

         /*recieve*/
        int r_len=recvfrom(sock_clit,buf,1024,0,(struct sockaddr*)&c_in,&len);

        if(r_len > 0)
        {  
            cout<<"image received!"<<endl;
            /*Verify first package*/
            if(buf[0]=='i' && buf[1]=='m' && buf[2]=='a' && buf[3]=='g' && buf[4]=='e')
            {
                int pkg_num = buf[5];
                int last_pkg_bytes = buf[6]*256 + buf[7];
                vector <uchar> rec_vec;

                long int total_size = (pkg_num-1)*1024 + last_pkg_bytes;
                //cout <<"received data length "<<total_size<<endl;
                rec_vec.resize(total_size);

                /*receive and reconstruct data*/
                for(int i=0; i<pkg_num; i++)
                {
                    r_len = recvfrom(sock_clit,buf,1024,0,(struct sockaddr*)&c_in,&len);
                    int pkg_size = 1024;
                    if(i == pkg_num-1 && last_pkg_bytes!=0) pkg_size=last_pkg_bytes;

                    //cout <<"pkg_size "<<pkg_size<<endl;

                    for(int j=0; j<pkg_size; j++)
                        rec_vec[1024*i+j] = buf[j];

                }

                /*decode and show*/
                rec_img = imdecode(rec_vec,CV_LOAD_IMAGE_COLOR);
                if(!rec_img.empty())imshow("client", rec_img);
                quit_flag = waitKey(10);
            }
        }
        //if(!input_frame.empty())imshow("client", input_frame); 
        //quit_flag = waitKey(20);

        ros::spinOnce();  
    } 
    
    destroyWindow("client");
    close(sock_clit);  

    return 0;
}
