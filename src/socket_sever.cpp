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
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "cv_bridge/cv_bridge.h"    
#include "sensor_msgs/image_encodings.h" 

#define PORT 10001  
#define BYTE_SEND_INTERVAL 1
#define HALF_RANGE 3.2
#define RESOLUSTION 0.1

using namespace std;
using namespace cv;
int sock_sev;
struct sockaddr_in s_in;// address structure 
struct sockaddr_in c_in; 
socklen_t in_len; 

// global mat for image
Mat rec_img;
bool image_ready;

// global mat for cloud
pcl::PointCloud<pcl::PointXYZ> cloud;
bool cloud_ready = false;
int point_num = 0;
int cur_num = 0;

int send_data_char(unsigned char* data, long int size, unsigned char type) //65536k at most
{
    /*package number and last package size*/
    
    int pkg_num = size/1024; 
    cout<<"pkg sent: "<<pkg_num<<endl;
    int last_pkg_bytes = size % 1024;
    if(last_pkg_bytes > 0) pkg_num += 1;
    else last_pkg_bytes = 1024;

    unsigned char buffer[1024];
    for(int i=0; i<1024; i++)
        buffer[i] = 0;
    unsigned char num_high = (unsigned char) (pkg_num / 256);
    unsigned char num_low = (unsigned char) (pkg_num % 256);

    /*send mark*/
    buffer[0] = 'd';
    buffer[1] = 'a';
    buffer[2] = 't';
    buffer[3] = type;
    buffer[4] = num_high;
    buffer[5] = num_low; 
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

int send_data_double(double *data, long int size) 
{
    /*package number and last package size*/
    int pkg_num = size / 1024;  //1k bytes on time
    int last_pkg_bytes = size % 1024;
    if(last_pkg_bytes > 0)
        pkg_num += 1;
    else
        last_pkg_bytes = 1024;

    double buffer[1024];
    for(int i = 0; i < 1024; i++) buffer[i] = 0;

    /*send mark*/
    buffer[0] = 2;
    buffer[1] = 2;
    buffer[2] = 2;
    buffer[3] = 2;
    buffer[4] = point_num;

    buffer[5] = (double)pkg_num;  
    buffer[6] = (double)(last_pkg_bytes / 256);
    buffer[7] = (double)(last_pkg_bytes % 256);

    sendto(sock_sev, buffer, 1024, 0, (struct sockaddr *)&c_in, sizeof(struct sockaddr));

    /*send data*/
    for(int j = 0; j < pkg_num; j++)
    {
        int pkg_size = 1024;
        if(j == pkg_num - 1) pkg_size = last_pkg_bytes;

        for(int i = 0; i < pkg_size; i++) buffer[i] = *(data + 1024 * j + i);

        sendto(sock_sev, buffer, 1024, 0, (struct sockaddr *)&c_in, sizeof(struct sockaddr));
    }

    return 1;
}

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    cout << "new cloud " << endl;
    if(cloud_ready == true) return;

    cloud.points.clear();
    pcl::fromROSMsg(*msg, cloud);
    cloud_ready = true;
}


void imageCallback(const sensor_msgs::ImageConstPtr& tem_msg)  
{ 
    cv_bridge::CvImagePtr cv_ptr;
    //cout<<"tring"<<endl;
    try    
    {    
        /*转化成CVImage*/    
        cv_ptr = cv_bridge::toCvCopy(tem_msg, sensor_msgs::image_encodings::BGR8);    
        cv::waitKey(1);   
    }    
    catch (cv_bridge::Exception& e)    
    {    
        //ROS_ERROR("Could not convert from '%s' to 'mono8'.", tem_msg->encoding.c_str());
	cout<<"convert failed"<<endl;        
	return;
    }

    rec_img = cv_ptr->image;
    image_ready = true;
    //cout<<"image received!"<<endl;
}

void timerCallback(const ros::TimerEvent &e)
{
    cout << "timer create cloud" << endl;
    if(cloud_ready == true) return;

    // create some cloud
    cloud.points.clear();
    int num = 0;
    for(int i = -20; i <= 20; ++i)
        for(int j = -20; j <= 20; ++j)
            for(int k = -20; k <= 20; ++k)
            {
                pcl::PointXYZ p;
                p.x = i * 0.1;
                p.y = j * 0.1;
                p.z = k * 0.1;
                num++;
                cloud.points.push_back(p);
            }
    // cout << "cloud received, mat:" << cloud_mat << endl;
    cloud_ready = true;
}


/* @function main */
int main( int argc, char **argv )
{
    ros::init(argc, argv, "socket_server");
    ros::NodeHandle n; 

    ros::Subscriber cloud_sub = n.subscribe("/ring_buffer/cloud2", 1, cloudCallback);
    ros::Subscriber image_sub = n.subscribe("/zed/rgb/image_raw_color", 1, imageCallback);
    ros::Duration(0.5).sleep();

    // just for test. Create some cloud and publish
    //ros::Timer timer = n.createTimer(ros::Duration(1.0), timerCallback);
  
    //socklen_t len;  
    unsigned char buf[1024]="";//content buff area 
    char recvbuf[8]="";
  
    s_in.sin_family = AF_INET;//IPV4 communication domain  
    s_in.sin_addr.s_addr=INADDR_ANY;//accept any address  
    s_in.sin_port = htons(PORT);//change port to netchar  
  
 
    sock_sev = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
    bind(sock_sev,(struct sockaddr *)&s_in,sizeof(struct sockaddr));  

  	socklen_t len=sizeof(struct sockaddr);

  	/*For image*/

     //namedWindow("server"); 

    /*open camera*/
    //VideoCapture camera(-1); 
    // if(!camera.isOpened())  
    // {  
    //     cout<<"can not open camera!"<<endl;
    //     return -1;  
    // }

    //Mat input_frame;
    Mat resize_frame;
    Size send_size = Size(640,480); 

     /*compress paras init*/
    vector<int> param= vector<int>(2); 
    param[0]=CV_IMWRITE_JPEG_QUALITY; 
    param[1]=40;//default(95) 0-100 

    vector<uchar> trans_buff_img;//buffer for coding  
    vector<uchar> trans_buff_cloud;  // buffer for coding

    cout<<"begin"<<endl; 

    char quit_flag = ' '; 
    int wait_counter = 0;
    int wait_limit = 10;

    while(ros::ok && quit_flag!='q'){ 

    	//cout<<"waiting"<<endl;        
    	in_len = sizeof(c_in);  
        //清空接收缓存数组  
        memset(recvbuf, 0, sizeof(recvbuf));  
        //开始接收数据  
        int n = recvfrom(sock_sev, recvbuf, sizeof(recvbuf), 0, (struct sockaddr *)&c_in, &in_len);  
        cout<<(int)recvbuf[5]<<(int)recvbuf[6]<<(int)recvbuf[7]<<endl;

        if(n > 0 && (int)recvbuf[5] == 1)  //if request for image
        {
            /*send*/
            //camera >> input_frame;
            //resize_frame = Mat(send_size, input_frame.type());
            //resize(input_frame, resize_frame, send_size, CV_INTER_CUBIC);
            // wait for cloud ready
            
            while(!image_ready && ros::ok() && wait_counter < wait_limit)
            {
                //cout << "wait for image" << endl;
                wait_counter ++;
                ros::spinOnce();
                ros::Duration(0.05).sleep();
            }

                wait_counter = 0;
            cout<<"sending image"<<endl;

            if(wait_counter < wait_limit)  //valid image recieved
            {
                resize_frame = Mat(send_size, rec_img.type());
                resize(rec_img, resize_frame, send_size, CV_INTER_CUBIC);

                imencode(".jpg",resize_frame,trans_buff_img,param);
                unsigned char* buff = &trans_buff_img[0];

                send_data_char(buff, trans_buff_img.size(), 'i');

                //cout<<"image size "<<trans_buff_img.size()<<endl;
                image_ready = false;

                    //if(!resize_frame.empty()) 
    		//{
    		//    imshow("server", resize_frame);
    		 //   waitKey(1);		
    		//}
            }
                else cout<<"No valid image!"<<endl;
        }

        else if(n > 0 && (int)recvbuf[5] == 2) //if request for cloud
        {
            // wait for cloud ready
            while(!cloud_ready && ros::ok() && wait_counter < wait_limit) 
            {
                cout << "wait for cloud" << endl;
                wait_counter ++;
                ros::spinOnce();
                ros::Duration(0.05).sleep();
            }

            wait_counter = 0;
            cout<<"sending cloud"<<endl;

            if(wait_counter < wait_limit)  
            {

                // encode point cloud
                trans_buff_cloud.clear();
                for(int i = 0; i < cloud.points.size(); ++i)
                {
                    trans_buff_cloud.push_back((unsigned char)((cloud.points.at(i).x + HALF_RANGE)/0.1) );
                    trans_buff_cloud.push_back((unsigned char)((cloud.points.at(i).y + HALF_RANGE)/0.1) );
                    trans_buff_cloud.push_back((unsigned char)((cloud.points.at(i).z + HALF_RANGE)/0.1) );
                    //cout<<"("<<cloud.points.at(i).x<<","<<cloud.points.at(i).y<<","<<cloud.points.at(i).z<<")"<<endl;
		    //cout<<(unsigned char)((cloud.points.at(i).x + HALF_RANGE)/0.1)<<endl;
                }

                unsigned char *buff = &trans_buff_cloud[0];
                send_data_char(buff, trans_buff_cloud.size(), 'c');
            }
	    cloud_ready = false;
            
        }


        quit_flag = waitKey(33);
       
        ros::spinOnce(); 
    }  
    //destroyWindow("server"); 
    close(sock_sev);

    return 0;
}
