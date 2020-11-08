//
// Created by sun on 2019/12/3.
//

#ifndef SRC_MULTITHREADSOCKET_H
#define SRC_MULTITHREADSOCKET_H

#include <unistd.h>
#include <string>
#include <iostream>
#include <utility>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <ctime>
#define Transfer_float_data_size 10
using namespace std;

enum socket_type{
    send_pcl = 1,
    send_float_data = 2,
    receive_pcl = 3,
    receive_float_data = 4
};


class MultiThreadSocket {
private:
    int _port = -1;
    std::string _address = "127.0.0.1";
    long long _send_delay = 50;
    long long _receive_delay = 10;
    bool _connected = false;
    bool _socket_inited = false;
    socket_type _current_socket_type;
    int _socket_fd = -1;
    bool _connected_to_server = false;
    bool _connected_to_client = false;

public :
    explicit MultiThreadSocket(socket_type _type);
    ~MultiThreadSocket();
    MultiThreadSocket(socket_type _type,string addr,int port);
    MultiThreadSocket(socket_type _type,int port);
    struct sockaddr_in c_in{}; //used for recev
    struct sockaddr_in s_in{}; //used for send
    int set_send_delay(int us);
    int set_receive_delay(int us);

    void set_address(std::string addr);
    bool hrt_check_enabled = false;
    bool socket_is_init();
    std::string get_address();
    socket_type get_socket_type();
    int get_send_delay();
    int get_receive_delay();

    int get_socket_fd();
    int init_socket(int domain = AF_INET, int type = SOCK_DGRAM, int protocol = IPPROTO_UDP);
    int socket_connect();
    int receive_msg(void* buf,int size = 1024);
    int send_msg(void* buf,int size = 1024);
    void close_socket();
    bool is_connected_to_server();
    bool is_connected_to_client();

    void wait_client_connect();

    void connect_to_server();

    int no_check_receive_msg(void *buf, int size);

    int no_check_send_msg(void *buf, int size);

    void send_heartbeat();

    bool receive_heartbeat();

    void restart_connection();

    };


#endif //SRC_MULTITHREADSOCKET_H
