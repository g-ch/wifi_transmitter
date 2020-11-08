//
// Created by sun on 2019/12/3.
//

#include "MultiThreadSocket.h"

#include <utility>

MultiThreadSocket::MultiThreadSocket(socket_type _type){
    //默认参数设置
    _current_socket_type = _type;
    init_socket();
}

// 析构函数 关闭socket
MultiThreadSocket::~MultiThreadSocket(){
    close_socket();
}

MultiThreadSocket::MultiThreadSocket(socket_type _type, string addr, int port) {
    _current_socket_type = _type;
    _port = port;
    _address = std::move(addr);
    init_socket();
    if(_current_socket_type==receive_pcl||_current_socket_type==receive_float_data){
        socket_connect();
    }
}

MultiThreadSocket::MultiThreadSocket(socket_type _type, int port) {
    _current_socket_type = _type;
    _port = port;
    init_socket();
}

// 修改发送间隔时间
int MultiThreadSocket::set_send_delay(int us = 1){
    return _send_delay = us;
}

int MultiThreadSocket::set_receive_delay(int us = 1){
    return _receive_delay = us;
}

// 设置地址
void MultiThreadSocket::set_address(std::string addr){
    _address = std::move(addr);
    if(_current_socket_type==receive_pcl||_current_socket_type==receive_float_data){
        s_in.sin_addr.s_addr=inet_addr(_address.c_str());//accept any address
        socket_connect();
    }
}

//查看当前地址
std::string MultiThreadSocket::get_address(){
    return _address;
}


bool MultiThreadSocket::socket_is_init() {
    return _socket_inited;
}

socket_type MultiThreadSocket::get_socket_type() {
    return _current_socket_type;
}

int MultiThreadSocket::get_send_delay() {
    return _send_delay;
}

int MultiThreadSocket::get_receive_delay() {
    return _receive_delay;
}

int MultiThreadSocket::get_socket_fd() {
    return _socket_fd;
}

int MultiThreadSocket::init_socket(int domain, int type, int protocol) {
    if(_current_socket_type==send_float_data||_current_socket_type==send_pcl){
        s_in.sin_family = AF_INET;//IPV4 communication domain
        s_in.sin_addr.s_addr=INADDR_ANY;//accept any address
        s_in.sin_port = htons(_port);//change port to netchar
        _socket_fd = socket(domain,type,protocol);
        bind(_socket_fd,(struct sockaddr *)&s_in,sizeof(struct sockaddr));
        timeval tv = {0, 500000};// 设置超时时间0.5s
        setsockopt(_socket_fd, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(timeval));
        int nRecvBuf=64*1024;//设置为64K
        setsockopt(_socket_fd,SOL_SOCKET,SO_RCVBUF,(const char*)&nRecvBuf,sizeof(int));
    } else{
        s_in.sin_family = AF_INET;//IPV4 communication domain
        s_in.sin_port = htons(_port);//change port to netchar
        s_in.sin_addr.s_addr=inet_addr(_address.c_str());//accept any address
        _socket_fd = socket(domain,type,protocol);
        //设置超时时间
        timeval tv = {0, 500000};// 设置超时时间0.5s
        setsockopt(_socket_fd, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(timeval));
        // 设置接收缓冲
        int nRecvBuf=64*1024;//设置为64K
        setsockopt(_socket_fd,SOL_SOCKET,SO_RCVBUF,(const char*)&nRecvBuf,sizeof(int));
    }
    return _socket_fd;
}

int MultiThreadSocket::socket_connect() {
    _connected = connect(get_socket_fd(), (struct sockaddr *) &s_in, sizeof(struct sockaddr)) == 0;
    usleep(get_send_delay());
    return _connected;

}

void MultiThreadSocket::connect_to_server(){
    if(!_connected_to_server){
        u_char buffer[3];
        buffer[0] = 'q';
        buffer[1] = 'a';
        buffer[2] = 'q';
        // memset(buffer,0, sizeof(buffer));
        no_check_send_msg(buffer, sizeof(buffer));
        memset(buffer,0, sizeof(buffer));
        no_check_receive_msg(buffer, sizeof(buffer));
        if(buffer[0]=='a'&&buffer[1]=='q'&&buffer[2]=='a'){
            _connected_to_server = true;
        }
    }
}

void MultiThreadSocket::wait_client_connect(){
    u_char buffer[3];
    if(!_connected_to_client){
        memset(buffer,0, sizeof(buffer));
        no_check_receive_msg(buffer, sizeof(buffer));
        if(buffer[0]=='q'&&buffer[1]=='a'&&buffer[2]=='q'){
            buffer[0] = 'a';
            buffer[1] = 'q';
            buffer[2] = 'a';
            no_check_send_msg(buffer, sizeof(buffer));
            _connected_to_client = true;
        }
    }
}

// 使用try catch
int MultiThreadSocket::receive_msg(void* buf,int size) {
//    if(hrt_check_enabled){
//        if(_connected_to_server){
//            socklen_t len = sizeof(c_in);
//            int _size =  recvfrom(get_socket_fd(), buf, size, 0, (struct sockaddr*)&c_in, &len);
//            usleep(get_receive_delay());
//            if(_size>0){
//                u_char buffer[8] = "qqqqqqq";
//                no_check_send_msg(buffer,8);
//            }else{
//                _connected_to_server = false;
//            }
//            return _size;
//        } else{
//            connect_to_server();
//        }
//    }
//    else{
        return no_check_receive_msg(buf,size);
//    }
}


int MultiThreadSocket::no_check_receive_msg(void* buf,int size) {
    socklen_t len = sizeof(c_in);
    int _size = -1;
    for(int time_out_counter = 0; time_out_counter < 10; time_out_counter++){
        _size =  recvfrom(get_socket_fd(), buf, size, 0, (struct sockaddr*)&c_in, &len);
        usleep(get_receive_delay());
        if(_size > 0) break;
    }
    return _size;
}

void MultiThreadSocket::send_heartbeat() {
    u_char buffer[8] = "qqqqqqq";
    no_check_send_msg(buffer,8);
}

bool MultiThreadSocket::receive_heartbeat() {
    u_char buffer[8];
    return no_check_receive_msg(buffer, sizeof(buffer)) == 8;
}

void MultiThreadSocket::restart_connection(){
    _connected_to_client = false;
    close_socket();
    usleep(1e5);
    init_socket();
}

// 使用try catch
int MultiThreadSocket::send_msg(void* buf,int size) {
//    if(hrt_check_enabled){
//        if(_connected_to_client){
//            socklen_t len = sizeof(struct sockaddr);
//            int _size;
//            if(_current_socket_type==send_float_data||_current_socket_type==send_pcl) {
//                _size = sendto(get_socket_fd(), buf, size, 0, (struct sockaddr *)&c_in,sizeof(struct sockaddr));
//            }else{
//                _size = sendto(get_socket_fd(), buf, size, 0, (struct sockaddr *)&s_in,sizeof(struct sockaddr));
//            }
//            usleep(get_send_delay());
//            if(_size>0){
//                u_char buffer[8];
//                _connected_to_client = true;
//                if(no_check_receive_msg(buffer, sizeof(buffer))!=8){
//                    _connected_to_client = false;
//                }
//            }
//            return _size;
//        } else{
//            wait_client_connect();
//        }
//    } else{
    return no_check_send_msg(buf,size);
//    }
}

// 使用try catch
int MultiThreadSocket::no_check_send_msg(void* buf,int size) {
    socklen_t len = sizeof(struct sockaddr);
    int _size;
    if(_current_socket_type==send_float_data||_current_socket_type==send_pcl) {
        _size = sendto(get_socket_fd(), buf, size, 0, (struct sockaddr *)&c_in,sizeof(struct sockaddr));
    }else{
        _size = sendto(get_socket_fd(), buf, size, 0, (struct sockaddr *)&s_in,sizeof(struct sockaddr));
    }
    usleep(get_send_delay());
    return _size;
}

void MultiThreadSocket::close_socket(){
    close(get_socket_fd());
}

bool MultiThreadSocket::is_connected_to_server() {
    return _connected_to_server;
}
bool MultiThreadSocket::is_connected_to_client() {
    return _connected_to_client;
}







