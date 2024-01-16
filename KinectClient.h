#ifndef KINECT_CLIENT_H
#define KINECT_CLIENT_H

#include <string>
#include <winsock2.h>
#include "KinectImage.h"
#include <sstream>
#include <iostream>
#include <ws2tcpip.h>

class SocketClient {
public:
    SocketClient(const std::string& serverAddr, int serverPort);
    ~SocketClient();
    bool sendImage(const kinect_image& image);

private:
    bool connectToServer();
    bool sendData(const std::string& data);

    std::string serverAddress; // 服务器地址
    int port;                  // 服务器端口
    SOCKET socket;             // socket句柄
    bool connected;            // 连接状态标志
};

#endif // KINECT_CLIENT_H
