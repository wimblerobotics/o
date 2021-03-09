#include "ros/ros.h"
#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sys/socket.h"
#include "sys/types.h"
#include <unistd.h>


int clientSocketFd;
char recvBuff[1024];
struct sockaddr_in serverAddress;


int main(int argc, char** argv) {
std::cout << "a\n";
  ros::init(argc, argv, "teensy_node");
  ros::NodeHandle nh;

  memset(recvBuff, '0', sizeof(recvBuff));
  clientSocketFd = socket(AF_INET, SOCK_STREAM, 0);
  if (clientSocketFd < 0) {
    ROS_ERROR("[teensy_node] Socket error");
    return 1;
  }

std::cout << "b\n";
  memset(&serverAddress, '0', sizeof(serverAddress));
  serverAddress.sin_addr.s_addr = inet_addr("192.168.2.120");
  serverAddress.sin_family = AF_INET;
  serverAddress.sin_port = htons(80);

std::cout << "c\n";
  if (connect(clientSocketFd, (struct sockaddr*) &serverAddress, sizeof(serverAddress)) < 0) {
    ROS_ERROR("[teensy_node] connect error");
    return 1;
  }

std::cout << "d\n";
  char request[] = "GET / HTTP/1.1\r\n\r\n";
  if (send(clientSocketFd, request, strlen(request), 0) < 0) {
    ROS_ERROR("[teensy_node] request failed");
    return 1;
  }

std::cout << "e\n";
  ssize_t n;
  if ((n = recv(clientSocketFd, recvBuff, sizeof(recvBuff), 0)) < 0) {
    ROS_ERROR("[teensy_node] recv error");
    return 1;
  }

  recvBuff[n] = '\0';
  puts(recvBuff);
  // int n;
  // while ((n = read(clientSocketFd, recvBuff, sizeof(recvBuff) - 1)) > 0) {
  //   recvBuff[n] = 0;
  //   fputs(recvBuff, stdout);
  // }

  ROS_INFO("[teensy_node] starting loop");
  ros::Rate r(20);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("[teensy_node] shutdown");
  return 0;
}