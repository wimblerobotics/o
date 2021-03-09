#include "ros/ros.h"
#include <arpa/inet.h>
#include <errno.h>
#include <regex.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sys/socket.h"
#include "sys/types.h"
#include <unistd.h>


int clientSocketFd;
char recvBuff[1024];
struct sockaddr_in serverAddress;


void parseGroup(char* recvBuff, regmatch_t *sensorGroups, uint8_t group, uint8_t numberValues) {
  uint16_t matchLength = sensorGroups[group].rm_eo - sensorGroups[group].rm_so;
  char groupString[matchLength + 1];
  strncpy(groupString, &recvBuff[sensorGroups[group].rm_so], matchLength);
  groupString[matchLength] = '\0';

  uint16_t index = 0;
  char* token = strtok(groupString, ",");
  while (token != nullptr) {
    if (index >= numberValues) {
      ROS_ERROR("[teensy_node::parseGroup] invalid index");
      exit(-1);
    }

    printf("Group: %d, value[%d]: %s\n", group, index++, token);
    token = strtok(nullptr, ",");
  }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "teensy_node");
  ros::NodeHandle nh;

  memset(recvBuff, '0', sizeof(recvBuff));
  clientSocketFd = socket(AF_INET, SOCK_STREAM, 0);
  if (clientSocketFd < 0) {
    ROS_ERROR("[teensy_node] Socket error");
    return 1;
  }

  memset(&serverAddress, '0', sizeof(serverAddress));
  serverAddress.sin_addr.s_addr = inet_addr("192.168.2.120");
  serverAddress.sin_family = AF_INET;
  serverAddress.sin_port = htons(80);

  if (connect(clientSocketFd, (struct sockaddr*) &serverAddress, sizeof(serverAddress)) < 0) {
    ROS_ERROR("[teensy_node] connect error");
    return 1;
  }

  const char request[] = "GET / HTTP/1.1\r\n\r\n";
  // int n;
  // while ((n = read(clientSocketFd, recvBuff, sizeof(recvBuff) - 1)) > 0) {
  //   recvBuff[n] = 0;
  //   fputs(recvBuff, stdout);
  // }

  ROS_INFO("[teensy_node] starting loop");
  ros::Rate r(1);

  regex_t sensorRegex;
  regmatch_t sensorGroups[4];

  static const char sensorRegexStr[] = 
    "\"motor_currents_ma\"\\:\\s*\\[(.*?)\\],\n.*\"sonar_mm\"\\:\\s*\\[(.*?)\\],\n.*\"time_of_flight_mm\"\\:\\s*\\[(.*?)\\]";
  if (regcomp(&sensorRegex, sensorRegexStr, REG_EXTENDED)) {
    ROS_ERROR("[teensy_node] regcomp error");
    return 1;
  }

  while (ros::ok()) {
    if (send(clientSocketFd, request, strlen(request), 0) < 0) {
      ROS_ERROR("[teensy_node] request failed");
      return 1;
    }

    ssize_t n;
    if ((n = recv(clientSocketFd, recvBuff, sizeof(recvBuff), 0)) < 0) {
      ROS_ERROR("[teensy_node] recv error");
      return 1;
    }

    if (n <= 2) {
      continue;
    }

    recvBuff[n] = '\0';
    printf("received message: '%s'\n", recvBuff);

    bool fail = regexec(&sensorRegex, recvBuff, 4, sensorGroups, 0);
    if (fail) {
      ROS_ERROR("[teensy_node] match failure");
      return 1;
    }

    for (int i = 0; i < 4; i++) {
      uint16_t matchLength = sensorGroups[i].rm_eo - sensorGroups[i].rm_so;
      char strCopy[matchLength + 1];
      strncpy(strCopy, &recvBuff[sensorGroups[i].rm_so], matchLength);
      strCopy[matchLength] = '\0';
      printf("match [%d]: '%s'\n", i, strCopy);
    }

    parseGroup(recvBuff, sensorGroups, 1, 2);
    parseGroup(recvBuff, sensorGroups, 2, 4);
    parseGroup(recvBuff, sensorGroups, 3, 8);

    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("[teensy_node] shutdown");
  return 0;
}