#include "ros/ros.h"
#include <arpa/inet.h>
#include <errno.h>
#include <regex.h>
#include "sensor_msgs/Range.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sys/socket.h"
#include "sys/types.h"
#include <unistd.h>


int clientSocketFd;
char recvBuff[1024];
struct sockaddr_in serverAddress;


void publishSonar(const ros::Publisher& sonarPublisher, const char* jsonArrayName, char* response, uint8_t numberValues) {
  const char regexPattern[] = "\"%s\"\\:\\s*\\[([-0-9, ]*)\\]";
  char regexStr[sizeof(jsonArrayName) +16];
  regex_t sensorRegex;
  regmatch_t sensorGroups[2];

  sprintf(regexStr, regexPattern, jsonArrayName);
  // printf("len jsonArrayName: %d, len regexStr: %d\n", strlen(jsonArrayName), strlen(regexStr));
  // printf("regexStr: %s\n", regexStr);

  if (regcomp(&sensorRegex, regexStr, REG_EXTENDED)) {
    ROS_ERROR("[teensy_node::publishSonar] regcomp error");
    exit(-1);
  }

  bool fail = regexec(&sensorRegex, response, 2, sensorGroups, 0);
  if (fail) {
    ROS_ERROR("[teensy_node::publishSonar] match failure");
    exit(-1);
  }

  uint16_t matchLength = sensorGroups[1].rm_eo - sensorGroups[1].rm_so;
  char groupString[matchLength + 1];
  strncpy(groupString, &response[sensorGroups[1].rm_so], matchLength);
  groupString[matchLength] = '\0';
  // printf("groupString: '%s'\n", groupString);

  uint16_t index = 0;
  char* savedPtr;
  char* token = strtok_r(groupString, ",] ", &savedPtr);
  
  sensor_msgs::Range rangeMessage;

  while (token != NULL) {
    if (index >= numberValues) {
      ROS_ERROR("[teensy_node::publishSonar] invalid index");
      //printf("token: %0lX, index: %d\n", (void*) token, index);
      exit(-1);
    }

    char frameId[] = "sonarX";
    frameId[strlen(frameId) -1] = '0' + index;
    long range = strtol(token, NULL, 10);
    if ((range > 0) && (range < 2000)) {
      rangeMessage.header.stamp = ros::Time::now();
      rangeMessage.header.frame_id = frameId;
      rangeMessage.radiation_type = sensor_msgs::Range::ULTRASOUND;
      rangeMessage.field_of_view = 0.26; // 15 degrees
      rangeMessage.min_range = 0.0;
      rangeMessage.max_range = 2.0;
      rangeMessage.range = range / 1000.0;
      sonarPublisher.publish(rangeMessage);
    }

    //printf("%s, value[%d]: %ld\n", jsonArrayName, index, strtol(token, NULL, 10));
    token = strtok_r(NULL, ",] ", &savedPtr);
    index += 1;
  }
}


void publishTimeOfFlight(const ros::Publisher& timeOfFlightPublisher, const char* jsonArrayName, char* response, uint8_t numberValues) {
  const char regexPattern[] = "\"%s\"\\:\\s*\\[([-0-9, ]*)\\]";
  char regexStr[sizeof(jsonArrayName) +16];
  regex_t sensorRegex;
  regmatch_t sensorGroups[2];

  sprintf(regexStr, regexPattern, jsonArrayName);
  // printf("len jsonArrayName: %d, len regexStr: %d\n", strlen(jsonArrayName), strlen(regexStr));
  // printf("regexStr: %s\n", regexStr);

  if (regcomp(&sensorRegex, regexStr, REG_EXTENDED)) {
    ROS_ERROR("[teensy_node::publishTimeOfFlight] regcomp error");
    exit(-1);
  }

  bool fail = regexec(&sensorRegex, response, 2, sensorGroups, 0);
  if (fail) {
    ROS_ERROR("[teensy_node::publishTimeOfFlight] match failure");
    exit(-1);
  }

  uint16_t matchLength = sensorGroups[1].rm_eo - sensorGroups[1].rm_so;
  char groupString[matchLength + 1];
  strncpy(groupString, &response[sensorGroups[1].rm_so], matchLength);
  groupString[matchLength] = '\0';
  // printf("groupString: '%s'\n", groupString);

  uint16_t index = 0;
  char* savedPtr;
  char* token = strtok_r(groupString, ",] ", &savedPtr);
  
  sensor_msgs::Range rangeMessage;

  while (token != NULL) {
    if (index >= numberValues) {
      ROS_ERROR("[teensy_node::publishTimeOfFlight] invalid index");
      //printf("token: %0lX, index: %d\n", (void*) token, index);
      exit(-1);
    }

    char frameId[] = "tofX";
    frameId[strlen(frameId) -1] = '0' + index;
    long range = strtol(token, NULL, 10);
    if ((range > 0) && (range < 2000)) {
      rangeMessage.header.stamp = ros::Time::now();
      rangeMessage.header.frame_id = frameId;
      rangeMessage.radiation_type = sensor_msgs::Range::ULTRASOUND;
      rangeMessage.field_of_view = 0.44; // 25 degrees
      rangeMessage.min_range = 0.0;
      rangeMessage.max_range = 2.0;
      rangeMessage.range = range / 1000.0;
      timeOfFlightPublisher.publish(rangeMessage);
    }

    //printf("%s, value[%d]: %ld\n", jsonArrayName, index, strtol(token, NULL, 10));
    token = strtok_r(NULL, ",] ", &savedPtr);
    index += 1;
  }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "teensy_node");
  ros::NodeHandle nh;
  ros::Publisher sonarPublisher = nh.advertise<sensor_msgs::Range>("sonar",8); 
  ros::Publisher timeOfFlightPublisher = nh.advertise<sensor_msgs::Range>("time_of_flight",8); 

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

  ROS_INFO("[teensy_node] starting loop");

  regex_t sensorRegex;
  regmatch_t sensorGroups[4];

  static const char sensorRegexStr[] = 
    "\"motor_currents_ma\"\\:\\s*\\[(.*?)\\],\n.*\"sonar_mm\"\\:\\s*\\[(.*?)\\],\n.*\"time_of_flight_mm\"\\:\\s*\\[(.*?)\\]";
  if (regcomp(&sensorRegex, sensorRegexStr, REG_EXTENDED)) {
    ROS_ERROR("[teensy_node] regcomp error");
    return 1;
  }

  ros::Rate r(20);
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
      // Not a sensor data message.
      continue;
    }

    recvBuff[n] = '\0';
    //printf("received message: '%s'\n", recvBuff);

    bool fail = regexec(&sensorRegex, recvBuff, 4, sensorGroups, 0);
    if (fail) {
      ROS_ERROR("[teensy_node] match failure");
      return 1;
    }

    publishSonar(sonarPublisher, "sonar_mm", recvBuff, 4);
    publishTimeOfFlight(timeOfFlightPublisher, "time_of_flight_mm", recvBuff, 8);
    // parseValues("motor_currents_ma", recvBuff, 2);
    // parseValues("time_of_flight_mm", recvBuff, 8);
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("[teensy_node] shutdown");
  return 0;
}