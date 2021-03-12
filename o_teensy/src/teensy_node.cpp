#include "ros/ros.h"
#include <arpa/inet.h>
#include <errno.h>
#include <regex.h>
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Temperature.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sys/socket.h"
#include "sys/types.h"
#include <unistd.h>

#include <o_teensy/MotorCurrent.h>


int clientSocketFd;
char recvBuff[1024];
struct sockaddr_in serverAddress;


void publishMotorCurrent(char* groupString, const ros::Publisher& publisher, uint8_t numberValues) {
  uint16_t index = 0;
  char* savedPtr;
  char* token = strtok_r(groupString, ",] ", &savedPtr);
  
  o_teensy::MotorCurrent message;

  while (token != NULL) {
    if (index >= numberValues) {
      ROS_ERROR("[teensy_node::publishMotorCurrent] invalid index");
      //printf("token: %0lX, index: %d\n", (void*) token, index);
      exit(-1);
    }

    char name[] = "motorX";
    name[strlen(name) -1] = '0' + index;
    long value = strtol(token, NULL, 10);
    message.name = name;
    message.current = value / 1000.0;
    message.timestamp = ros::Time::now();
    publisher.publish(message);

    token = strtok_r(NULL, ",] ", &savedPtr);
    index += 1;
  }
}


void publishSonar(char* groupString, const ros::Publisher& publisher, uint8_t numberValues) {
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
      publisher.publish(rangeMessage);
    }

    //printf("%s, value[%d]: %ld\n", jsonArrayName, index, strtol(token, NULL, 10));
    token = strtok_r(NULL, ",] ", &savedPtr);
    index += 1;
  }
}


void publishTemperature(char* groupString, const ros::Publisher& publisher, uint8_t numberValues) {
  uint16_t index = 0;
  char* savedPtr;
  char* token = strtok_r(groupString, ",] ", &savedPtr);
  
  sensor_msgs::Temperature temperatureMessage;

  while (token != NULL) {
    if (index >= numberValues) {
      ROS_ERROR("[teensy_node::publishTemperature] invalid index");
      //printf("token: %0lX, index: %d\n", (void*) token, index);
      exit(-1);
    }

    char frameId[] = "temperatureX";
    frameId[strlen(frameId) -1] = '0' + index;
    long temperatureTenthsC = strtol(token, NULL, 10);
    temperatureMessage.header.stamp = ros::Time::now();
    temperatureMessage.header.frame_id = frameId;
    temperatureMessage.temperature = temperatureTenthsC / 10.0;
    temperatureMessage.variance = 0;
    publisher.publish(temperatureMessage);

    //printf("%s, value[%d]: %ld\n", jsonArrayName, index, strtol(token, NULL, 10));
    token = strtok_r(NULL, ",] ", &savedPtr);
    index += 1;
  }
}


void publishTimeOfFlight(char* groupString, const ros::Publisher& publisher, uint8_t numberValues) {
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
      publisher.publish(rangeMessage);
    }

    //printf("%s, value[%d]: %ld\n", jsonArrayName, index, strtol(token, NULL, 10));
    token = strtok_r(NULL, ",] ", &savedPtr);
    index += 1;
  }
}


void publishSensor(const ros::Publisher& publisher, const char* jsonArrayName, char* response, uint8_t numberValues,
void (*publishFunction)(char* token, const ros::Publisher& publisher, uint8_t numberValues)) {
  const char regexPattern[] = "\"%s\"\\:\\s*\\[([-0-9, ]*)\\]";
  char regexStr[sizeof(jsonArrayName) +16];
  regex_t sensorRegex;
  regmatch_t sensorGroups[2];

  sprintf(regexStr, regexPattern, jsonArrayName);
  if (regcomp(&sensorRegex, regexStr, REG_EXTENDED)) {
    ROS_ERROR("[teensy_node::publishSensor] regcomp error");
    exit(-1);
  }

  bool fail = regexec(&sensorRegex, response, 2, sensorGroups, 0);
  if (fail) {
    ROS_ERROR("[teensy_node::publishSensor] match failure");
    exit(-1);
  }

  uint16_t matchLength = sensorGroups[1].rm_eo - sensorGroups[1].rm_so;
  char groupString[matchLength + 1];
  strncpy(groupString, &response[sensorGroups[1].rm_so], matchLength);
  groupString[matchLength] = '\0';
  publishFunction(groupString, publisher, numberValues);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "teensy_node");
  ros::NodeHandle nh;
  ros::Publisher motorCurrentPublisher = nh.advertise<o_teensy::MotorCurrent>("motor_current",8); 
  ros::Publisher sonarPublisher = nh.advertise<sensor_msgs::Range>("sonar",8); 
  ros::Publisher temperaturePublisher = nh.advertise<sensor_msgs::Temperature>("temperatureC",8); 
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

    publishSensor(motorCurrentPublisher, "motor_currents_ma", recvBuff, 2, publishMotorCurrent);
    publishSensor(sonarPublisher, "sonar_mm", recvBuff, 4, publishSonar);
    publishSensor(temperaturePublisher, "temperature_tenthsC", recvBuff, 2, publishTemperature);
    publishSensor(timeOfFlightPublisher, "time_of_flight_mm", recvBuff, 8, publishTimeOfFlight);
    ros::spinOnce();
    r.sleep();
  }

  ROS_INFO("[teensy_node] shutdown");
  return 0;
}