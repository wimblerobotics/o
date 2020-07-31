#include "ros/ros.h"
#include <boost/bind.hpp>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <o_msgs/Monitor.h>
#include <o_msgs/RoboClawStatus.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nav_msgs/OccupancyGrid.h>


typedef enum LIDAR_SOURCE {
	NEATO,
	D435_LEFT,
	D435_right,
	NUMBER_LIDAR_SOURCES
} SCAN_SOURCE;

float merged_scans[360][NUMBER_LIDAR_SOURCES];

    // float lowestDist = 1e8;
    // int left_count = 0;
    // int right_count = 0;
    // bool obstacle_left = false;
    // bool obstacle_center = false;
    // bool obstacle_right = false;
    // geometry_msgs::Point32 left_point;
    // geometry_msgs::Point32 center_point;
    // geometry_msgs::Point32 right_point;
    // left_point.x = left_point.y = left_point.z = 0.0;
    // center_point.x = center_point.y = center_point.z = 0.0;
    // right_point.x = right_point.y = right_point.z = 0.0;

    // visualization_msgs::Marker marker;
    // marker.header.frame_id = "scan";
    // marker.header.stamp = ros::Time::now();
    // marker.ns = "closestNamespace";
    // marker.id = 0;
    // marker.type = shape;
    // marker.action = visualization_msgs::Marker::DELETEALL;
    // marker.lifetime = ros::Duration();
    // marker_pub.publish(marker);

    // float angle;
    // int i;
    // for (i = 0, angle = laser_msg->angle_min; 
    //      i < laser_msg->ranges.size(); 
    //      ++i, angle += laser_msg->angle_increment) { 
    //     float range = laser_msg->ranges[i];
    //     if ((range < laser_msg->range_min) || (range > laser_msg->range_max)) continue;

    //     if (range < lowestDist) {
    //         lowestDist = range;
    //     }


    //     if (range < kHAZARD_DISTANCE) {
    //         geometry_msgs::Point32 marker_point;
    //         marker_point.x = cos(angle) * range;
    //         marker_point.y = sin(angle) * range;
    //         marker_point.z = 0;
    //         ROS_INFO("angle: %fr (%fd), range: %f, index: %d, x: %f, y: %f",
    //                  angle,
    //                  angles::to_degrees(angle),
    //                  range,
    //                  i,
    //                  marker_point.x,
    //                  marker_point.y);
    //         addMarker(marker_point, i);
    //     }
    // }
void laserCallback(const sensor_msgs::LaserScanConstPtr& laser_msg) {
}


// some definitions of functions
//#define MAP_WXGX(map, i) (origin_x + (i - map->info.width / 2) * map->info.resolution)
//#define MAP_WYGY(map, j) (origin_y + (j - map->info.height / 2) * map->info.resolution)
inline int mapIndex(const nav_msgs::OccupancyGrid::ConstPtr& costmap_msg, int i, int j) {
    int result = i + (j * costmap_msg->info.width);
    return result;
}

inline double mapXtoGlobalX(const nav_msgs::OccupancyGrid::ConstPtr& costmap_msg, double origin_x, int map_x) {
    double result = origin_x + (map_x - (costmap_msg->info.width / 2)) * costmap_msg->info.resolution;
    return result;
}

inline double mapYtoGlobalY(const nav_msgs::OccupancyGrid::ConstPtr& costmap_msg, double origin_y, int map_y) {
    double result = origin_y + (map_y - (costmap_msg->info.height / 2)) * costmap_msg->info.resolution;
    return result;
}

void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& costmap_msg) {
	ROS_INFO("[roam_node] width: %d"
			 ", height: %d"
			 ", resolution: %7.4f"
			 ", origin: [%7.4f, %7.4f, %7.4f]",
		 	 costmap_msg->info.width, 
			 costmap_msg->info.height,
			 costmap_msg->info.resolution, 
			 costmap_msg->info.origin.position.x, 
			 costmap_msg->info.origin.position.y, 
			 costmap_msg->info.origin.position.z);

    double dist_sq;
    double d_x = -10000;
    double d_y = -10000;
    double min_dist = 10000;
    geometry_msgs::PoseStamped new_pt;
    double origin_x;
    double origin_y;
    geometry_msgs::PoseStamped pt;

    pt.header.frame_id = "base_link";
    pt.pose.position.x = 0;
    pt.pose.position.y = 0;
    pt.pose.position.z = 0;
    pt.pose.orientation.w = 1;
    pt.pose.orientation.x = 0;
    pt.pose.orientation.y = 0;
    pt.pose.orientation.z = 0;

    origin_x = costmap_msg->info.origin.position.x + ((costmap_msg->info.width / 2) * costmap_msg->info.resolution);
    origin_y = costmap_msg->info.origin.position.y + ((costmap_msg->info.height / 2) * costmap_msg->info.resolution);

    tf::TransformListener tf_l(ros::Duration(10));

    try {
        tf_l.waitForTransform(costmap_msg->header.frame_id,
                              pt.header.frame_id, 
                              ros::Time(0), 
                              ros::Duration(1));
        tf_l.transformPose(costmap_msg->header.frame_id, pt, new_pt);
    } catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }

    double pt_x = new_pt.pose.position.x;
    double pt_y = new_pt.pose.position.y;
    double pt_th = tf::getYaw(new_pt.pose.orientation);

    // getting minimum distance
    for (std::size_t j = 0; j < costmap_msg->info.height; j++) {
        for (std::size_t i = 0; i < costmap_msg->info.height; i++) {
            double cost = costmap_msg->data[mapIndex(costmap_msg, i, j)];
            if(cost == 100) {
                // convert to world position
                double w_x = mapXtoGlobalX(costmap_msg, origin_x, i);
                double w_y = mapYtoGlobalY(costmap_msg, origin_y, j);
                double dist_sq = pow(w_x - pt_x, 2)+pow(w_y - pt_y, 2);
                if (dist_sq < min_dist) {
                    min_dist = dist_sq;
                    d_x = w_x;
                    d_y = w_y;
                }
            }
        }
    }

    double angle = std::atan2(d_y - pt_y, d_x - pt_x) - pt_th;

    ROS_INFO("roam_node] min_dist: %7.4f"
             ", at x: %7.4f"
             ", y: %7.4f"
             ", angle: %7.4f"
             , sqrt(min_dist)
             , d_x
             , d_y
             , angle);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "roam_node");
	ros::NodeHandle nh;

    ros::Subscriber sub_d435_left = nh.subscribe<sensor_msgs::LaserScan>("/plaser_scan_left", 1, laserCallback);
    ros::Subscriber sub_d435_right = nh.subscribe<sensor_msgs::LaserScan>("plaser_scan_right", 1, laserCallback);
    ros::Subscriber sub_neato = nh.subscribe<sensor_msgs::LaserScan>("/o/scan", 1, laserCallback);

	ros::Subscriber foo = nh.subscribe<nav_msgs::OccupancyGrid>("/move_base/local_costmap/costmap", 1, costmapCallback);

 	ROS_INFO("[roam_node] starting loop");
    ros::Rate r(10);
	while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
	}

	ROS_INFO("[roam_node] shutdown");
}
