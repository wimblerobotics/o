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

// TODO: Take distance arg and return vector of (x,y,angle) tuples of points within that distance in costmap

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


inline int mapIndex(const nav_msgs::OccupancyGrid::ConstPtr& costmap_msg, int i, int j) {
    int result = i + (j * costmap_msg->info.width);
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

    std::size_t d_x = -10000;
    std::size_t d_y = -10000;
    double min_dist = 10000;
    geometry_msgs::PoseStamped new_pt;
    geometry_msgs::PoseStamped pt;

    pt.header.frame_id = "base_link";
    pt.pose.position.x = 0;
    pt.pose.position.y = 0;
    pt.pose.position.z = 0;
    pt.pose.orientation.w = 1;
    pt.pose.orientation.x = 0;
    pt.pose.orientation.y = 0;
    pt.pose.orientation.z = 0;

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
    std::size_t i_midpoint = costmap_msg->info.width / 2;
    std::size_t j_midpoint = costmap_msg->info.height / 2;

    // getting minimum distance
    for (std::size_t j = 0; j < costmap_msg->info.height; j++) {
        for (std::size_t i = 0; i < costmap_msg->info.width; i++) {
            double cost = costmap_msg->data[mapIndex(costmap_msg, i, j)];
            if(cost == 100) {
                // convert to world position
                double d2_sq = ((i - i_midpoint) * (i - i_midpoint)) + ((j - j_midpoint) * (j - j_midpoint));
                double d2_dist = sqrt(d2_sq * 1.0) * costmap_msg->info.resolution;
                // if (d2_dist < 1.0) {
                //     ROS_INFO("[roam_node] i: %ld, j: %ld"
                //              ", d2_sq: %7.4f, d2_dist: %7.4f"
                //              ", old min_dist: %7.4f"
                //              ", new min: %s"
                //              , i, j
                //              , d2_sq, d2_dist
                //              , min_dist
                //              , (d2_dist < min_dist) ? "TRUE" : "false"
                //     );
                // }
                if (d2_dist < min_dist) {
                    min_dist = d2_dist;
                    d_x = i;
                    d_y = j;
                }
            }
        }
    }

    double angle = std::atan2(d_y - (j_midpoint * 1.0), d_x - (i_midpoint * 1.0));

    ROS_INFO("roam_node] min_dist: %7.4f"
             ", at x: %ld (%7.4f)"
             ", y: %ld (%7.4f)"
             ", angle: %7.4f"
             ", j_midpoint %ld, j_midpoint: %ld"
             ", d_y - j_midpoint: %7.4f, d_x - i_midpoint: %7.4f"
             , min_dist
             , d_x, (d_x * costmap_msg->info.resolution) + costmap_msg->info.origin.position.x
             , d_y, (d_y * costmap_msg->info.resolution) + costmap_msg->info.origin.position.y
             , angle
             , j_midpoint, i_midpoint
             , d_y - (j_midpoint * 1.0), d_x - (i_midpoint * 1.0)
    );
    for (std::size_t jx = costmap_msg->info.height; jx > 0 ; jx--) {
        char line[costmap_msg->info.width];
        for (std::size_t ix = 0; ix < costmap_msg->info.width; ix++) {
            double cost = costmap_msg->data[mapIndex(costmap_msg, ix, jx - 1)];
            line[ix] = (cost == 100) ? '*' : '.';
        }

        if (jx == j_midpoint) line[i_midpoint] = '@';
        if (jx == d_y) line[d_x] = 'X';
        ROS_INFO("%c %3ld: %s", (jx == d_y)? '>' : ' ', jx, line);
    }
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
