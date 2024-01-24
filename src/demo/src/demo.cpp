/*
 * @Copyright (C) 2024 by Altos Radar. All rights reserved.
 * @Author:info@altosradar.com
 * @Date: 2024-01-24 13:14:09
 * @Description: Pointcloud visualization demo
 */

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <string>

const static double scale = 30;
ros::Publisher info_pub;

void callback(const sensor_msgs::PointCloud2::Ptr& msg) {
    pcl::PointCloud<pcl::PointXYZHSV> cloud;
    pcl::fromROSMsg(*msg, cloud);

    visualization_msgs::Marker info;
    info.header.frame_id = "altosRadar";
    info.header.stamp = msg.get()->header.stamp;
    info.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    info.action = visualization_msgs::Marker::ADD;

    size_t num = 0;
    for (const auto& point : cloud.points) {
        if (point.s != 0) {
            num++;
        }
    }

    // text
    std::stringstream description;
    description << "Point number: " << num;
    info.text = description.str();
    info.pose.position.x = -30;
    info.pose.position.y = 0;
    info.pose.position.z = 3;

    // scale
    info.scale.x = 5;
    info.scale.y = 1;
    info.scale.z = 10;
    info.color.r = 1.0;
    info.color.g = 0.0;
    info.color.b = 0.0;
    info.color.a = 1;

    info_pub.publish(info);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo");
    ros::NodeHandle nh;

    auto sub = nh.subscribe("/altosRadar", 1, callback);
    info_pub = nh.advertise<visualization_msgs::Marker>("/info", 10);

    ros::spin();
    return 0;
}
