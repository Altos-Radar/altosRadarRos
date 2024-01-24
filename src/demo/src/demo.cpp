/*
 * @Copyright (C) 2024 by Altos Radar. All rights reserved.
 * @Author:info@altosradar.com
 * @Date: 2024-01-24 13:14:09
 * @Description: Pointcloud visualization demo
 */

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <string>
#include <visualization_msgs/Marker.h>

ros::Publisher info_pub;
ros::Publisher origin_pub;

void callback(const sensor_msgs::PointCloud2::Ptr& msg)
{
    pcl::PointCloud<pcl::PointXYZHSV> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // Point numbers in current cycle
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

    /// text
    std::stringstream description;
    description << "Point number: " << num;
    info.text = description.str();

    /// pose
    info.pose.position.x = -30;
    info.pose.position.y = 0;
    info.pose.position.z = 3;
    info.pose.orientation.x = 0;
    info.pose.orientation.y = 0;
    info.pose.orientation.z = 0;
    info.pose.orientation.w = 1;

    /// scale
    info.scale.x = 5;
    info.scale.y = 1;
    info.scale.z = 10;
    info.color.r = 1.0;
    info.color.g = 0.0;
    info.color.b = 0.0;
    info.color.a = 1;

    info_pub.publish(info);

    // Origin
    visualization_msgs::Marker origin;
    origin.header.frame_id = "altosRadar";
    origin.header.stamp = msg.get()->header.stamp;
    origin.type = visualization_msgs::Marker::SPHERE;
    origin.action = visualization_msgs::Marker::ADD;

    /// pose
    origin.pose.position.x = 0;
    origin.pose.position.y = 0;
    origin.pose.position.z = 0;
    origin.pose.orientation.x = 0;
    origin.pose.orientation.y = 0;
    origin.pose.orientation.z = 0;
    origin.pose.orientation.w = 1;

    /// scale
    origin.scale.x = 4;
    origin.scale.y = 4;
    origin.scale.z = 4;
    origin.color.r = 1.0;
    origin.color.g = 1.0;
    origin.color.b = 0.0;
    origin.color.a = 1;

    origin_pub.publish(origin);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "demo");
    ros::NodeHandle nh;

    auto sub = nh.subscribe("/altosRadar", 1, callback);
    info_pub = nh.advertise<visualization_msgs::Marker>("/info", 10);
    origin_pub = nh.advertise<visualization_msgs::Marker>("/origin", 10);

    ros::spin();
    return 0;
}
