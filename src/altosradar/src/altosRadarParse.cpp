/**
 * @Author: Fengze Han
 * @Date:   2025-01-17 15:17:44
 * @Last Modified by:   Fengze Han
 * @Last Modified time: 2025-11-18 12:47:14
 */
#include "pointCloud.h"
#include <algorithm>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <math.h>
#include <netinet/in.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

using namespace std;
#define widthSet 8000
#define PORT 4040
#define vrMax 60
#define vrMin -60
#define errThr 3
#define PI 3.1415926

// ------------------- Object List Wire format definitions -------------------
// //
#pragma pack(push, 1)
struct ObjectDet {
    std::uint64_t timestamp_ns{0};
    float x{0.f}, y{0.f}, z{0.f};
    float vx{0.f}, vy{0.f};
    float vx_rel{0.f}, vy_rel{0.f};
    float length{0.f}, width{0.f}, height{0.f};
    float std_x{0.f}, std_y{0.f}, std_z{0.f};
    float std_vx{0.f}, std_vy{0.f};
    std::uint32_t id{0};
    std::uint16_t lifetime{0};
    std::uint8_t track_state{0};
    std::uint8_t motion_state{0};
    std::uint8_t object_type{0};
    std::uint8_t track_quality{0};
    std::uint8_t type_quality{0};
    std::uint8_t size_quality{0};
};
static_assert(sizeof(ObjectDet) == 80,
              "ObjectDet wire size mismatch (expected 80 bytes)");

static constexpr std::size_t kMaxObjects = 150;

struct ObjectsPacket {
    PCKHEADER pckHeader{};
    std::uint32_t sec{0};
    std::uint32_t nsec{0};
    std::uint32_t frameId{0};
    std::uint32_t objectCount{0};
    ObjectDet objects[kMaxObjects];
};
#pragma pack(pop)

static constexpr std::size_t kObjectsOffset = offsetof(ObjectsPacket, objects);

/// @brief Calculate the radar cross section(rcs) of single point
/// @return rcs
float rcsCal(float range, float azi, float snr, float* rcsBuf) {
    int ind = (azi * 180. / PI + 60.1) * 10.;
    float rcs = powf32(range, 2.6) * snr / 5.0e6 / rcsBuf[ind];

    return rcs;
}

/// @brief Calculate the radar ego velocity based on histogram. The assumption
/// is that the statc points are more than the moving points.
/// @return velocity
float hist(vector<POINTCLOUD> pointCloudVec, float* histBuf, float step) {
    int ind = 0;
    float vr = 0;

    for (int i = 0; i < pointCloudVec.size(); i++) {
        for (int j = 0; j < POINTNUM; j++) {
            if (abs(pointCloudVec[i].point[j].range) > 0) {
                vr = pointCloudVec[i].point[j].doppler /
                     cos(pointCloudVec[i].point[j].azi);
                ind = (vr - vrMin) / step;

                // filter the abnormal doppler value
                if (vr > vrMax || vr < vrMin || isnan(vr)) {
                    continue;
                }

                // if (vr <= 0) {
                histBuf[ind]++;
                // }
            }
        }
    }

    return float(
               (max_element(histBuf, histBuf + (int((vrMax - vrMin) / step))) -
                histBuf)) *
               step +
           vrMin;
}

/// @brief Calculate the point cloud in euclidean coordinate based on the radar
/// data. position x,y,z, doppler h, rcs s, direction v: -1 opposite direction
/// moving, 0 static, 1 same direction moving
void calPoint(vector<POINTCLOUD> pointCloudVec,
              pcl::PointCloud<pcl::PointXYZHSV>& cloud,
              int installFlag,
              float* rcsBuf,
              float step,
              float* histBuf) {
    for (int i = 0; i < pointCloudVec.size(); i++) {
        for (int j = 0; j < POINTNUM; j++) {
            if (abs(pointCloudVec[i].point[j].range) > 0) {
                pointCloudVec[i].point[j].ele =
                    installFlag * (pointCloudVec[i].point[j].ele);

                cloud.points[i * POINTNUM + j].x =
                    (pointCloudVec[i].point[j].range) *
                    cos(pointCloudVec[i].point[j].azi) *
                    cos(pointCloudVec[i].point[j].ele);
                cloud.points[i * POINTNUM + j].y =
                    (pointCloudVec[i].point[j].range) *
                    sin(pointCloudVec[i].point[j].azi) *
                    cos(pointCloudVec[i].point[j].ele);
                cloud.points[i * POINTNUM + j].z =
                    (pointCloudVec[i].point[j].range) *
                    sin(pointCloudVec[i].point[j].ele);
                cloud.points[i * POINTNUM + j].h =
                    pointCloudVec[i].point[j].doppler;
                cloud.points[i * POINTNUM + j].s =
                    rcsCal(pointCloudVec[i].point[j].range,
                           pointCloudVec[i].point[j].azi,
                           pointCloudVec[i].point[j].snr, rcsBuf);
            }
        }
    }
    memset(histBuf, 0, sizeof(float) * int((vrMax - vrMin) / step));
    float vrEst = hist(pointCloudVec, histBuf, step);
    float tmp;
    for (int i = 0; i < pointCloudVec.size(); i++) {
        for (int j = 0; j < POINTNUM; j++) {
            if (abs(pointCloudVec[i].point[j].range) > 0) {
                tmp = cloud.points[i * POINTNUM + j].h -
                      vrEst * cos(pointCloudVec[i].point[j].azi);
                if (tmp < -errThr) {
                    cloud.points[i * POINTNUM + j].v = -1;
                } else if (tmp > errThr) {
                    cloud.points[i * POINTNUM + j].v = 1;
                } else {
                    cloud.points[i * POINTNUM + j].v = 0;
                }
            }
        }
    }
}

int main(int argc, char** argv) {
    // print argv
    for (int i = 0; i < argc; i++) {
        printf("argv[%d]: %s\n", i, argv[i]);
    }

    float* rcsBuf = (float*)malloc(1201 * sizeof(float));
    FILE* fp_rcs = fopen("data//rcs.dat", "rb");
    fread(rcsBuf, 1201, sizeof(float), fp_rcs);
    fclose(fp_rcs);

    ros::init(argc, argv, "altosRadar");
    ros::NodeHandle nh;

    if (argc < 2) {
        printf("Usage: %s <port>\n", argv[0]);
        return 0;
    }

    int port = atoi(argv[1]);
    if (port < 0 || port > 65535) {
        printf("Port number should be between 0 and 65535\n");
        return 0;
    }
    if (port == 0) {
        port = PORT;
    }

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(
        "altosRadar" + std::to_string(port), 1);
    ros::Publisher markerPub =
        nh.advertise<visualization_msgs::Marker>("points_number", 10);
    ros::Publisher originPub =
        nh.advertise<visualization_msgs::Marker>("origin", 10);

    // Object list publishers
    ros::Publisher pub_obj_id =
        nh.advertise<visualization_msgs::MarkerArray>("objectsTracking", 1);
    ros::Publisher pub_obj_bbox =
        nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(
            "objectsTrackingBbox", 1);
    ros::Publisher pub_obj_bbox_static =
        nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(
            "objectsTrackingBboxStatic", 1);
    ros::Publisher pub_obj_bbox_dynamic =
        nh.advertise<jsk_recognition_msgs::BoundingBoxArray>(
            "objectsTrackingBboxDynamic", 1);
    ros::Publisher pub_obj_arrow =
        nh.advertise<visualization_msgs::MarkerArray>("objectsTrackingArrow",
                                                      1);

    visualization_msgs::Marker origin;
    origin.header.frame_id = "altosRadar";
    origin.type = visualization_msgs::Marker::SPHERE;
    origin.action = visualization_msgs::Marker::ADD;
    origin.header.stamp = ros::Time::now();
    origin.pose.position.x = 0.;
    origin.pose.position.y = 0.;
    origin.pose.position.z = 0.;
    origin.pose.orientation.x = 0.;
    origin.pose.orientation.y = 0.;
    origin.pose.orientation.z = 0.;
    origin.pose.orientation.w = 1.;

    origin.scale.x = 3.;
    origin.scale.y = 3.;
    origin.scale.z = 3.;
    origin.color.r = 1.;
    origin.color.g = 1.;
    origin.color.b = 0.;
    origin.color.a = 1.;

    visualization_msgs::Marker marker;
    marker.ns = "basic_shapes";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.scale.z = 10;
    marker.color.b = 1.0f;
    marker.color.g = 1.0f;
    marker.color.r = 1.0f;
    marker.color.a = 1;
    geometry_msgs::Pose pose;
    pose.position.x = -5.;
    pose.position.y = 0.;
    pose.position.z = 0.;

    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZHSV> cloud;
    cloud.width = widthSet * 2;
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);

    struct sockaddr_in addr;
    struct sockaddr_in from;

    struct ip_mreq req;
    socklen_t len = sizeof(from);

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (-1 == sockfd) {
        perror("socket");
        return 0;
    }
    struct timeval timeout = {1, 300};
    setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout,
               sizeof(struct timeval));
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout,
               sizeof(struct timeval));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;
    int ret = bind(sockfd, (struct sockaddr*)&addr, sizeof(addr));
    if (-1 == ret) {
        perror("bind");
        return 0;
    }

    req.imr_multiaddr.s_addr = inet_addr("224.1.2.4");
    req.imr_interface.s_addr = inet_addr(/*"0.0.0.0"*/ "192.168.3.1");

    ret = setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &req, sizeof(req));
    if (ret < 0) {
        perror("setsockopt");
        return 0;
    }

    // Setup object list socket (port + 10)
    int obj_port = port + 10;
    int obj_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (obj_sockfd >= 0) {
        int reuse = 1;
        setsockopt(obj_sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        struct sockaddr_in obj_addr{};
        obj_addr.sin_family = AF_INET;
        obj_addr.sin_port = htons(obj_port);
        obj_addr.sin_addr.s_addr = htonl(INADDR_ANY);

        if (bind(obj_sockfd, (struct sockaddr*)&obj_addr, sizeof(obj_addr)) >=
            0) {
            struct ip_mreq obj_req{};
            obj_req.imr_multiaddr.s_addr = inet_addr("224.1.2.4");
            obj_req.imr_interface.s_addr = inet_addr("192.168.3.1");
            setsockopt(obj_sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &obj_req,
                       sizeof(obj_req));

            // Set non-blocking
            int flags = fcntl(obj_sockfd, F_GETFL, 0);
            if (flags >= 0) fcntl(obj_sockfd, F_SETFL, flags | O_NONBLOCK);

            ROS_INFO("Object list socket listening on port %d", obj_port);
        } else {
            perror("Object list socket bind failed");
            close(obj_sockfd);
            obj_sockfd = -1;
        }
    } else {
        perror("Object list socket creation failed");
    }

    // Object list configuration
    double marker_lifetime_sec = 0.17;
    double bbox_scale_factor = 1;

    int installFlag = -1;
    float step = 0.2;  // bin size for histogram
    vector<POINTCLOUD> pointCloudVec;
    POINTCLOUD pointCloudBuf;
    char* recvBuf = (char*)&pointCloudBuf;
    float* histBuf =
        (float*)malloc(sizeof(float) * int((vrMax - vrMin) / step));
    unsigned short curObjInd;
    struct timeval tv;
    struct tm tm;
    int cntPointCloud[2] = {0, 0};
    gettimeofday(&tv, NULL);
    localtime_r(&tv.tv_sec, &tm);
    char filePath[1024];
    unsigned char mode;
    sprintf(filePath, "data//%d_%d_%d_%d_%d_%d_altos.dat", tm.tm_year + 1900,
            tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    FILE* fp = fopen(filePath, "wb");
    bool sendFlag = 0;
    long tmpTime = pointCloudBuf.pckHeader.sec;
    FILE* fp_time = fopen("timeVal.txt", "wt");
    double init_time = -1;

    // Object list buffer
    std::uint8_t obj_buf[16384];
    struct sockaddr_in obj_from{};
    socklen_t obj_fromlen = sizeof(obj_from);

    while (ros::ok()) {
        originPub.publish(origin);

        // Poll object list socket (non-blocking)
        if (obj_sockfd >= 0) {
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(obj_sockfd, &rfds);
            timeval tv_select{0, 0};
            int r = select(obj_sockfd + 1, &rfds, nullptr, nullptr, &tv_select);
            if (r > 0 && FD_ISSET(obj_sockfd, &rfds)) {
                ssize_t n = recvfrom(obj_sockfd, obj_buf, sizeof(obj_buf), 0,
                                     (struct sockaddr*)&obj_from, &obj_fromlen);
                if (n >= static_cast<ssize_t>(kObjectsOffset)) {
                    const auto* rawHeader =
                        reinterpret_cast<const PCKHEADER*>(obj_buf);
                    if (rawHeader->header == 0xABCD4321u) {
                        std::size_t availableObjBytes =
                            static_cast<std::size_t>(n) - kObjectsOffset;
                        std::size_t objsInPacket =
                            availableObjBytes / sizeof(ObjectDet);
                        if (objsInPacket > 0 &&
                            objsInPacket <= rawHeader->objectCount) {
                            const auto* objs =
                                reinterpret_cast<const ObjectDet*>(
                                    obj_buf + kObjectsOffset);
                            ros::Time obj_stamp(rawHeader->sec,
                                                rawHeader->nsec);

                            // Prepare object visualization messages
                            visualization_msgs::MarkerArray id_markers;
                            visualization_msgs::MarkerArray arrow_markers;
                            jsk_recognition_msgs::BoundingBoxArray bbox_all;
                            bbox_all.header.frame_id = "altosRadar";
                            bbox_all.header.stamp = obj_stamp;
                            jsk_recognition_msgs::BoundingBoxArray bbox_static;
                            bbox_static.header = bbox_all.header;
                            jsk_recognition_msgs::BoundingBoxArray bbox_dynamic;
                            bbox_dynamic.header = bbox_all.header;

                            for (std::size_t i = 0; i < objsInPacket; ++i) {
                                const auto& o = objs[i];

                                // Bounding box
                                jsk_recognition_msgs::BoundingBox bbox;
                                bbox.header = bbox_all.header;
                                bbox.pose.position.x = o.x;
                                bbox.pose.position.y = o.y;
                                bbox.pose.position.z = o.z;
                                double yaw = std::atan2(o.vy, o.vx);
                                geometry_msgs::Quaternion q =
                                    tf::createQuaternionMsgFromYaw(yaw);
                                bbox.pose.orientation = q;
                                bbox.dimensions.x =
                                    o.length * bbox_scale_factor;
                                bbox.dimensions.y = o.width * bbox_scale_factor;
                                bbox.dimensions.z =
                                    o.height * bbox_scale_factor;
                                bbox_all.boxes.push_back(bbox);

                                bool is_dynamic = (o.motion_state == 0);
                                bool is_static = (o.motion_state == 1);
                                if (is_static)
                                    bbox_static.boxes.push_back(bbox);
                                if (is_dynamic) {
                                    jsk_recognition_msgs::BoundingBox dyn =
                                        bbox;
                                    dyn.dimensions.x = std::max(4.0f, o.length);
                                    dyn.dimensions.y = std::max(1.8f, o.width);
                                    dyn.dimensions.z = std::max(1.6f, o.height);
                                    bbox_dynamic.boxes.push_back(dyn);
                                }

                                // ID marker
                                visualization_msgs::Marker m;
                                m.header = bbox_all.header;
                                m.ns = "objectTracking";
                                m.id = o.id;
                                m.type = visualization_msgs::Marker::
                                    TEXT_VIEW_FACING;
                                m.action = visualization_msgs::Marker::ADD;
                                m.text = std::to_string(o.id);
                                m.scale.x = m.scale.y = m.scale.z = 2.8;
                                m.color.a = 1.0;
                                m.color.r = 1.0;
                                m.color.g = 1.0;
                                m.color.b = 0.0;
                                m.pose.position.x = o.x;
                                m.pose.position.y = o.y;
                                m.pose.position.z = o.z + 1.5;
                                m.pose.orientation.w = 1.0;
                                m.lifetime = ros::Duration(marker_lifetime_sec);
                                id_markers.markers.push_back(m);

                                if (is_dynamic) {
                                    visualization_msgs::Marker arrow;
                                    arrow.header = bbox_all.header;
                                    arrow.ns = "objectTrackingArrow";
                                    arrow.id = o.id;
                                    arrow.type =
                                        visualization_msgs::Marker::ARROW;
                                    arrow.action =
                                        visualization_msgs::Marker::ADD;
                                    arrow.scale.y = 0.5;
                                    arrow.scale.z = 0.2;
                                    arrow.scale.x = o.length + 1.0f;
                                    arrow.color.a = 1.0;
                                    arrow.color.r = 1.0;
                                    arrow.color.g = 0.0;
                                    arrow.color.b = 0.0;
                                    arrow.pose.position.x = o.x;
                                    arrow.pose.position.y = o.y;
                                    arrow.pose.position.z = o.z;
                                    arrow.pose.orientation = q;
                                    arrow.lifetime =
                                        ros::Duration(marker_lifetime_sec);
                                    arrow_markers.markers.push_back(arrow);
                                }
                            }

                            // Publish object messages
                            pub_obj_id.publish(id_markers);
                            pub_obj_bbox.publish(bbox_all);
                            pub_obj_bbox_static.publish(bbox_static);
                            pub_obj_bbox_dynamic.publish(bbox_dynamic);
                            pub_obj_arrow.publish(arrow_markers);

                            ROS_INFO("Published %zu objects, frameId %u",
                                     objsInPacket, rawHeader->frameId);
                        }
                    }
                }
            }
        }

        memset(recvBuf, 0, sizeof(POINTCLOUD));
        ret = recvfrom(sockfd, recvBuf, sizeof(POINTCLOUD), 0,
                       (struct sockaddr*)&from, &len);
        // printf("ret = %d\t%d\n", pointCloudBuf.pckHeader.objectCount, ret);
        if (ret > 0) {
            if ((pointCloudBuf.pckHeader.mode == 0 && cntPointCloud[1] > 0)) {
                if (pointCloudVec.size() * POINTNUM <
                    cntPointCloud[0] + cntPointCloud[1]) {
                    printf(
                        "FrameId %d Loss %lu pack(s) in %d "
                        "packs------------------------\n",
                        pointCloudBuf.pckHeader.frameId,
                        (unsigned long)(int(ceil(cntPointCloud[0] / POINTNUM) +
                                            ceil(cntPointCloud[1] / POINTNUM)) -
                                        pointCloudVec.size()),
                        int(ceil(cntPointCloud[0] / POINTNUM) +
                            ceil(cntPointCloud[1] / POINTNUM)));
                }

                // reset the cloud
                for (int k = 0; k < widthSet * 2; k++) {
                    cloud.points[k].x =
                        0;  //(pointCloudVec[i].point[j].range)*cos(pointCloudVec[i].point[j].azi)*cos(pointCloudVec[i].point[j].ele);
                    cloud.points[k].y =
                        0;  //(pointCloudVec[i].point[j].range)*sin(pointCloudVec[i].point[j].azi)*cos(pointCloudVec[i].point[j].ele);
                    cloud.points[k].z =
                        0;  //(pointCloudVec[i].point[j].range)*sin(pointCloudVec[i].point[j].azi)*cos(pointCloudVec[i].point[j].ele);
                    cloud.points[k].h =
                        0;  // pointCloudVec[i].point[j].doppler;
                    cloud.points[k].s =
                        0;  // rcsCal(pointCloudVec[i].point[j].range,pointCloudVec[i].point[j].azi,pointCloudVec[i].point[j].snr,rcsBuf);
                }
                calPoint(pointCloudVec, cloud, installFlag, rcsBuf, step,
                         histBuf);
                ros::Duration(0.005).sleep();
                pcl::toROSMsg(cloud, output);
                output.header.frame_id = "altosRadar";
                marker.header.frame_id = "altosRadar";
                marker.header.stamp = ros::Time::now();
                ostringstream str;
                str << "pointNum:" << cntPointCloud[0] + cntPointCloud[1];
                marker.text = str.str();
                marker.pose = pose;
                markerPub.publish(marker);
                output.header.stamp = ros::Time::now();
                pub.publish(output);
                pointCloudVec.clear();
                cntPointCloud[0] = 0;
                cntPointCloud[1] = 0;
                struct timeval tv;
                long long t1;
                gettimeofday(&tv, NULL);
                t1 = tv.tv_sec * 1000ll + tv.tv_usec / 1000;
                localtime_r(&tmpTime, &tm);
                fprintf(fp_time, "1:%f\n", t1 / 1e3);
            }
            fwrite(recvBuf, 1, ret, fp);
            curObjInd = pointCloudBuf.pckHeader.curObjInd;
            mode = pointCloudBuf.pckHeader.mode;
            cntPointCloud[mode] = pointCloudBuf.pckHeader.objectCount;
            pointCloudVec.push_back(pointCloudBuf);
            if ((mode == 1 && (curObjInd + 1) * POINTNUM >=
                                  pointCloudBuf.pckHeader.objectCount)) {
                if (pointCloudVec.size() * POINTNUM <
                    cntPointCloud[0] + cntPointCloud[1]) {
                    printf(
                        "FrameId %d %lu Loss %lu pack(s) in %d "
                        "packs------------------------\n",
                        pointCloudBuf.pckHeader.frameId,
                        (unsigned long)pointCloudVec.size(),
                        (unsigned long)(int(ceil(cntPointCloud[0] / POINTNUM) +
                                            ceil(cntPointCloud[1] / POINTNUM)) -
                                        pointCloudVec.size()),
                        int(ceil(cntPointCloud[0] / POINTNUM) +
                            ceil(cntPointCloud[1] / POINTNUM)));
                }

                // reset the cloud
                for (int k = 0; k < widthSet * 2; k++) {
                    cloud.points[k].x =
                        0;  //(pointCloudVec[i].point[j].range)*cos(pointCloudVec[i].point[j].azi)*cos(pointCloudVec[i].point[j].ele);
                    cloud.points[k].y =
                        0;  //(pointCloudVec[i].point[j].range)*sin(pointCloudVec[i].point[j].azi)*cos(pointCloudVec[i].point[j].ele);
                    cloud.points[k].z =
                        0;  //(pointCloudVec[i].point[j].range)*sin(pointCloudVec[i].point[j].azi)*cos(pointCloudVec[i].point[j].ele);
                    cloud.points[k].h =
                        0;  // pointCloudVec[i].point[j].doppler;
                    cloud.points[k].s =
                        0;  // rcsCal(pointCloudVec[i].point[j].range,pointCloudVec[i].point[j].azi,pointCloudVec[i].point[j].snr,rcsBuf);
                }

                calPoint(pointCloudVec, cloud, installFlag, rcsBuf, step,
                         histBuf);
                ros::Duration(0.005).sleep();
                pcl::toROSMsg(cloud, output);
                output.header.frame_id = "altosRadar";
                marker.header.frame_id = "altosRadar";
                marker.header.stamp = ros::Time::now();
                ostringstream str;
                double timestamp_sec =
                    double(pointCloudBuf.pckHeader.sec) +
                    double(pointCloudBuf.pckHeader.nsec) / 1e9;

                init_time = init_time < 0 ? timestamp_sec : init_time;
                timestamp_sec -= init_time;
                printf("timestamp_sec: %.6f\n", timestamp_sec);

                str << "pointNum:" << cntPointCloud[0] + cntPointCloud[1];
                printf("pointNum of %d frame: %d\n",
                       pointCloudBuf.pckHeader.frameId,
                       cntPointCloud[0] + cntPointCloud[1]);
                printf(
                    "----------------------------------------------------\n");

                marker.text = str.str();
                marker.pose = pose;
                markerPub.publish(marker);
                output.header.stamp = ros::Time::now();
                pub.publish(output);
                pointCloudVec.clear();
                cntPointCloud[0] = 0;
                cntPointCloud[1] = 0;
                struct timeval tv;
                long long t1;
                gettimeofday(&tv, NULL);
                t1 = tv.tv_sec * 1000ll + tv.tv_usec / 1000;
                localtime_r(&tmpTime, &tm);
                fprintf(fp_time, "%f\n", t1 / 1e3);
                // cloud.clear();
            }
        } else {
            init_time = -1;
            printf("recv failed (timeOut)   %d\n", ret);
        }
    }

    close(sockfd);
    if (obj_sockfd >= 0) close(obj_sockfd);
    free(histBuf);
    fclose(fp);
    fclose(fp_time);
    return 0;
}
