#include<ros/ros.h>
#include<stdio.h>
#include<stdlib.h>
#include <sensor_msgs/PointCloud2.h> 
#include<string.h>
#include<errno.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<unistd.h>
#include"pointCloud.h"
#include <pcl/point_cloud.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include<math.h>
#include <sys/time.h>
#include <algorithm>
using namespace std;
#define widthSet 4220
#define PORT 4040
#define vrMax 60
#define vrMin -60
#define errThr 3
float hist(float *vr,float *histBuf,float step,int vrInd)
{
    int ind = 0;
    for(int i = 0;i<vrInd;i++)
    {
        ind = (vr[i] - vrMin)/step;
        if(vr[i]>60||vr[i]<-60)
        {
            // printf("vr[%d] = %f\n",ind,vr[i]);
            continue;
        }

        histBuf[ind]++;
    }
    return float((max_element(histBuf,histBuf+(int((vrMax-vrMin)/step))) - histBuf))*step+vrMin;
}
int main(int argc,char **argv)
{
    
    ros::init(argc, argv, "altosRadar");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("altosRadar", 1);
    sensor_msgs::PointCloud2 output; 
    pcl::PointCloud<pcl::PointXYZHSV> cloud; 
    printf("---------------------------\n");
    cloud.width = widthSet; 
    cloud.height = 1; 
    cloud.points.resize(cloud.width * cloud.height);
    
    struct sockaddr_in addr;
    struct sockaddr_in from;

    struct ip_mreq req;
    socklen_t len = sizeof(from);
 
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (-1 == sockfd)
    {
        perror("socket"); 
        return 0;
    }
    struct timeval timeout = {1,300}; 
    setsockopt(sockfd,SOL_SOCKET,SO_SNDTIMEO,(char *)&timeout,sizeof(struct timeval));
    setsockopt(sockfd,SOL_SOCKET,SO_RCVTIMEO,(char *)&timeout,sizeof(struct timeval));
 
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);
    addr.sin_addr.s_addr = INADDR_ANY;
    int ret = bind(sockfd, (struct sockaddr *)&addr, sizeof(addr));
    if (-1 == ret)
    {
        perror("bind"); 
        return 0;
    }
 
    req.imr_multiaddr.s_addr = inet_addr("224.1.2.4");
    req.imr_interface.s_addr = inet_addr(/*"0.0.0.0"*/"192.168.3.1");;
    ret = setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &req, sizeof(req));
    if (ret < 0)
    {
        perror("setsockopt"); 
        return 0;
    }
    
    int recvFrameLen = 0;
	int frameNum = 0;
    int tmp;
    POINTCLOUD pointCloudBuf;
	char* recvBuf= (char*)&pointCloudBuf;
    struct timeval  tv;
	struct tm tm;

    gettimeofday(&tv, NULL);
    localtime_r(&tv.tv_sec, &tm);
    char filePath[1024];
    sprintf(filePath,"%d_%d_%d_%d_%d_%d.dat",tm.tm_year + 1900,tm.tm_mon + 1,tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec);
    FILE *fp = fopen(filePath,"wb");
    int frameId = 0;
    int objectCntFrame = 0;
    int i;
    int objectCntLast,objectCnt;
    float offsetRange = 0;
    float offsetAzi = 0.0*3.1415926/180;
    float offsetEle = -0.0*3.1415926/180;
    float vr[widthSet];
    float vrAzi[widthSet];
    float step = 0.2;
    float *histBuf = (float*)malloc(sizeof(float)*int((vrMax-vrMin)/step));
    int vrInd = 0;
    float vrEst = 0;
    int cntFrameobj = 30;

    while(ros::ok())
    {
        ret = recvfrom(sockfd, recvBuf, 1440, 0, (struct sockaddr *)&from, &len);
        if (ret > 0)
		{
            
            fwrite(recvBuf, 1, ret, fp);

            // printf("pointCloudBuf.pckHeader.objectCount = %d \tpckHeader.curObjNum = %d\n",pointCloudBuf.pckHeader.curObjInd,pointCloudBuf.pckHeader.curObjNum);
 
            // long tmpTime = pointCloudBuf.pckHeader.sec;
            // localtime_r(&tmpTime, &tm);
            // printf("%d_%d_%d_%d_%d_%d\n",tm.tm_year + 1900,tm.tm_mon + 1,tm.tm_mday,tm.tm_hour,tm.tm_min,pointCloudBuf.pckHeader.sec);
            pointCloudBuf.pckHeader.curObjNum = pointCloudBuf.pckHeader.curObjNum/44;
            objectCnt = pointCloudBuf.pckHeader.objectCount;
            pointCloudBuf.pckHeader.curObjInd = pointCloudBuf.pckHeader.curObjInd*30;
            tmp = pointCloudBuf.pckHeader.frameId;
            // if(pointCloudBuf.pckHeader.mode==1)
            // {
            //     continue;
            // }
            if(frameId == 0 || frameId == tmp)
            {
                frameId = tmp;
                for(i = 0;i<pointCloudBuf.pckHeader.curObjNum;i=i+1)
                {
                    if(abs(pointCloudBuf.point[i].range)>0)
                    {
                        pointCloudBuf.point[i].ele = -(pointCloudBuf.point[i].ele -offsetEle);
                        pointCloudBuf.point[i].azi = asin(sin(pointCloudBuf.point[i].azi)/cos(pointCloudBuf.point[i].ele));
                        cloud.points[pointCloudBuf.pckHeader.curObjInd+i].x = (pointCloudBuf.point[i].range-offsetRange)*cos(pointCloudBuf.point[i].azi-offsetAzi)*cos(pointCloudBuf.point[i].ele); 
                        cloud.points[pointCloudBuf.pckHeader.curObjInd+i].y = (pointCloudBuf.point[i].range-offsetRange)*sin(pointCloudBuf.point[i].azi-offsetAzi)*cos(pointCloudBuf.point[i].ele);; 
                        cloud.points[pointCloudBuf.pckHeader.curObjInd+i].z = (pointCloudBuf.point[i].range-offsetRange)*sin(pointCloudBuf.point[i].ele) ; 
                        cloud.points[pointCloudBuf.pckHeader.curObjInd+i].h = pointCloudBuf.point[i].doppler; 
                        cloud.points[pointCloudBuf.pckHeader.curObjInd+i].s = 10*log10(pointCloudBuf.point[i].snr); 
                        vr[pointCloudBuf.pckHeader.curObjInd+i] = pointCloudBuf.point[i].doppler/cos(pointCloudBuf.point[i].azi-offsetAzi);
                        vrAzi[pointCloudBuf.pckHeader.curObjInd+i] = pointCloudBuf.point[i].azi;
                    }
                }
                vrInd = pointCloudBuf.pckHeader.curObjInd + POINTNUM;
                objectCntFrame = pointCloudBuf.pckHeader.curObjInd + POINTNUM;
                objectCntLast = objectCnt;
                cntFrameobj = cntFrameobj + 30;

            }else{
                if(cntFrameobj<objectCntLast)
                {
                    printf("-------------------------dataLoss %d\t%d\t%d pack(s) in %d packs------------------------\n",cntFrameobj,objectCntLast,(objectCntLast - cntFrameobj)/POINTNUM,objectCntLast/POINTNUM);
                }
                memset(histBuf,0,sizeof(float)*int((vrMax-vrMin)/step));
                vrEst = hist(vr,histBuf,step,vrInd);
                printf("Frame %d: objectCnt is %d\n",frameId,objectCntLast);
                cntFrameobj = 30;
                objectCntLast = objectCnt;
                for(i = 0;i<vrInd;i++)
                {
                    cloud.points[i].v = cloud.points[i].h - vrEst*cos(vrAzi[i]);
                    if(cloud.points[i].v<-errThr)
                    {
                        cloud.points[i].v = -1;
                    }else if(cloud.points[i].v>errThr)
                    {
                        cloud.points[i].v = 1;
                    }else{
                        cloud.points[i].v = 0;
                    }
                }
                ros::Duration(0.005).sleep();
                pcl::toROSMsg(cloud, output); 
                output.header.frame_id = "altosRadar"; 
                pub.publish(output);
                frameId = tmp;
                for(int i = 0;i<widthSet;i++)
                {
                    cloud.points[i].x = 0;
                    cloud.points[i].y = 0;
                    cloud.points[i].z = 0;
                    cloud.points[i].h = 0;
                    cloud.points[i].s = 0;
                    cloud.points[i].v = 0;
                }
                for(int i = 0;i<pointCloudBuf.pckHeader.curObjNum;i++)
                {
                    if(abs(pointCloudBuf.point[i].range)>0)
                    {
                        pointCloudBuf.point[i].ele = -(pointCloudBuf.point[i].ele -offsetEle);
                        pointCloudBuf.point[i].azi = asin(sin(pointCloudBuf.point[i].azi)/cos(pointCloudBuf.point[i].ele));
                        cloud.points[pointCloudBuf.pckHeader.curObjInd+i].x = (pointCloudBuf.point[i].range-offsetRange)*cos(pointCloudBuf.point[i].azi-offsetAzi); 
                        cloud.points[pointCloudBuf.pckHeader.curObjInd+i].y = (pointCloudBuf.point[i].range-offsetRange)*sin(pointCloudBuf.point[i].azi-offsetAzi);;
                        cloud.points[pointCloudBuf.pckHeader.curObjInd+i].z = (pointCloudBuf.point[i].range-offsetRange)*sin(pointCloudBuf.point[i].ele) ; 
                        cloud.points[pointCloudBuf.pckHeader.curObjInd+i].h = pointCloudBuf.point[i].doppler; 
                        cloud.points[pointCloudBuf.pckHeader.curObjInd+i].s = 10*log10(pointCloudBuf.point[i].snr); 
                        vr[pointCloudBuf.pckHeader.curObjInd+i] = pointCloudBuf.point[i].doppler/cos(pointCloudBuf.point[i].azi-offsetAzi);
                        vrAzi[pointCloudBuf.pckHeader.curObjInd+i] = pointCloudBuf.point[i].azi;    
                    }
                }
                vrInd = pointCloudBuf.pckHeader.curObjInd + POINTNUM;
            }
		}else
        {
            printf("recv failed (timeOut)   %d\n",ret);
        }
    }
    close(sockfd);
    free(histBuf);
    fclose(fp);
    return 0;
}