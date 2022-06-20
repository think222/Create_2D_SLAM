#ifndef SCAN_TO_POINTCLOUD2_CONVERTED_H
#define SCAN_TO_POINTCLOUD2_CONVERTED_H

//ros
#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>

//pcl_ros
#include<pcl_ros/point_cloud.h>

//pcl
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

class ScanToPointCloud2{
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;
private:
    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_;
    ros::Subscriber laser_scan_subscriber_;
    ros::Publisher pointcloud2_publisher_;
    PointT invaild_point_;
public:
    ScanToPointCloud2();
    ~ScanToPointCloud2();
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msgs);
};

#endif
