#include"scan2pointcloud/scan_to_pointcloud2_converter.h"
#include<limits>

ScanToPointCloud2::ScanToPointCloud2():private_node_("~"){
    ROS_INFO_STREAM("\033[1;32m---->Scan to PointCloud2 Converter.\033[0m");
    laser_scan_subscriber_=node_handle_.subscribe("scan",1,&ScanToPointCloud2::ScanCallback,this);

    pointcloud2_publisher_=node_handle_.advertise<PointCloudT>("pointcloud2_converted",1,this);

    invaild_point_.x=std::numeric_limits<float>::quiet_NaN();
    invaild_point_.y=std::numeric_limits<float>::quiet_NaN();
    invaild_point_.z=std::numeric_limits<float>::quiet_NaN();
}

ScanToPointCloud2::~ScanToPointCloud2()
{
    ROS_INFO("Destroying ScanToPointCloud2");
}

void ScanToPointCloud2::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg){
    PointCloudT::Ptr cloud_msg=boost::shared_ptr<PointCloudT>(new PointCloudT());
    cloud_msg->points.resize(scan_msg->ranges.size());
    for(unsigned int i = 0; i<scan_msg->ranges.size();++i){
        PointT &point_tmp=cloud_msg->points[i];
        float range=scan_msg->ranges[i];
        if(!std::isfinite(range)){
            point_tmp=invaild_point_;
            continue;
        }
        if(range>scan_msg->range_min&&range<scan_msg->range_max){
            float angle = scan_msg->angle_min+i*scan_msg->angle_increment;
            point_tmp.x=range*cos(angle);
            point_tmp.y=range*sin(angle);
            point_tmp.z=0.0;
        }
        else
        point_tmp=invaild_point_;
    }
    cloud_msg->width=scan_msg->ranges.size();
    cloud_msg->height=1;
    cloud_msg->is_dense=false;
    pcl_conversions::toPCL(scan_msg->header,cloud_msg->header);

    pointcloud2_publisher_.publish(cloud_msg);
}

int main(int argc, char** argv){
    ros::init(argc,argv,"scan2pointcloud2");
    ScanToPointCloud2 scantopointcloud2;

    ros::spin();
    return 0;
}