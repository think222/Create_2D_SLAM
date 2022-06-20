#ifndef SCAN_MATCH_PLICP_H
#define SCAN_MATCH_PLICP_H

#include<cmath>
#include<vector>
#include<chrono>

//ros
#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<geometry_msgs/TwistStamped.h>

//tf2
#include<tf2/utils.h>
#include<tf2/LinearMath/Transform.h>
#include<tf2_ros/transform_listener.h>
#include"tf2_ros/transform_broadcaster.h"

//csm
#include<csm/csm_all.h>
#undef min
#undef max

class ScanMatchPLICP{
private:
    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_;
    ros::Subscriber laser_scan_subscriber_;

    ros::Time last_icp_time_;

    bool initialized_;

    std::vector<double> a_cos_;
    std::vector<double> a_sin_;

    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point end_time_;
    std::chrono::duration<double> time_used_;

    //csm
    sm_params input_;
    sm_result output_;
    LDP prev_ldp_scan_;

    void InitParams();
    void CreateCache(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
    void LaserScanToLDP(const sensor_msgs::LaserScan::ConstPtr &scan_msg,LDP &ldp);
    void ScanMatchWithLDPICP(LDP &curr_ldp_scan,const ros::Time &time);

public:
    ScanMatchPLICP();
    ~ScanMatchPLICP();

    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
};

#endif
