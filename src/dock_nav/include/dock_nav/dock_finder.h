#pragma once

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

// dock position
struct DockPos {
    float32* dist;
    float32* angle;
    float32 left_dist;
    float32 mid_dist;
    float32 right_dist;
};

class DockFinder
{
public:
    struct DockPos cur_pos;
    DockFinder();
    void startUpdate(); // start update position

private:

    ros::NodeHandle node;
    // ros::Publisher commandPub; // Publisher to the robot's velocity command topic
    ros::Subscriber laserSub;  // Subscriber to the robot's laser scan topic

    void callback(const std_msgs::Float32_::ConstPtr& msg);
    void setDockPos(float32[] dist, float32[] angle);
};