#pragma once

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "dock_finder.h"

class DockNav {
public:

    DockNav();
    void startDockNav();
    void stopDockNav();

private:
    bool enable; // check enable
    ros::NodeHandle node;
    ros::Publisher commandPub; // Publisher to the robot's velocity command topic
    // ros::Subscriber laserSub;  // Subscriber to the robot's laser scan topic

    DockFinder dockFinder;

}

