//
// Created by bill on 22-6-2.
//
#include "ros/ros.h"
#include "slam/pos.h"
#include "geometry_msgs/PoseStamped.h"

#ifndef SLAM_DOC_NAV_H
#define SLAM_DOC_NAV_H

class DockNav {
private:
    const double FORWARD_SPEED = 0.3;
    const double ANGULAR_VELOCITY = 0.4;
    const double NAV_ANGLE_RANGE = 30.0;   // keep dock in ± this angle

    double nav_radian_range;

    bool enable; // check enable
    bool startSpin;
    ros::NodeHandle node;
    ros::Publisher commandPub; // Publisher to the robot's velocity command topic
    ros::Publisher stopNav; // Publisher to stop Nav when in the range of 0.1m
    ros::Publisher movingFlagPub;   // maybe is useful
    ros::Subscriber laserSub;  // Subscriber to the robot's laser scan topic
    ros::Subscriber movingFlag; // To nav or route

    geometry_msgs::PoseStamped currentPos;

    float dock_angle;

    int updateCount = 0;
    double tolerance = 0.5;     // 5cm
    geometry_msgs::PoseStamped lastPos;
    // check if new Pos is in range
    bool isNearCurrentGoal(geometry_msgs::PoseStamped newPos);

    // Send a velocity command
    void moveForward();

    // Spin base_link so that the robot face the dock
    void doSpinning();

    void callback(const geometry_msgs::PoseStamped& msg);

    void checkDockPosUpdate();

    void flagToAction(const slam::pos &msg);

public:
    DockNav();
    void startDockNav();
    void stopDockNav();
    void startFrontNav();
};

#endif //SLAM_DOC_NAV_H