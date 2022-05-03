#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class Wander
{
public:
    // Tunable parameters 
    const double FORWARD_SPEED = 0.3;
    double angular_velocity;    // get angular velocity randomly
    const double MIN_SCAN_ANGLE = -15.0 / 180 * M_PI;   //-15 degree rad
    const double MAX_SCAN_ANGLE = +15.0 / 180 * M_PI;
    // Should be smaller than sensor_msgs::LaserScan::range_max
    const float MIN_DIST_FROM_OBSTACLE = 0.3;   //Minimum distance from an object
    Wander();
    void startMoving();

private:
    ros::NodeHandle node;
    ros::Publisher commandPub; // Publisher to the robot~s velocity command topic
    ros::Subscriber laserSub;  // Subscriber to the robot~s laser scan topic
    bool keepMoving;           // Indicates whether the robot should continue moving ，0 or 1
    bool keepMoving0;       //KeepMoving the previous state of
    bool getRandom;             //When keepMoving it changes from 1 to 0
    //Indicates whether the robot should keep moving, 0 or 1
    void moveForward();
    void turnCorner();//
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan); //Ptr is pointer
};
