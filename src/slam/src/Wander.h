#include "ros/ros.h"
#include "slam/pos.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"

class Wander
{
public:
    // Tunable parameters 
    double FORWARD_SPEED = 0.1;
    double angular_velocity;    // get angular velocity randomly
    long timeout;                // the time to slow down
    const double MIN_SCAN_ANGLE = -30.0 / 180 * M_PI;   //-15 degree rad
    const double MAX_SCAN_ANGLE = +30.0 / 180 * M_PI;
    // Should be smaller than sensor_msgs::LaserScan::range_max
    const float MIN_DIST_FROM_OBSTACLE = 0.3;   //Minimum distance from an object
    Wander();
    void startMoving();
    void stopMoving();

private:
    ros::NodeHandle node;
    ros::Publisher commandPub; // Publisher to the robots velocity command topic
    ros::Subscriber laserSub;  // Subscriber to the robots laser scan topic
    ros::Subscriber movingFlag; // The subscriber to recvice the flag
    ros::Subscriber slowDownSignal; // slow down when recognize doc
    bool keepMoving;           // Indicates whether the robot should continue moving ，0 or 1
    bool keepMoving0;       //KeepMoving the previous state of
    bool startMovingFlag;   // The flag to control flag
    bool getRandom;             //When keepMoving it changes from 1 to 0
    //Indicates whether the robot should keep moving, 0 or 1
    void moveForward();
    void turnCorner(); //
    void ChangeFlag(const slam::pos &msg);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan); //Ptr is pointer
    void slowDown(const geometry_msgs::PoseStamped &msg);    // slow down when recognize doc
};
