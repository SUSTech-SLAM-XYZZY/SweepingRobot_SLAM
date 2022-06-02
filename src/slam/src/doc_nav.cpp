#include "math.h"
#include "ros/ros.h"
#include "message_flag.h"
#include "slam/pos.h"
#include "sensor_msgs/LaserScan.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "tf/transform_datatypes.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

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
    bool isNearCurrentGoal(geometry_msgs::PoseStamped newPos) {
        double x_ = newPos.pose.position.x - lastPos.pose.position.x;
        double y_ = newPos.pose.position.y - lastPos.pose.position.y;
        double z_ = newPos.pose.position.z - lastPos.pose.position.z;
        if ((std::pow(x_, 2) + std::pow(y_, 2) + std::pow(z_, 2)) <= std::pow(tolerance, 2)) {
            if (updateCount < 3) {
                if (updateCount == 0) {
                    lastPos = newPos;
                }
                updateCount++;
                return false;
            }
            else {
                updateCount = 0;
                return true;
            }
        }
        else {
            updateCount = 0;
            return false;
        }
    }

    // Send a velocity command
    void moveForward() {
        geometry_msgs::Twist msg; // The default constructor will set all commands to 0
        msg.linear.x = FORWARD_SPEED;
        ROS_INFO("SPEED : % f", msg.linear.x);
        commandPub.publish(msg);
    }

    // Spin base_link so that the robot face the dock
    void doSpinning() {
        static geometry_msgs::Twist msg;
        msg.angular.z = ANGULAR_VELOCITY * (dock_angle < 0 ? 1 : -1);
        ROS_INFO("Angular velocity : % f", msg.angular.z);
        commandPub.publish(msg);
    }

    void callback(const geometry_msgs::PoseStamped& msg) {
        if (isNearCurrentGoal(msg)) {
            currentPos = msg;
        }
    }

    void checkDockPosUpdate() {
        dock_angle = tf::getYaw(currentPos.pose.orientation);
        startSpin = dock_angle > nav_radian_range || dock_angle < nav_radian_range;
    }

    void flagToAction(const slam::pos &msg){
        if(msg.flag == START_NAV){
            this->startFrontNav();
        }else if(msg.flag == START_ROUTE){
            this->startDockNav();
        }
    }

public:

    DockNav() {
        enable = false;
        startSpin = true;
        nav_radian_range = NAV_ANGLE_RANGE / 180.0 * 3.14159;
//        commandPub = node.advertise<>("", 10);
        movingFlag = node.subscribe("/pos/movingFlag", 10, flagToAction, this);
        movingFlagPub = node.advertise<slam::pos>("/move_base/cancel", 10);
        stopNav = node.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10);
    }


    void startDockNav() {
        //tell the action client that we want to spin a thread by default
        MoveBaseClient ac("move_base", true);

        //wait for the action server to come up
        while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }

        ros::Rate rate(10);
        ROS_INFO("Go to dock now...");

        while (ros::ok()) {
            ros::spinOnce();

            // TODO: switch state
            if (enable) {
                // TODO: publish position to Nav
                geometry_msgs::Twist msg;
                msg.linear.x = FORWARD_SPEED;
                ROS_INFO("SPEED : % f", msg.linear.x);
                commandPub.publish(msg);
                ROS_INFO("Sending goal");
                move_base_msgs::MoveBaseGoal currentGoal;

                ac.sendGoal(currentGoal);
                ac.waitForResult();
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    ROS_INFO("Hooray, the base moved 1 meter forward");
                else
                    ROS_INFO("The base failed to move forward 1 meter for some reason");

                // finish and exit
                return;
            }
        }
    }

    void stopDockNav() {
        enable = false;
    }

    void startFrontNav(){
        // TODO : recvice a pos
        geometry_msgs::PoseStamped goal;
        geometry_msgs::PoseStamped mypos;
        double range = 0.1;
        if(pow(goal.pose.position.x - mypos.pose.position.x, 2) + pow(goal.pose.position.y - mypos.pose.position.y, 2) < pow(range, 2)){
            actionlib_msgs::GoalID sMsg;
            stopNav.publish(sMsg);
            // ready to send to startDocNav, but wait for a second
            sleep(2);
            // The request may send to robot, can start to startDocNav
            slam::pos flagMsg;
            flagMsg.flag = 3;
            movingFlagPub.publish(flagMsg);
        }
    }
};