#include "math.h"
#include "ros/ros.h"
#include "doc_nav.h"
#include "message_flag.h"
#include "slam/pos.h"
#include "sensor_msgs/LaserScan.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "tf/transform_datatypes.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

DockNav::DockNav() {
    enable = false;
    startSpin = true;
    nav_radian_range = NAV_ANGLE_RANGE / 180.0 * 3.14159;
//        commandPub = node.advertise<>("", 10);
    movingFlag = node.subscribe("/pos/movingFlag", 10, &DockNav::flagToAction, this);
    movingFlagPub = node.advertise<slam::pos>("/pos/movingFlag", 10);
    stopNav = node.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 10);

    cout << "init finish ! " << endl;
}

bool DockNav::isNearCurrentGoal(geometry_msgs::PoseStamped newPos) {
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
void DockNav::moveForward() {
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = FORWARD_SPEED;
    ROS_INFO("SPEED : % f", msg.linear.x);
    commandPub.publish(msg);
}

// Spin base_link so that the robot face the dock
void DockNav::doSpinning() {
    static geometry_msgs::Twist msg;
    msg.angular.z = ANGULAR_VELOCITY * (dock_angle < 0 ? 1 : -1);
    ROS_INFO("Angular velocity : % f", msg.angular.z);
    commandPub.publish(msg);
}

void DockNav::callback(const geometry_msgs::PoseStamped& msg) {
    if (isNearCurrentGoal(msg)) {
        currentPos = msg;
    }
}

void DockNav::checkDockPosUpdate() {
    dock_angle = tf::getYaw(currentPos.pose.orientation);
    startSpin = dock_angle > 2 * PI - nav_radian_range || dock_angle < nav_radian_range;
}

void DockNav::flagToAction(const slam::pos &msg){
    if(msg.flag == START_NAV){
        this->getFrontPos();
    }else if(msg.flag == START_ROUTE){
        this->startDockNav();
    }
}

geometry_msgs::PoseStamped DockNav::getFrontPos(){
    double roll, pitch, yaw;
    tf::Quaternion angle_tmp;
    tf::quaternionMsgToTF(lastPos.pose.orientation,angle_tmp);
    tf::Matrix3x3(angle_tmp).getRPY(roll, pitch, yaw);
    cout << "roll is " << roll << " pitch is " << pitch << " yaw is " << yaw << endl;
}

void DockNav::loop(){
    while (node.ok()){
        ros::spinOnce();
    }
}

void DockNav::startDockNav() {
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

void DockNav::stopDockNav() {
    enable = false;
}

void DockNav::startFrontNav(){
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