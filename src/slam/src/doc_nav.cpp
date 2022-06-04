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
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_listener.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

DockNav::DockNav() {
    enable = true;
    startSpin = true;
    dock_angle = 0.0f;
    dock_dis = 100.0f;
    nav_radian_range = NAV_ANGLE_RANGE / 180.0 * 3.14159;
//        commandPub = node.advertise<>("", 10);
    commandPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    movingFlag = node.subscribe("/pos/movingFlag", 1000, &DockNav::flagToAction, this);
    docPos = node.subscribe("/pos/doc_pos", 1000, &DockNav::callback, this);
    nowPose = node.subscribe("tracked_pose", 1000, &DockNav::updateNowPos, this);
    movingFlagPub = node.advertise<slam::pos>("/pos/movingFlag", 1000);
    stopNav = node.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1000);
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
            updateCount = 1;
            return true;
        }
    }
    else {
        updateCount = 1;
        lastPos = newPos;
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
    msg.angular.z = ANGULAR_VELOCITY * (dock_angle > 0 ? 1 : -1);
    ROS_INFO("Angular velocity : % f", msg.angular.z);
    commandPub.publish(msg);
}

void DockNav::callback(const geometry_msgs::PoseStamped& msg) {
    if (isNearCurrentGoal(msg)) {
        ROS_INFO("Doc pos set!");
        currentPos = msg;

        // update base_CurrentGoal
        tf::TransformListener listener;
        currentPos.header.stamp = ros::Time();
        try {
            listener.waitForTransform("/base_link", "/map", ros::Time(0), ros::Duration(3.0));
            listener.transformPose("base_link", currentPos, base_CurrentPos);
            cout << base_CurrentPos << endl;
        } catch (tf::TransformException ex) {
            ROS_WARN("transfrom exception : %s", ex.what());
            return;
        }
        ROS_INFO("Dock base_link is set");
    }
    ROS_INFO("Now Update count is %d", updateCount);

    dock_angle = tf::getYaw(base_CurrentPos.pose.orientation);
    ROS_INFO("GetYaw: %f, range is %f, %f", dock_angle, (float)2 * PI - nav_radian_range, (float)nav_radian_range);
    startSpin = dock_angle < -nav_radian_range || dock_angle > nav_radian_range;

    float dock_dis = std::pow((nowPosInMap.pose.position.x - currentPos.pose.position.x), 2)
            + std::pow((nowPosInMap.pose.position.y - currentPos.pose.position.y), 2);
    ROS_INFO("dock_dis = %f, MIN_DOCK_DIS = %f", dock_dis, MIN_DOCK_DIS);

    enable = dock_dis > MIN_DOCK_DIS * MIN_DOCK_DIS;
}

void DockNav::flagToAction(const slam::pos &msg){
    if(msg.flag == START_NAV){
        this->startFrontNav();
    }else if(msg.flag == START_ROUTE){
        this->startDockNav();
    }
}

void DockNav::updateNowPos(const geometry_msgs::PoseStamped &msg){
    nowPosInMap = msg;
}

geometry_msgs::PoseStamped DockNav::getFrontPos(){
    double roll, pitch, yaw;
    geometry_msgs::PoseStamped frontDoc;
    tf::Quaternion angle_tmp;
    tf::quaternionMsgToTF(lastPos.pose.orientation,angle_tmp);
    tf::Matrix3x3(angle_tmp).getRPY(roll, pitch, yaw);
//    cout << "roll is " << roll << " pitch is " << pitch << " yaw is " << yaw << endl;
    yaw += PI;
    double front_len = 0.2;
    double delta_x = front_len * cos(yaw);
    double delta_y = front_len * sin(yaw);
    frontDoc.header.frame_id = lastPos.header.frame_id;
    frontDoc.pose.position.x = lastPos.pose.position.x + delta_x;
    frontDoc.pose.position.y = lastPos.pose.position.y + delta_y;
    frontDoc.pose.position.z = lastPos.pose.position.z;
    frontDoc.pose.orientation.x = lastPos.pose.orientation.x;
    frontDoc.pose.orientation.y = lastPos.pose.orientation.y;
    frontDoc.pose.orientation.z = lastPos.pose.orientation.z;
    frontDoc.pose.orientation.w = lastPos.pose.orientation.w;
    return frontDoc;
}

void DockNav::loop() {
    while (node.ok()){
        ros::spinOnce();
    }
}

void DockNav::startDockNav() {
    ros::Rate rate(10);
    while(ros::ok()){
        ros::spinOnce();

        if (!enable) {
            FORWARD_SPEED = 0;
            moveForward();
            cout << "Shutting down!" << endl;
            return;
        }

        if (startSpin) {
            doSpinning();
        } else {
            moveForward();
        }
        rate.sleep();
    }
}

void DockNav::stopDockNav() {
    enable = false;
}

void DockNav::startFrontNav(){
    // TODO : recvice a pos
    geometry_msgs::PoseStamped goal = getFrontPos();
    double range = 0.1;
    // TODO : set as global variable
    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal sendGoal;
    sendGoal.target_pose = goal;
    ac.sendGoal(sendGoal);
    ros::Rate rate(10);
    while(ros::ok()){
        ros::spinOnce();
        cout << "Now dis is " << sqrt(pow(goal.pose.position.x - this->nowPosInMap.pose.position.x, 2) + pow(goal.pose.position.y - this->nowPosInMap.pose.position.y, 2)) << endl;
        if(pow(goal.pose.position.x - this->nowPosInMap.pose.position.x, 2) + pow(goal.pose.position.y - this->nowPosInMap.pose.position.y, 2) < pow(range, 2)){
            cout << "Approaching goal, ready to stop" << endl;
            actionlib_msgs::GoalID sMsg;
            stopNav.publish(sMsg);
            // ready to send to startDocNav, but wait for a second
            sleep(2);
            // The request may send to robot, can start to startDocNav
//            slam::pos flagMsg;
//            flagMsg.flag = START_ROUTE;
//            movingFlagPub.publish(flagMsg);
            break;
        }
        rate.sleep();
    }
}