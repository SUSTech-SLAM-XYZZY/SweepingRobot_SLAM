#include "dock_nav/dock_nav.h"

DockNav::DockNav() {
    enable = false;
    // init publish to Nav
    commandPub = node.advertise<>("", 10);
}

void DockNav::startDockNav() {
    if (enable) {
        // TODO: publish position to Nav
        geometry_msgs::Twist msg;
        msg.linear.x = FORWARD_SPEED;
        ROS_INFO("SPEED : % f", msg.linear.x);
        commandPub.publish(msg);
    }
}

void DockNav::stopDockNav() {
    enable = false;
}