#include "dock_nav/dock_finder.h"

DockFinder::DockFinder() {
    cur_pos.dist = nullptr;
    cur_pos.angle = nullptr;

    // Subscribe to the dock finder part 
    // TODO:size?
    laserSub = node.subscribe("/pos/angle_dist", 6, &DockFinder::setDockPos, this)

}

void DockFinder::setDockPos(float32[] dist, float32[] angle) {
    cur_pos.dist = dist;
    cur_pos.angle = angle;
}

void DockFinder::callback(const std_msgs::Float32_::ConstPtr& msg) {
    // TODO: msg
    // setDockPos(msg->dist, msg->angle);
}