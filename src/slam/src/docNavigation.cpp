//
// Created by bill on 22-6-1.
//
#include "message_flag.h"
#include "doc_nav.h"

using namespace std;

int main(int argc, char **argv){
    ros::init(argc, argv, "Doc_Navigation");
    // init the subscriber
    DockNav DockNav;
    // waiting
    ros::spin();
//    DockNav.loop();
    return 0;
}