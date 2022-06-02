#include "Wander.h"
#include "geometry_msgs/Twist.h"
#include "slam/pos.h"
#include<stdlib.h>  //random number
#include<time.h>
#include "message_flag.h"
#define random(x) (rand()%x)

Wander::Wander()
{
    keepMoving = true;
    keepMoving0 = true;
    getRandom = false;
    startMovingFlag = false;
    // Advertise a new publisher for the robots velocity command topic
    commandPub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // Subscribe to the simulated robots laser scan topic
    laserSub = node.subscribe("/scan", 10, &Wander::scanCallback, this);
    // The subscriber to recvice the flag
    movingFlag = node.subscribe("/pos/movingFlag", 10, &Wander::ChangeFlag, this);
}
// Send a velocity command
void Wander::moveForward()
{
    geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    msg.linear.x = FORWARD_SPEED;
    ROS_INFO("SPEED : % f", msg.linear.x);
    commandPub.publish(msg);
}

void Wander::turnCorner()
{
    static geometry_msgs::Twist msg; // The default constructor will set all commands to 0
    if(keepMoving0==1 && keepMoving==0)
    getRandom = 1;
    else
    getRandom = 0;

    if(getRandom)   //If you are turning, you do not need to regenerate random numbers, only the first moment of turning generates random numbers
    {
        angular_velocity = 0.2;
        msg.angular.z = angular_velocity;
    }
    ROS_INFO("Angular velocity : % f", msg.angular.z);
    commandPub.publish(msg);
}

// Process the incoming laser scan message
void Wander::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    keepMoving0 = keepMoving;
    bool isObstacleInFront = false;
    keepMoving = true;
    // Find the closest range between the defined minimum and maximum angles

    int minIndex = 360 + floor((MIN_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    //floor() rounds down
    //Change the radian system to 360 degree system, here minndex is 345 degrees when MIN_SCAN_ANGLE = -15.0 / 180 * M_PI
    //Function: Returns the largest integer less than the incoming parameter
    int maxIndex = floor((MAX_SCAN_ANGLE - scan->angle_min) / scan->angle_increment);
    //Function: scan->angle_min indicates the rad that starts during the actual scan
    
    std::vector<std::pair<float, float>> scaled_ranges;
    for (int i = minIndex; i < 360; i++)
        scaled_ranges.push_back(std::make_pair(i, scan->ranges[i]));
    for (int i = 0; i <= maxIndex; i++)
        scaled_ranges.push_back(std::make_pair(i, scan->ranges[i]));
    for (auto value : scaled_ranges)
    {
//        ROS_INFO("%f", value.second);
        // std::cout << value.first << ":" << value.second << ",";
        //Should be the first and second value of value
        //The first value represents the angle 345-15 and the second represents the distance
        if (value.second < MIN_DIST_FROM_OBSTACLE && value.second != 0)
        {
            keepMoving = false;
            ROS_INFO("Turn a corner!");
            break;
        }
    }
    
     //std::cout << std::endl;
}

void Wander::ChangeFlag(const slam::pos &msg){
    if(msg.flag == START_WANDERING){
        this->startMovingFlag = true;
    }
    else if(msg.flag == STOP_WANDERING){
        this->stopMoving();
    }
}


void Wander::startMoving()
{
    srand((int)time(0));
    startMovingFlag = true;
    ros::Rate rate(10);
    ROS_INFO("Start moving");
    // Keep spinning loop until user presses Ctrl+C or the robot got too close to an obstacle
    while (ros::ok())
    {
        // Need to call this function often to allow ROS to process incoming messages
        ros::spinOnce();

        // the variable to info and stop
        if(startMovingFlag){
            if (!keepMoving)
                turnCorner();
            else
                moveForward();
        }
        rate.sleep();
    }
}

void Wander::stopMoving(){
    startMovingFlag = false;
    sleep(1);
    geometry_msgs::Twist msg;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    commandPub.publish(msg);
    ROS_INFO("Stop Moving!");
}