//
// Created by bill on 22-5-10.
//
#include <cstdio>
#include <cmath>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <stack>

#define PI 3.14159

using namespace std;
const int INF = 0x3f3f3f3f;
const int len_board = 250;
const int point_num = 500;
vector<double> k;

struct Point {
    int dist;
    float angle;
    int flag;
    int syn_quality;
    float x;
};

struct Features {
    vector<Point> list;
    int stages = 0;
    int Mean = 0;
    float last = 0;
} FT;

struct position{
    double x = 0;
    double y = 0;
    double z = 0;
} nowP;

struct orientation{
    double x = 0;
    double y = 0;
    double z = 0;
    double w = 0;
} nowOrientation;

void callback_func(const sensor_msgs::LaserScan &msg);
//vector<cv::Point2d> points = {};
pcl::PointCloud<pcl::PointXY>::Ptr cloud(new pcl::PointCloud<pcl::PointXY>);

vector<vector<Point *> > *calc_knn(float dist, float angle, int flag, int syn_quality);

void print_cluster(vector<vector<Point *>> cluster);

vector<vector<Point *> > *straightLine(vector<vector<Point *>> cluster);

vector<vector<Point *> > strength_cluster(vector<Point *> line);

vector<vector<Point *> > *find_Features_cluster_tmp(vector<vector<Point *> > *cluster);

void sendRosInfo(vector<vector<Point *> > cluster_syn, double k);

void update_pos(const geometry_msgs::PoseStamped &msg);

// set to global
ros::Publisher angle_dist;
ros::Subscriber nowPos;

int main(int argc, char** argv) {
    ros::init(argc, argv, "recognize_rcv"); //初始化订阅者节点
    ros::NodeHandle n;
    ros::Subscriber ros_tutorial_sub = n.subscribe("scan", 1000, callback_func);
    nowPos = n.subscribe("tracked_pose", 1000, update_pos);
    angle_dist = n.advertise<geometry_msgs::PoseStamped>("/pos/angle_dist", 10);
    ros::spin();
    return 0;
}

void update_pos(const geometry_msgs::PoseStamped &msg){
    nowP.x = msg.pose.position.x;
    nowP.y = msg.pose.position.y;
    nowP.z = msg.pose.position.z;
    nowOrientation.x = msg.pose.orientation.x;
    nowOrientation.y = msg.pose.orientation.y;
    nowOrientation.z = msg.pose.orientation.z;
    nowOrientation.w = msg.pose.orientation.w;
//    cout << "Position update success!" << endl;
}

void callback_func(const sensor_msgs::LaserScan &msg){
    float angle_min = msg.angle_min;
    float angle_max = msg.angle_max;
    float angle_increment = msg.angle_increment;
    float range_min = msg.range_min;
    float range_max = msg.range_max;
    vector<float> ranges = msg.ranges;
    vector<float> intensities = msg.intensities;
    int data_num = ranges.size();
    float angle = angle_min;
    for (int i = 0; i < data_num; ++i) {
        int flag = ranges.at(i) < range_min ? 1 : 0;
        vector<vector<Point *> > *tmp = calc_knn(ranges.at(i), angle, flag,
                                                 intensities.at(i));
        vector<vector<Point *> > *result = find_Features_cluster_tmp(tmp);
        if (!tmp->empty()) {
            cout << "===============find feature!!!================" << endl;
//            print_cluster(*result);
            cout << "===============end of feature!================" << endl;
            FT.list.clear();
            cloud->clear();
        }
        tmp->clear();
        delete tmp;
        result->clear();
        delete result;
        // angle
        angle += angle_increment;
        if(angle > angle_max)angle = angle_max;
    }
}

float calc_dist(int dist_1, float angle_1, int dist_2, float angle_2) {
    float x_1 = dist_1 * sin(angle_1 * PI);
    float y_1 = dist_1 * cos(angle_1 * PI);
    float x_2 = dist_2 * sin(angle_2 * PI);
    float y_2 = dist_2 * cos(angle_1 * PI);
    if (x_1 == 0 or x_2 == 0) {
        return (float) -INF;
    } else {
        return sqrt(pow(x_1 - x_2, 2) + pow(y_1 - y_2, 2));
    }
}

vector<vector<Point *> > *calc_knn(float dist, float angle, int flag, int syn_quality) {
    Point tmp;
    angle = angle * 180 / M_PI;
    dist = dist * 1000;
    tmp.angle = angle;
    tmp.dist = dist;
    tmp.flag = flag;
    tmp.syn_quality = syn_quality;
    tmp.x = dist * sin(angle * PI / 180);
//    cout << tmp.angle << " " << tmp.dist << endl;
    vector<vector<Point *>> cluster = {};
    if (flag == 0) {
        FT.list.push_back(tmp);
//        points.emplace_back(sin(angle * PI / 180) * dist, cos(angle * PI / 180) * dist);
        pcl::PointXY point;
        point.x = (float) (sin(angle * PI / 180) * dist);
        point.y = (float) (cos(angle * PI / 180) * dist);
        cloud->push_back(point);
    }
    if (FT.list.size() == point_num) {
        int vis[point_num];
        for (int i = 0; i < point_num; i++) {
            vis[i] = 0;
        }

        pcl::KdTreeFLANN<pcl::PointXY> kdtree;

        kdtree.setInputCloud(cloud);

        for (int i = 0; i < point_num; i++) {
            if (vis[i] == 0) {
                std::vector<Point *> C = {};
                stack<int> Q;
                Q.push(i);
                while (!Q.empty()) {
                    int top = Q.top();
                    Point out = FT.list.at(top);
                    Q.pop();
                    if (vis[top] != 2) {
                        vis[top] = 2;
                        auto out_x = (float) (out.dist * sin(out.angle * PI / 180));
                        auto out_y = (float) (out.dist * cos(out.angle * PI / 180));
                        pcl::PointXY searchPoint;
                        std::vector<int> pointIdxRadiusSearch;
                        std::vector<float> pointRadiusSquaredDistance;

                        searchPoint.x = out_x;
                        searchPoint.y = out_y;

                        float radius = 60;

//                        std::cout << "Neighbors within radius search at (" << searchPoint.x
//                                  << " " << searchPoint.y
//                                  << ") with radius=" << radius << std::endl;

                        if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) >
                            0) {
                            for (std::size_t y = 0; y < pointIdxRadiusSearch.size(); ++y) {
                                if (vis[pointIdxRadiusSearch[y]] == 0) {
                                    C.push_back(&(FT.list.at(pointIdxRadiusSearch[y])));
                                    Q.push(pointIdxRadiusSearch[y]);
                                    vis[pointIdxRadiusSearch[y]] = 1;
                                }
                            }
                        }
//                        vis[top] = 2;
                    }
                }
                if (C.size() > 5) {
                    cluster.push_back(C);
                }
            }
        }
//        print_cluster(cluster);
        return straightLine(cluster);
    }
    return new vector<vector<Point *>>();
}

float get_avg_bias(vector<Point *> cluster){
    // function : (5927.0371 +/- 78.954) * (x ^(-0.80869 +/- 0.00236))
    float bias = 0;
    for(Point * tmp : cluster){
        float dist = tmp->dist;
        float k = 0;
        if(dist <= 500){
            k = 20;
        }else{
            k = 10;
        }
        float syn = tmp->syn_quality;
        float upbound = (5927.0371 + 78.954) * (pow(dist, -0.80869 + 0.00236));
        float downbound = (5927.0371 - 78.954) * (pow(dist, -0.80869 - 0.00236));
        if(syn <= upbound + k && syn >= downbound - k){
            continue;
        }
        else{
            bias += min(abs(upbound + k - syn), abs(downbound - k - syn));
        }
    }
    return bias / cluster.size();
}

void print_cluster(vector<vector<Point *>> cluster) {
    float dist_x = 0;
    float dist_y = 0;
    float dist_z = 0;
    for (int i = 0; i < cluster.size(); i++) {
        for (int x = 0; x < cluster.at(i).size(); x++) {
            float xa = cluster.at(i).at(x)->dist * sin(cluster.at(i).at(x)->angle * PI / 180);
            float ya = cluster.at(i).at(x)->dist * cos(cluster.at(i).at(x)->angle * PI / 180);
            float za = cluster.at(i).at(x)->syn_quality;
            std::cout << cluster.at(i).at(x)->dist << " " << cluster.at(i).at(x)->angle << " " <<  cluster.at(i).at(x)->flag << " " << cluster.at(i).at(x)->syn_quality << " " << i << endl;
//            std::cout << xa << " "
//                      << ya << " "
//                      << cluster.at(i).at(x)->flag << " " << za
////                      << " " << sqrt(pow(dist_x - xa, 2) + pow(dist_y - ya, 2) + pow((dist_z  - za)/5, 2))
//                      << " " << i << endl;
            dist_x = xa;
            dist_y = ya;
            dist_z = za;
        }
    }
}

// TODO : Speed up
vector<vector<Point *> > *straightLine(vector<vector<Point *>> cluster) {
    vector<vector<Point *> > *result = new vector<vector<Point *> >();
    k.clear();
    int col = 0;
    for (int i = 0; i < cluster.size(); i++) {
        int vis[cluster.at(i).size()];
        for (int y = 0; y < cluster.at(i).size(); y++) {
            vis[y] = 0;
        }
        int count = 0;
        while (cluster.at(i).size() - count > 10) {
            vector<int> forwarding;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2d(new pcl::PointCloud<pcl::PointXYZ>);
            for (int x = 0; x < cluster.at(i).size(); x++) {
                if (vis[x] == 0) {
                    pcl::PointXYZ point;
                    point.x = (float) (sin(cluster.at(i).at(x)->angle * PI / 180) * cluster.at(i).at(x)->dist);
                    point.y = (float) (cos(cluster.at(i).at(x)->angle * PI / 180) * cluster.at(i).at(x)->dist);
                    cloud2d->push_back(point);
                    forwarding.push_back(x);
                }
            }

            pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_line(
                    new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud2d));    //指定拟合点云与几何模型
            pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_line);    //创建随机采样一致性对象
            ransac.setDistanceThreshold(10);    //内点到模型的最大距离
            ransac.setMaxIterations(1000);        //最大迭代次数
            ransac.computeModel();                //执行RANSAC空间直线拟合

            vector<int> inliers;                //存储内点索引的向量
            ransac.getInliers(inliers);            //提取内点对应的索引

            /// 根据索引提取内点
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud<pcl::PointXYZ>(*cloud2d, inliers, *cloud_line);

            /// 模型参数
            Eigen::VectorXf coefficient;
            ransac.getModelCoefficients(coefficient);
//            cout << "直线点向式方程为：\n"
//                 << "   (x - " << coefficient[0] << ") / " << coefficient[3]
//                 << " = (y - " << coefficient[1] << ") / " << coefficient[4]
//                 << " = (z - " << coefficient[2] << ") / " << coefficient[5]
//                 << endl;

//            cout << "k " << coefficient[4] / coefficient[3] << endl;
            k.push_back(coefficient[4] / coefficient[3]);
            vector<Point *> *out_line = new vector<Point *>();
            for (int t = 0; t < inliers.size(); t++) {
                vis[forwarding.at(inliers.at(t))] = 1;
                // TODO : pass all parameters to following solver

                // TODO : output color
                out_line->push_back(cluster.at(i).at(forwarding.at(inliers.at(t))));
//                std::cout << cluster.at(i).at(forwarding.at(inliers.at(t)))->dist << " " << cluster.at(i).at(forwarding.at(inliers.at(t)))->angle << " " <<  cluster.at(i).at(forwarding.at(inliers.at(t)))->flag << " " << cluster.at(i).at(forwarding.at(inliers.at(t)))->syn_quality << " " << col << endl;
                count++;
            }
            // TODO : use point strength to cluster
            // strength_cluster(out_line);
            col++;
            result->push_back(*out_line);
        }
    }
//    cout << "-------------------------------------------------" << endl;
    return result;
}


vector<vector<Point *>> strength_cluster(vector<Point *> line) {
    vector<vector<Point *>> cluster = {};
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2d(new pcl::PointCloud<pcl::PointXYZ>);
    int vis[line.size()];
    for (int i = 0; i < line.size(); i++) {
        pcl::PointXYZ point;
        point.x = (float) (sin(line.at(i)->angle * PI / 180) * line.at(i)->dist);
        point.y = (float) (cos(line.at(i)->angle * PI / 180) * line.at(i)->dist);
        point.z = line.at(i)->syn_quality / 2;
        cloud2d->push_back(point);
        vis[i] = 0;
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud(cloud2d);

    for (int i = 0; i < line.size(); i++) {
        if (vis[i] == 0) {
            std::vector<Point *> *C = new vector<Point *>();
            stack<int> Q;
            Q.push(i);
            while (!Q.empty()) {
                int top = Q.top();
                Q.pop();
                if (vis[top] != 2) {
                    vis[top] = 2;
                    pcl::PointXYZ searchPoint;
                    std::vector<int> pointIdxRadiusSearch;
                    std::vector<float> pointRadiusSquaredDistance;

                    searchPoint.x = (float) (sin(line.at(top)->angle * PI / 180) * line.at(top)->dist);
                    searchPoint.y = (float) (cos(line.at(top)->angle * PI / 180) * line.at(top)->dist);
                    searchPoint.z = line.at(top)->syn_quality / 2;

                    float radius = 10;

                    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) >
                        0) {
                        for (std::size_t y = 0; y < pointIdxRadiusSearch.size(); ++y) {
                            if (vis[pointIdxRadiusSearch[y]] == 0) {
                                C->push_back(line.at(pointIdxRadiusSearch[y]));
                                Q.push(pointIdxRadiusSearch[y]);
                                vis[pointIdxRadiusSearch[y]] = 1;
                            }
                        }
                    }
//                    vis[top] = 2;
                }
            }
            if (C->size() > 5) {
                cluster.push_back(*C);
            }
        }
    }
    return cluster;
}

inline bool comp(Point *p1, Point *p2) {
    return p1->angle < p2->angle;
}

bool SortFunction (Point * i,Point * j) { return (i->x < j->x); }

float get_two_point_dist(Point * a, Point * b){
    float ax = a->dist * sin(a->angle * PI / 180);
    float ay = a->dist * cos(a->angle * PI / 180);
    float bx = b->dist * sin(b->angle * PI / 180);
    float by = b->dist * cos(b->angle * PI / 180);
    return sqrt(pow(ax - bx, 2) + pow(ay - by, 2));
}

vector<vector<Point *> > *find_Features_cluster_tmp(vector<vector<Point *> > *cluster) {
    vector<vector<Point *> > *result = new vector<vector<Point *>>;
    int kidx = 0;
    for (vector<Point *> line : *cluster) {
        std::sort(line.begin(), line.end(), SortFunction);
        double kvalue = k.at(kidx++);
        vector<vector<Point *> > cluster_syn = strength_cluster(line);
        vector<Point *> *tmplist = new vector<Point *>();
        // three stages
        int stages = 0;
        int N = cluster_syn.size();
        vector<int> idx;
        int lastidx = 0;
        float bias = 2;
        for (int i = 0; i < N; i++){
            std::sort(cluster_syn.at(i).begin(), cluster_syn.at(i).end(), SortFunction);
            float syn = get_avg_bias(cluster_syn.at(i));
            if(stages == 0 && syn < bias){
                idx.clear();
                idx.push_back(i);
                lastidx = i;
                stages ++;
            }
            else if(stages == 1){
                if(syn < bias){
                    float dist = get_two_point_dist(cluster_syn.at(idx.at(0)).back(), cluster_syn.at(i).front());
                    if(dist >= 80){
                        i = lastidx + 1;
                        stages = 0;
                        idx.clear();
                        continue;
                    }
                    if(dist <= 30){
                        idx.at(0) = i;
                        continue;
                    }
                    dist = get_two_point_dist(cluster_syn.at(i).front(), cluster_syn.at(i).back());
                    if(dist > 60 || dist < 20){
                        stages = 0;
                        idx.clear();
                        continue;
                    }
                    idx.push_back(i);
                    stages++;
                }
            }
            else if(stages == 2){
                if(syn < bias){
                    float dist = get_two_point_dist(cluster_syn.at(idx.at(1)).back(), cluster_syn.at(i).front());
                    if(dist >= 80){
                        i = lastidx + 1;
                        stages = 0;
                        idx.clear();
                        continue;
                    }
                    if(dist <= 30){
                        idx.at(1) = i;
                        continue;
                    }
                    idx.push_back(i);
                    stages++;
                }
            }
            if(stages == 3){
                // just break the loop
                for (int j = 0; j < idx.size(); ++j) {
                    tmplist->insert(tmplist->end(), cluster_syn.at(j).begin(), cluster_syn.at(j).end());
                }
                sendRosInfo(cluster_syn, kvalue);
                break;
            }
        }
        result->push_back(*tmplist);
    }
    return result;
}

int get_quadrant(double x, double y){
    if(x >= 0 && y >= 0){
        return 1;
    }else if(x <= 0 && y >= 0){
        return 2;
    }else if(x <= 0 && y <= 0){
        return 3;
    }else if(x >= 0 && y <= 0){
        return 4;
    }
}

void sendRosInfo(vector<vector<Point *> > cluster_syn, double K){
    vector<Point *> mid = cluster_syn.at(1);
    int mid_len = mid.size();
    double dis = (mid.at(0)->dist + mid.at(mid_len - 1)->dist) / 2;
    double angle = (mid.at(0)->angle + mid.at(mid_len - 1)->angle) / 2;
    double x = cos(angle * PI / 180) * dis / 1000;
    double y = sin(angle * PI / 180) * dis / 1000;
    // use for not transfrom
    double x_tmp = sin(angle * PI / 180) * dis / 1000;
    double y_tmp = cos(angle * PI / 180) * dis / 1000;
    //  TODO :  use tf
    tf::TransformListener listener;
    tf::StampedTransform transform;
    //    try{
    //        listener.lookupTransform("/map", "/odom",ros::Time(0), transform);
    //    }catch (tf::TransformException &ex) {
    //        ROS_ERROR("%s",ex.what());
    //        return;
    //    }
    geometry_msgs::PoseStamped pose_odom;
    pose_odom.header.stamp = ros::Time();
    pose_odom.header.frame_id = "base_link";

    geometry_msgs::PoseStamped pose_map;

    double angleFromK = atan(-1.0/K);
//    cout << "X is " << x << " Y is " << y << " angle is " << angle << endl;
//    cout << "angle before transform " << angleFromK * 180 / PI << endl;

    if(x_tmp > 0){
        if(angleFromK < 0){
            angleFromK += 2 * PI;
        }
    }else{
        angleFromK += PI;
    }

//    cout << "angle after transform " << angleFromK * 180 / PI << endl;
    // Test for angle
    double angleAftertf = angleFromK + PI/2;
    if(angleAftertf > 2 * PI) angleAftertf -= 2 * PI;
    angleAftertf = PI - angleAftertf;
    if(angleAftertf < 0)angleAftertf += 2 * PI;
//    cout << "angle after transform tf " << angleAftertf * 180 / PI << endl;
    // Test for angle end
    Eigen::Quaterniond quaterniontf = Eigen::AngleAxisd(angleAftertf, Eigen::Vector3d::UnitZ()) *
                                      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
    pose_odom.pose.position.x = x;
    pose_odom.pose.position.y = y;
    pose_odom.pose.position.z = 0;
    pose_odom.pose.orientation.x = quaterniontf.x();
    pose_odom.pose.orientation.y = quaterniontf.y();
    pose_odom.pose.orientation.z = quaterniontf.z();
    pose_odom.pose.orientation.w = quaterniontf.w();

    try {
        listener.waitForTransform("/base_link", "/map", ros::Time(0), ros::Duration(3.0));
        listener.transformPose("map", pose_odom, pose_map);
    }
    catch (tf::TransformException ex) {
        ROS_WARN("transfrom exception : %s", ex.what());
        return;
    }
    // send pos
    if (ros::ok()) {
        cout << "send!" << endl;
//        cout << pose_map.pose.orientation << endl;
        angle_dist.publish(pose_map);
    }
    // TODO : use tf end

    // TODO ： 30 cm test start
//    double delta_x = abs(0.3 * cos(angleAftertf));
//    double delta_y = abs(0.3 * sin(angleAftertf));
//    int quadrant = get_quadrant(x, y);
////    cout << "x is " << x << " y is " << y << endl;
////    cout << "delta_x is " << delta_x << " delta_y is " << delta_y << endl;
//    if(quadrant == 1){
//        pose_odom.pose.position.x = x - delta_x;
//        pose_odom.pose.position.y = y - delta_y;
////        cout << "x is " << x - delta_x << " y is " << y - delta_y << endl;
//    }else if(quadrant == 2){
//        pose_odom.pose.position.x = x + delta_x;
//        pose_odom.pose.position.y = y - delta_y;
////        cout << "x is " << x + delta_x << " y is " << y - delta_y << endl;
//    }else if(quadrant == 3){
//        pose_odom.pose.position.x = x + delta_x;
//        pose_odom.pose.position.y = y + delta_y;
////        cout << "x is " << x + delta_x << " y is " << y + delta_y << endl;
//    }else if(quadrant == 4){
//        pose_odom.pose.position.x = x - delta_x;
//        pose_odom.pose.position.y = y + delta_y;
////        cout << "x is " << x - delta_x << " y is " << y + delta_y << endl;
//    }
//    // TODO : join together
//    try {
//        listener.waitForTransform("/base_link", "/map", ros::Time(0), ros::Duration(3.0));
//        listener.transformPose("map", pose_odom, pose_map);
////        cout << "x is " << pose_odom.pose.position.x << " y is " << pose_odom.pose.position.y << endl;
//    }
//    catch (tf::TransformException ex) {
//        ROS_WARN("transfrom exception : %s", ex.what());
//        return;
//    }
    // TODO : 30 cm test end
//    Eigen::Quaterniond q1 = Eigen::Quaterniond(nowOrientation.w, nowOrientation.x, nowOrientation.y,
//                                               nowOrientation.z).normalized();
//    Eigen::Quaterniond q2 = Eigen::Quaterniond(1, 0, 0, 0).normalized();
//
//    Eigen::Vector3d t1 = Eigen::Vector3d(nowP.x, nowP.y, 0);
//    Eigen::Vector3d t2 = Eigen::Vector3d(0, 0, 0);
//
//    Eigen::Vector3d p1 = Eigen::Vector3d(x, y, 0);
//    Eigen::Vector3d p2;
//
//    p2 = q2 * q1.inverse() * (p1 - t1) + t2;
////    cout << "dis is " << dis << " angle is " << angle << endl;
////    cout << "x is " << x << " y is " << y << endl;
////    cout << "now P is " << nowP.x << " " << nowP.y << endl;
//    cout << p2.transpose() << endl;
//    cout << "K angle is " << atan(K) / PI * 180 << endl;
//    cout << "Euler angle is " << q1.toRotationMatrix().eulerAngles(2, 1, 0)[0] / PI * 180 << endl;
//    cout << "angle is " << atan(K) / PI * 180 + q1.toRotationMatrix().eulerAngles(2, 1, 0)[0] / PI * 180 << endl;
//
//    Eigen::Quaterniond quaternion =
//            Eigen::AngleAxisd(atan(K) + q1.toRotationMatrix().eulerAngles(2, 1, 0)[0], Eigen::Vector3d::UnitZ()) *
//            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
//            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());
//
//    if (ros::ok()) {
//        geometry_msgs::PoseStamped msg;
//        msg.pose.position.x = p2.x();
//        msg.pose.position.y = p2.y();
//        msg.pose.position.z = p2.z();
//        msg.pose.orientation.x = quaternion.x();
//        msg.pose.orientation.y = quaternion.y();
//        msg.pose.orientation.z = quaternion.z();
//        msg.pose.orientation.w = quaternion.w();
//        cout << "send!" << endl;
//        angle_dist.publish(msg);
//    }
}