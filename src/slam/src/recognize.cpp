//
// Created by bill on 22-5-10.
//
#include <cstdio>
#include <cmath>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "slam/pos.h"
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <stack>

#define PI 3.14159

using namespace std;
const int INF = 0x3f3f3f3f;
const int len_board = 250;
const int point_num = 500;

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

void callback_func(const sensor_msgs::LaserScan &msg);
//vector<cv::Point2d> points = {};
pcl::PointCloud<pcl::PointXY>::Ptr cloud(new pcl::PointCloud<pcl::PointXY>);

float calc_dist(int dist_1, float angle_1, int dist_2, float angle_2);

vector<vector<Point *> > *calc_knn(float dist, float angle, int flag, int syn_quality);

void print_cluster(vector<vector<Point *>> cluster);

vector<vector<Point *> > *straightLine(vector<vector<Point *>> cluster);

vector<vector<Point *> > strength_cluster(vector<Point *> line);

vector<vector<Point *> > *find_Features_simple(vector<vector<Point *> > *cluster);

vector<vector<Point *> > *find_Features_cluster(vector<vector<Point *> > *cluster);

vector<vector<Point *> > *find_Features_cluster_tmp(vector<vector<Point *> > *cluster);

void sendRosInfo(vector<Point *> *tmplist, vector<vector<Point *> > cluster_syn);

// set to global
ros::Publisher angle_dist;

int main(int argc, char** argv) {
    ros::init(argc, argv, "recognize_rcv"); //初始化订阅者节点
    ros::NodeHandle n;
    ros::Subscriber ros_tutorial_sub = n.subscribe("scan", 1000, callback_func);
    angle_dist = n.advertise<slam::pos>("/pos/angle_dist", 10);
    ros::spin();
    return 0;
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
            print_cluster(*result);
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

inline bool check_consecutive(vector<Point *> line) {
    sort(line.begin(), line.end(), comp);
    float k = 2; // parameters
    return abs(line.front()->angle - line.back()->angle) > line.size() * k;
}

inline int get_average_dist(vector<Point *> line) {
    int sum_dist = 0;
    for (Point *p: line) {
        sum_dist += p->dist;
    }
    return (int) sum_dist / line.size();
}

inline int get_average_str(vector<Point *> line) {
    int sum_str = 0;
    for (Point *p: line) {
        sum_str += p->syn_quality;
    }
    return (int) sum_str / line.size();
}

inline int get_average_str_weight_back(vector<Point *> line) {
    float sum_str = 0;
    float weight = 1.0 / line.size();
    for (Point *p: line) {
        sum_str += p->syn_quality * weight;
        weight += 1.0 / line.size();
    }
    return (int) (2 * sum_str / line.size());
}

inline int get_average_str_front(vector<Point *> line) {
    float sum_str = 0;
    float weight = 1.0;
    for (Point *p: line) {
        sum_str += p->syn_quality * weight;
        weight -= 1.0 / line.size();
    }
    return (int) (2 * sum_str / line.size());
}

vector<vector<Point *> > *find_Features_simple(vector<vector<Point *> > *cluster) {
    vector<vector<Point *> > *result = new vector<vector<Point *>>();
    for (vector<Point *> line: *cluster) {
        if (check_consecutive(line))
            continue;

        int N = line.size();
        int radii_syn = 10;

        int sum_syn_quality = 0;
        int sum_dist = 0;
        int stage = 1;
        int s = 0;
        int syn_avg[5];
        int dist_avg[5];
        for (int i = 0; i < N; i++) {
            Point *p = line.at(i);
            sum_syn_quality += p->syn_quality;
            sum_dist += p->dist;
            s++;
            if (i == N * stage / 5 - 1) {
                syn_avg[stage - 1] = (int) sum_syn_quality / s;
                dist_avg[stage - 1] = (int) sum_dist / s;
                sum_syn_quality = 0;
                sum_dist = 0;
                stage++;
                s = 0;
            }
        }

        //强-弱-强-弱-强
        if (!(syn_avg[1] < syn_avg[0] && syn_avg[1] < syn_avg[2] && syn_avg[1] < syn_avg[4] &&
              syn_avg[3] < syn_avg[0] && syn_avg[3] < syn_avg[2] && syn_avg[3] < syn_avg[4])) {
            continue;
        }

        // 强-强-强之间不能差距太大
        if (2 * syn_avg[2] < (syn_avg[0] + syn_avg[4] - radii_syn) ||
            2 * syn_avg[2] > (syn_avg[0] + syn_avg[4] + radii_syn)) {
            continue;
        }

        //长度不能超过该距离下最长的长度
        int avg_dist = 0;
        for (int i = 0; i < 5; i += 2) {
            avg_dist += dist_avg[i];
        }
        avg_dist /= 3;
        int num_point_max = (int) 1.1 * 2 * atan(len_board / (2 * avg_dist));
        if (N > num_point_max) {
            continue;
        }
        result->push_back(line);
    }
    return result;
}

bool SortFunction (Point * i,Point * j) { return (i->x < j->x); }

vector<vector<Point *> > *find_Features_cluster(vector<vector<Point *> > *cluster) {
    vector<vector<Point *> > *result = new vector<vector<Point *>>;
    for (vector<Point *> line: *cluster) {
        if (check_consecutive(line)) {
            continue;
        }

        // int N = line.size();
        int radii_syn = 10;

        vector<vector<Point *> > cluster_syn = strength_cluster(line);
        print_cluster(cluster_syn);
        cout << "-----------------------------------------------------" << endl;
        int N = cluster_syn.size();
        int dist_avg[N], syn_avg[N], line_size[N];
        for (int i = 0; i < N; i++) {
            std::sort(cluster_syn.at(i).begin(), cluster_syn.at(i).end(), SortFunction);
            dist_avg[i] = get_average_dist(cluster_syn[i]);
            syn_avg[i] = get_average_dist(cluster_syn[i]);
            line_size[i] = cluster_syn.at(i).at(0)->angle;
        }

        for (int i = 0; i < N; i++) {
            int last_idx = i;
//            int max_size = line_size[i] * 10;
            float atansize = 2 * atan((double)len_board / (double)(2 * dist_avg[i])) * 180 / PI;
            int max_size = (int) 1.2 * atansize;
            int min_size = (int) 0.8 * atansize;
            int total_size = 0;
            while (last_idx < N) {
                total_size += line_size[last_idx++];
                if (total_size >= max_size) {
                    last_idx--;
                    break;
                }
            }
            if (total_size < min_size) {
                continue;
            }

            for (int j = i + 1; j < last_idx; j++) {
                for (int k = j + 1; k <= last_idx; k++) {
                    //强-弱-强-弱-强
//                    if (!(syn_avg[i] < syn_avg[0] && syn_avg[1] < syn_avg[2] && syn_avg[1] < syn_avg[4] &&
//                          syn_avg[3] < syn_avg[0] && syn_avg[3] < syn_avg[2] && syn_avg[3] < syn_avg[4])) {
//                        continue;
//                    }

                    // check length between (0.6, 1.4)
                    if (line_size[i] + line_size[k] > line_size[j] * 2 * 1.4 ||
                        line_size[i] + line_size[k] < line_size[j] * 2 * 0.6) {
                        continue;
                    }
                    //check average distance
                    if (dist_avg[i] + dist_avg[k] > dist_avg[j] * 2 * 1.3 ||
                        dist_avg[i] + dist_avg[k] < dist_avg[j] * 2 * 0.7) {
                        continue;
                    }
                    // check average strength
                    if (syn_avg[i] + syn_avg[k] > syn_avg[j] * 2 * 1.3 ||
                        syn_avg[i] + syn_avg[k] < syn_avg[j] * 2 * 0.7) {
                        continue;
                    }

                    vector<Point *> *feature = new vector<Point *>();
                    for (int ii = i; ii <= k; ii++) {
                        for (Point *p: cluster_syn[ii]) {
                            feature->push_back(p);
                        }
                        Point * p = new Point();
                    }
                    result->push_back(*feature);
                }
            }
        }
    }
    return result;
}

float get_two_point_dist(Point * a, Point * b){
    float ax = a->dist * sin(a->angle * PI / 180);
    float ay = a->dist * cos(a->angle * PI / 180);
    float bx = b->dist * sin(b->angle * PI / 180);
    float by = b->dist * cos(b->angle * PI / 180);
    return sqrt(pow(ax - bx, 2) + pow(ay - by, 2));
}



vector<vector<Point *> > *find_Features_cluster_tmp(vector<vector<Point *> > *cluster) {
    vector<vector<Point *> > *result = new vector<vector<Point *>>;
    for (vector<Point *> line : *cluster) {
        std::sort(line.begin(), line.end(), SortFunction);
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
                sendRosInfo(tmplist, cluster_syn);
                break;
            }
        }
        result->push_back(*tmplist);
    }
    return result;
}

void sendRosInfo(vector<Point *> *tmplist, vector<vector<Point *> > cluster_syn){
    if(ros::ok()){
        slam::pos msg;
        vector<float> dist;
        vector<float> angle;
        vector<float> dist_mid;
        vector<float> angle_mid;
        for(int i = 0; i < tmplist->size(); i++){
            dist.push_back(tmplist->at(i)->dist);
            angle.push_back(tmplist->at(i)->angle);
        }
        for(int i = 0; i < cluster_syn.at(1).size(); i++){
            dist_mid.push_back(cluster_syn.at(1).at(i)->dist);
            angle_mid.push_back(cluster_syn.at(1).at(i)->angle);
        }
        msg.dist = dist;
        msg.angle = angle;
        msg.dist_mid = dist_mid;
        msg.angle_mid = angle_mid;
        angle_dist.publish(msg);
    }
}