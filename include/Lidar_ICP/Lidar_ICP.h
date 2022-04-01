#ifndef LIDAR_ICP
#define LIDAR_ICP

#include <math.h>
#include <string>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using namespace std;
using namespace Eigen;

class Lidar_ICP
{
public:
    Lidar_ICP();
    ~Lidar_ICP();
    void point_base_matching(const vector<vector<Vector2f>> &point_pairs);
    void ICP();
    void scan2_callback(const sensor_msgs::LaserScanConstPtr &data);
    void scan1_callback(const sensor_msgs::LaserScanConstPtr &data);
    void print_rotation();
    void print_trans();
    void print_T();
    double get_transx();
    double get_transy();
    double get_rotation();

private:
protected:
    ros::NodeHandle nh_;
    ros::Subscriber scan1_sub_;
    ros::Subscriber scan2_sub_;
    std::vector<Vector2f> scan1_poses_;
    std::vector<Vector2f> scan2_poses_;
    bool ready1_;
    bool ready2_;
    Matrix2d rotation_;
    Vector2d trans_;
    Matrix3d T_;
};

#endif
