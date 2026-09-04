#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <random>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Fast-tracker Random Forest & Obstacle Map Generator
// Derived from ZJU FAST-Lab (Fast-tracker / Fast-Planner)
// Publishes sensor_msgs/PointCloud2 obstacle map for RViz and APF testing

ros::Publisher _all_map_pub;

int _obs_num;
double _x_size, _y_size, _z_size;
double _x_l, _x_h, _y_l, _y_h, _w_l, _w_h, _h_l, _h_h;
double _resolution;
double _sensing_rate;

pcl::PointCloud<pcl::PointXYZ> cloud_all;

void pubMap() {
    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(cloud_all, map_msg);
    map_msg.header.frame_id = "world";
    map_msg.header.stamp = ros::Time::now();
    _all_map_pub.publish(map_msg);
}

void generateRandomForest() {
    cloud_all.clear();

    std::random_device rd;
    std::default_random_engine eng(rd());

    std::uniform_real_distribution<double> rand_x(_x_l, _x_h);
    std::uniform_real_distribution<double> rand_y(_y_l, _y_h);
    std::uniform_real_distribution<double> rand_w(_w_l, _w_h);
    std::uniform_real_distribution<double> rand_h(_h_l, _h_h);

    // Generate random cylindrical obstacles (tree trunks / pillars)
    for (int i = 0; i < _obs_num; ++i) {
        double x = rand_x(eng);
        double y = rand_y(eng);
        double w = rand_w(eng); // radius
        double h = rand_h(eng); // height

        // Keep origin (0,0) clear for takeoff
        if (sqrt(x * x + y * y) < 2.0) continue;

        for (double z = 0; z <= h; z += _resolution) {
            for (double theta = 0; theta < 2 * M_PI; theta += _resolution / w) {
                pcl::PointXYZ pt;
                pt.x = x + w * cos(theta);
                pt.y = y + w * sin(theta);
                pt.z = z;
                cloud_all.points.push_back(pt);
            }
        }
    }

    cloud_all.width = cloud_all.points.size();
    cloud_all.height = 1;
    cloud_all.is_dense = true;
    ROS_INFO("[MapGenerator] Generated random forest with %zu points.", cloud_all.points.size());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "random_forest_sensing");
    ros::NodeHandle nh("~");

    nh.param("map/obs_num", _obs_num, 30);
    nh.param("map/x_size", _x_size, 20.0);
    nh.param("map/y_size", _y_size, 20.0);
    nh.param("map/z_size", _z_size, 5.0);
    nh.param("map/circle_num", _obs_num, 30);
    nh.param("map/resolution", _resolution, 0.1);
    nh.param("sensing/rate", _sensing_rate, 1.0);

    _x_l = -_x_size / 2.0;
    _x_h =  _x_size / 2.0;
    _y_l = -_y_size / 2.0;
    _y_h =  _y_size / 2.0;
    _w_l = 0.2;
    _w_h = 0.8;
    _h_l = 1.0;
    _h_h = _z_size;

    _all_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);

    generateRandomForest();

    ros::Rate loop_rate(_sensing_rate);
    while (ros::ok()) {
        pubMap();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
