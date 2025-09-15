//
// Created by zfg on 25-9-12.
//
#include "PointMassPathSearching.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <deque>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


int main(int argc, char **argv) {
    ros::init(argc, argv, "pmps_test_node");
    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/generated_trajectory", 10, true);


    const double max_t = 20.0; // Maximum acceleration (m/s^2)
    const int num_waypoints = 7; // Number of waypoints to define the circle
    const double initial_z = 2.0; // Constant altitude for the track

    // --- Initial State ---
    Eigen::Vector3d start_position(0.0, 0.0, initial_z);
    Eigen::Vector3d start_velocity(0.0, 0.0, 0.0);
    // Attitude as a quaternion (w, x, y, z) - representing no rotation
    Eigen::Vector4d start_attitude(1.0, 0.0, 0.0, 0.0);

    // --- Generate Circular Track Waypoints ---
    std::deque<waypoint> circular_track;

    // The first waypoint is the starting position
    circular_track.emplace_back(start_position, start_velocity, 0);

    // Generate points on a circle starting from the initial position
    for (int i = 1; i <= num_waypoints; ++i) {
        double angle = (2.0 * M_PI * i) / num_waypoints;
        Eigen::Vector3d point(
                25 * (1 - cos(angle)),
                25 * sin(angle),
                initial_z
        );
        Eigen::Vector3d direction(sin(angle), cos(angle), 1.0);
//        Eigen::Vector3d direction(1,0, 1.0);
        direction.normalize();
        circular_track.emplace_back(point,direction, i);
    }

    // --- Trajectory Planning ---
    ROS_INFO("Setting up PointMassPathSearching solver...");
    PointMassPathSearching pmps{
            1.0 * max_t, 1.0 * max_t, max_t,
            -1.0 * max_t, -1.0 * max_t, -max_t,
            static_cast<int>(circular_track.size())
    };

    Trajectories::Trajectories trajectory_data;
    bool fix_replan = false; // Perform a full search, not a fixed replan

    ROS_INFO("Solving for the optimal trajectory...");
    auto start_time = std::chrono::system_clock::now();
    // 求解轨迹，传入track的航点数可以更改
    //PointMassPathSearching::normSample里可以更改采样速度大小的定义范围
    // 当前的速度方向严格按照给定的direction，不进行速度方向的采样和搜索
    pmps.solve(circular_track, start_position, start_velocity, start_attitude, fix_replan);
    // 按照给定时间步长进行采样
    pmps.drawTrajectory(0.03); // Generate trajectory points with a 0.03s timestep
    // 返回一个列表，按时间步存放[posx,posy,posz,velx,vely,velz]
    pmps.getPointMassTrajectory(&trajectory_data);

    auto end_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_time - start_time;
    ROS_INFO("Trajectory calculation finished in %.4f seconds.", elapsed_seconds.count());
    std::deque<std::vector<double>> trajectory = trajectory_data.get_full_trajectories_vec();
    // --- Convert Trajectory to ROS Path Message ---
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map"; // Or "odom", depending on your RViz fixed frame

    for (const auto& point : trajectory) {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = path_msg.header.stamp;
            pose.header.frame_id = path_msg.header.frame_id;
            pose.pose.position.x = point.at(0);
            pose.pose.position.y = point.at(1);
            pose.pose.position.z = point.at(2);
            pose.pose.orientation.w = 1.0; // Default orientation
            path_msg.poses.push_back(pose);
    }

    // --- Publish and Wait ---
    if (path_msg.poses.empty()) {
        ROS_WARN("No trajectory points were generated!");
    } else {
        path_pub.publish(path_msg);
        ROS_INFO("Published trajectory with %zu points. Visualize in RViz on topic /generated_trajectory", path_msg.poses.size());
    }

    ros::spin(); // Keep the node alive to ensure the message is received

    return 0;
}